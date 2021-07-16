//! Trajectory control module state

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// Internal
use super::*;
use crate::auto::{loc::Pose, path::*};
use comms_if::tc::loco_ctrl::MnvrCmd;
use log::{info, warn};
use nalgebra::{Vector2, Vector3};
use serde::{Deserialize, Serialize};
use util::{params, session};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

#[derive(Debug, Serialize, Clone)]
pub struct TrajCtrl {
    #[serde(skip)]
    params: Params,

    /// Executing mode
    mode: TrajCtrlMode,

    input_pose: Option<Pose>,
    output_mnvr_cmd: Option<MnvrCmd>,
    report: StatusReport,

    /// The sequence of paths to execute.
    path_sequence: Option<Vec<Path>>,

    /// Index of the current executing path
    path_index: usize,

    /// Index of the current target point within the path
    target_point_index: usize,

    /// Controller objects used to calculate manouvre commands
    controllers: TrajControllers,

    /// Output used to tune the system
    tuning_output: TrajCtrlTuningOutput,
}

/// The status report containing various error flags and monitoring quantities.
#[derive(Default, Copy, Clone, Debug, Serialize, Deserialize)]
pub struct StatusReport {
    /// The lateral error to the current path segment
    pub lat_error_m: f64,

    /// Longitudonal error from the current target point
    pub long_error_m: f64,

    /// The heading error to the current path segment
    pub head_error_rad: f64,

    /// If true the limit on the lateral error has been exceeded
    pub lat_error_limit_exceeded: bool,

    /// If true the limit on the heading error has been exceeded
    pub head_error_limit_exceeded: bool,

    /// True if the requested path sequence has finished
    pub sequence_finished: bool,

    /// True if the sequence has been aborted
    pub sequence_aborted: bool,

    pub target_point_idx: usize,
}

#[derive(Default, Clone, Debug, Serialize)]
pub struct TrajCtrlTuningOutput {
    /// The time
    pub time_s: f64,

    /// The lateral error to the current path segment
    pub lat_error_m: f64,

    /// Longitudonal error from the current target point
    pub long_error_m: f64,

    /// The heading error to the current path segment
    pub head_error_rad: f64,

    /// Output from the lateral error controller
    pub lat_ctrl: f64,

    /// Output from the heading error controller
    pub head_ctrl: f64,
}

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Potential errors that can occur during processing of the module.
#[derive(Debug, thiserror::Error)]
pub enum TrajCtrlError {
    #[error("Could not load parameters: {0}")]
    ParamLoadError(params::LoadError),

    /// A sequence is already loaded. This error occurs when attempting to start a new sequence
    /// before the current one has finished.
    #[error("Attempted to load a path sequence while one is already loaded")]
    SequenceAlreadyLoaded,

    /// Attempted to load an empty sequence. This error occurs if the sequence to be loaded doesn't
    /// have any paths in it.
    #[error("Attempted to load empty path sequence")]
    AttemptEmptySeqLoad,

    /// Attempted to load a sequence containing invalid paths. The contained vector provides the
    /// indices of the paths which were invalid.
    #[error("Loaded sequence contains invalid paths at index(s) {0:?}")]
    SequenceContainsInvalidPaths(Vec<usize>),

    /// Attempted to follow an empty path sequence. This error occurs if the module is in follow
    /// path mode but there is no sequence loaded.
    #[error("No path sequence has been set")]
    NoSequence,

    /// Attempted to control trajectory when the pose is not known.
    #[error("No pose has been set")]
    NoPose,
}

/// The possible modes of execution of TrajCtrl. Each mode is handled by a
/// `mode_xyz` function.
#[derive(Debug, Copy, Clone, Serialize)]
pub enum TrajCtrlMode {
    Off,
    FollowPath,
    HeadingAdjust,
    SequenceFinished,
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl TrajCtrl {
    /// Intiailise the TrajCtrl module.
    ///
    /// Expected init data is a path to the parameter file.
    pub fn init(params_path: &str) -> Result<Self, TrajCtrlError> {
        // Load the parameters
        let params = match params::load(params_path) {
            Ok(p) => p,
            Err(e) => return Err(TrajCtrlError::ParamLoadError(e)),
        };

        Ok(Self::new(params))
    }

    pub fn new(params: Params) -> Self {
        // Initialise the controllers
        let controllers = TrajControllers::new(&params);

        // Build self
        Self {
            params,
            mode: TrajCtrlMode::Off,
            controllers,
            input_pose: None,
            output_mnvr_cmd: None,
            report: StatusReport::default(),
            path_sequence: None,
            path_index: 0,
            target_point_index: 0,
            tuning_output: TrajCtrlTuningOutput::default(),
        }
    }

    /// Process trajectory control.
    ///
    /// Processing involves:
    ///  1. Get the current executing path segment (incrementing the target if
    ///     required.)
    ///  1. Calculating LocoCtrl command based on current position and path
    pub fn proc(&mut self, pose: &Pose) -> Result<(Option<MnvrCmd>, StatusReport), TrajCtrlError> {
        // Setup cycle data
        self.input_pose = Some(*pose);
        self.output_mnvr_cmd = None;
        self.report = StatusReport::default();
        self.tuning_output.time_s = session::get_elapsed_seconds();

        // Mode execution. Each of the mode functions returns either the mode
        // to switch to or an error
        match self.mode {
            TrajCtrlMode::Off => self.mode_off(),
            TrajCtrlMode::FollowPath => self.mode_follow_path(),
            TrajCtrlMode::HeadingAdjust => self.mode_head_adjust(),
            TrajCtrlMode::SequenceFinished => self.mode_seq_finished(),
        }?;

        Ok((self.output_mnvr_cmd, self.report))
    }

    /// Begin executing a path sequence.
    ///
    /// This function will load a new path sequence into TrajCtrl. Execution
    /// will begin on the next call to `proc`. This function must be called
    /// before calling `proc` in order to begin executing the path in the same
    /// cycle.
    ///
    /// In the next call to `proc` the module will be in `HeadingAdjust` mode,
    /// in order to line up the rover with the first segment of the path.
    ///
    /// Loading a new sequence before the current one has been finished will
    /// result in an error. To stop a path sequence whilst it's executing you
    /// must call `abort_path_sequence`.
    pub fn begin_path_sequence(&mut self, seq: Vec<Path>) -> Result<(), TrajCtrlError> {
        // Check to see if there's already a sequence loaded
        if self.path_sequence.is_some() {
            return Err(TrajCtrlError::SequenceAlreadyLoaded);
        }

        // Verify that the new sequence contains at least one path
        if seq.is_empty() {
            return Err(TrajCtrlError::AttemptEmptySeqLoad);
        }

        // Check that all paths in the sequence are valid
        let mut invalid_path_indexes: Vec<usize> = Vec::new();
        for (i, path) in seq.iter().enumerate() {
            if path.get_num_points() < 2 {
                invalid_path_indexes.push(i);
            }
        }

        // If there were invalid paths
        if !invalid_path_indexes.is_empty() {
            return Err(TrajCtrlError::SequenceContainsInvalidPaths(
                invalid_path_indexes,
            ));
        }

        // Setup counters and sequence. The target must be 1 not 0 as a segment
        // is defined backwards, i.e. between the target and previous points.
        self.path_sequence = Some(seq);
        self.path_index = 0;
        self.target_point_index = 1;

        // Switch into Heading Adjust mode.
        self.mode = TrajCtrlMode::HeadingAdjust;

        Ok(())
    }

    /// Abort the currently executing path sequence.
    ///
    /// This will transfer the mode into sequence finished so that on the next
    /// call to `proc` a stop command is issued and the path sequence cleared.
    pub fn abort_path_sequence(&mut self) -> Result<(), TrajCtrlError> {
        // If there's already a loaded path exit, otherwise don't do anything
        if self.path_sequence.is_some() {
            self.mode = TrajCtrlMode::SequenceFinished;
        }

        Ok(())
    }

    /// Mode not executing.
    ///
    /// No actions are taken in this mode. To move from NotExec to
    /// HeadingAdjust the user must call `begin_path_sequence`.
    fn mode_off(&mut self) -> Result<(), TrajCtrlError> {
        Ok(())
    }

    /// Mode following path
    ///
    /// In this mode TrajCtrl will command LocoCtrl to execute the current path.
    fn mode_follow_path(&mut self) -> Result<(), TrajCtrlError> {
        // Validate sequence
        let path_seq = match self.path_sequence {
            Some(ref s) => s.clone(),
            None => return Err(TrajCtrlError::NoSequence),
        };

        // Validate pose
        let pose = match self.input_pose {
            Some(p) => p,
            None => return Err(TrajCtrlError::NoPose),
        };

        // ---- TARGET MANAGEMENT ----

        // Find longitudonal error to next target
        let long_err_m = self.get_long_error()?;

        self.tuning_output.long_error_m = long_err_m;

        // If the error is negative we have passed the target, so increment to
        // the next target
        if long_err_m < 0f64 {
            self.target_point_index += 1;
        }

        // If the target point index is at or greater than the length of the
        // path then switch to the next path sequence.
        if self.target_point_index >= path_seq[self.path_index].get_num_points() {
            self.path_index += 1;
            self.target_point_index = 0;
        }

        self.report.target_point_idx = self.target_point_index;

        // If the path index is now greater than the length of the sequence
        // switch to the squence finished mode and leave execution here.
        if self.path_index >= path_seq.len() {
            self.mode = TrajCtrlMode::SequenceFinished;
            info!("Sequence finished");
            return Ok(());
        }

        // ---- COMMAND GENERATION ----

        // Get the current path segment
        //
        // Can safely unwrap here as target management already performed
        let segment = path_seq[self.path_index]
            .get_segment_to_target(self.target_point_index)
            .unwrap();

        // Get the command
        let mnvr_cmd = self.controllers.get_ackerman_cmd(
            &segment,
            &pose,
            &mut self.report,
            &mut self.tuning_output,
            &self.params,
        );

        // Check for error exceedance
        if self.report.lat_error_limit_exceeded || self.report.head_error_limit_exceeded {
            warn!("Limits exceeded:");
            info!("Lateral error = {} m", self.report.lat_error_m);
            info!("Heading error = {} rad", self.report.head_error_rad);

            self.report.sequence_aborted = true;

            // Switch to sequence exceeded mode immediately so that we are
            // stopped as close to the path as possible.
            self.mode = TrajCtrlMode::SequenceFinished;
            self.mode_seq_finished()?;
        } else {
            // Set the command in the output data
            self.output_mnvr_cmd = Some(mnvr_cmd);
        }

        Ok(())
    }

    /// Mode heading adjustment.
    ///
    /// In this mode TrajCtrl will command LocoCtrl to execute a point turn to
    /// align the rover with the current path segment.
    ///
    /// This is done between different paths in a sequence in case of
    /// discontinuity in the paths, or if the heading error becomes too great
    /// to be corrected by follow path.
    fn mode_head_adjust(&mut self) -> Result<(), TrajCtrlError> {
        // Validate sequence
        let path_seq = match self.path_sequence {
            Some(ref s) => s,
            None => return Err(TrajCtrlError::NoSequence),
        };

        // Validate pose
        let pose = match self.input_pose {
            Some(p) => p,
            None => return Err(TrajCtrlError::NoPose),
        };

        // Start by getting the segment we want to align to
        let segment = path_seq[self.path_index]
            .get_segment_to_target(self.target_point_index)
            .unwrap();

        // Get the vector pointing in the pose direction (in 2D)
        let pose_dir = pose.forward2();

        // Heading error is then the angle between the two direction vectors
        let head_err_rad = segment.direction.angle(&pose_dir);

        // If the error is less than the threshold the adjustment is complete
        if head_err_rad.abs() < self.params.head_adjust_threshold_rad {
            // Issue a stop command and switch to follow path mode
            self.output_mnvr_cmd = Some(MnvrCmd::Stop);

            self.mode = TrajCtrlMode::FollowPath;
        } else {
            // To determine which way to steer we need to know if the segment is to the left or
            // right of the rover, which we can do using the cross product. If we take the 2D and
            // extend them to 3D (with z = 0), z will be positive if the segment is to the left,
            // and negative if to the right. We therefore take the sign of the cross.z and multiply
            // it by our speed, giving us the rotation direction (speed is +ve ccw about Z)
            let cross = Vector3::new(pose_dir[0], pose_dir[1], 0.0).cross(&Vector3::new(
                segment.direction[0],
                segment.direction[1],
                0.0,
            ));

            // If we're within the fine threshold drive at the fine rate, otherwise drive at the
            // coarse rate.
            if head_err_rad.abs() < self.params.head_adjust_fine_threshold_rad {
                self.output_mnvr_cmd = Some(MnvrCmd::PointTurn {
                    rate_rads: cross[0].signum() * self.params.head_adjust_fine_rate_rads,
                });
            } else {
                self.output_mnvr_cmd = Some(MnvrCmd::PointTurn {
                    rate_rads: cross[0].signum() * self.params.head_adjust_coarse_rate_rads,
                });
            }
        }

        Ok(())
    }

    /// Mode sequence finished.
    ///
    /// This mode is run when all the paths in the current sequence have been
    /// completed and the rover should come to a full stop. It will set the
    /// output command to `Stop` and clear the path sequence.
    fn mode_seq_finished(&mut self) -> Result<(), TrajCtrlError> {
        // Set the stop command
        self.output_mnvr_cmd = Some(MnvrCmd::Stop);

        // Clear the path sequence
        self.path_sequence = None;
        self.path_index = 0;
        self.target_point_index = 0;

        // Set the sequence as finished
        self.report.sequence_finished = true;

        // Switch to NotExecuting mode
        self.mode = TrajCtrlMode::Off;

        // Return
        Ok(())
    }

    /// Get the longitudonal error to the current path segment.
    ///
    /// Positive errors indiciate that the rover hasn't reached the target yet.
    /// Negative errors indicate the target has been reached.
    fn get_long_error(&mut self) -> Result<f64, TrajCtrlError> {
        // Validate sequence
        let path_seq = match self.path_sequence {
            Some(ref s) => s,
            None => return Err(TrajCtrlError::NoSequence),
        };

        // Validate pose
        let pose = match self.input_pose {
            Some(p) => p,
            None => return Err(TrajCtrlError::NoPose),
        };

        // Get the current path segment
        //
        // The unwrap here should be OK since we will init in a safe location
        // from a previous path and should not be beyond the current end of
        // the path.
        let segment = path_seq[self.path_index]
            .get_segment_to_target(self.target_point_index)
            .unwrap();

        // Get the slope and intercept of the line that passes through the
        // rover's position and is perpendicular to the segment.
        let lat_slope_m = -1f64 / segment.slope_m;
        let lat_intercept_m = pose.position_m[1] - lat_slope_m * pose.position_m[0];

        // Find the point of intersection by equating the lines for the segment
        // and the lateral.
        let mut isect_m_lm = [0f64; 2];
        isect_m_lm[0] = (lat_intercept_m - segment.intercept_m) / (segment.slope_m - lat_slope_m);
        isect_m_lm[1] = segment.slope_m * isect_m_lm[0] + segment.intercept_m;

        // Get the vector from the intersection to the target
        let isect_vec = segment.target_m - Vector2::from(isect_m_lm);

        // Get the vector from the start to the target
        let seg_vec = segment.target_m - segment.start_m;

        // The sign of the error is the sign of the dot of these vectors
        let dot = isect_vec.dot(&seg_vec);

        // The error is then the length of the intersection vector, multiplied by the sign of the
        // dot product
        self.report.long_error_m = isect_vec.norm() * dot.signum();

        Ok(self.report.long_error_m)
    }
}
