//! Trajectory control module state

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// Internal
use super::*;
use crate::loc::Pose;
use crate::loco_ctrl::{MnvrCommand, MnvrType};
use util::{
    module::State,
    params,
    maths::norm,
    session::Session
};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

pub struct TrajCtrl {
    params: Params,

    /// Executing mode
    mode: Mode,

    input_data: InputData,
    output_data: OutputData,
    report: StatusReport,

    /// The sequence of paths to execute.
    path_sequence: Vec<Path>,

    /// Index of the current executing path
    path_index: usize,

    /// Index of the current target point within the path
    target_point_index: usize,

    /// Controller objects used to calculate manouvre commands
    controllers: TrajControllers
}

/// Input data to the module
#[derive(Copy, Clone)]
pub struct InputData {
    pose: Pose
}

#[derive(Default, Copy, Clone)]
pub struct OutputData {
    mnvr_cmd: Option<MnvrCommand>
}

/// The status report containing various error flags and monitoring quantities.
#[derive(Default, Copy, Clone)]
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
    pub head_error_limit_exceeded: bool
}

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Potential errors that could occur during initialisation of the module.
pub enum InitError {
    ParamLoadError(params::LoadError)
}

/// Potential errors that can occur during processing of the module.
pub enum ProcError {
    /// A sequence is already loaded. This error occurs when attempting to 
    /// start a new sequence before the current one has finished.
    SequenceAlreadyLoaded,

    /// Attempted to load an empty sequence. This error occurs if the sequence 
    /// to be loaded doesn't have any paths in it.
    AttemptEmptySeqLoad,

    /// Attempted to load a sequence containing invalid paths. The contained
    /// vector provides the indices of the paths which were invalid.
    SequenceContainsInvalidPaths(Vec<usize>)
}

/// The possible modes of execution of TrajCtrl. Each mode is handled by a 
/// `mode_xyz` function. 
pub enum Mode {
    NotExecuting,
    FollowingPath,
    HeadingAdjust,
    SequenceFinished
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl State for TrajCtrl {
    type InitData = &'static str;
    type InitError = InitError;
    
    type InputData = InputData;
    type OutputData = OutputData;
    type StatusReport = StatusReport;
    type ProcError = ProcError;
    
    /// Intiailise the TrajCtrl module.
    ///
    /// Expected init data is a path to the parameter file.
    fn init(
        &mut self, 
        init_data: Self::InitData, 
        _session: &Session
    ) -> Result<(), Self::InitError> {
        // Load the parameters
        self.params = match params::load(init_data) {
            Ok(p) => p,
            Err(e) => return Err(InitError::ParamLoadError(e))
        };

        // Initialise the controllers
        self.controllers = TrajControllers::new(&self.params);

        Ok(())
    }

    /// Process trajectory control.
    ///
    /// Processing involves:
    ///  1. Get the current executing path segment (incrementing the target if
    ///     required.)
    ///  1. Calculating LocoCtrl command based on current position and path
    fn proc(
        &mut self, 
        input_data: &Self::InputData
    ) -> Result<(Self::OutputData, Self::StatusReport), Self::ProcError> {

        // Setup cycle data
        self.input_data = *input_data;
        self.output_data = OutputData::default();
        self.report = StatusReport::default();

        // Mode execution. Each of the mode functions returns either the mode
        // to switch to or an error
        match self.mode {
            Mode::NotExecuting => self.mode_not_exec(),
            Mode::FollowingPath => self.mode_follow_path(),
            Mode::HeadingAdjust => self.mode_head_adjust(),
            Mode::SequenceFinished => self.mode_seq_finished()
        }?;

        Ok((self.output_data, self.report))
    }
}

impl<'a> TrajCtrl {

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
    pub fn begin_path_sequence(
        &mut self, seq: Vec<Path>
    ) -> Result<(), ProcError> {

        // Check to see if there's already a sequence loaded
        if self.path_sequence.len() > 0 {
            return Err(ProcError::SequenceAlreadyLoaded)
        }

        // Verify that the new sequence contains at least one path
        if seq.len() == 0 {
            return Err(ProcError::AttemptEmptySeqLoad)
        }
        
        // Check that all paths in the sequence are valid
        let mut invalid_path_indexes: Vec<usize> = vec![];
        for (i, path) in seq.iter().enumerate() {
            if path.get_num_points() < 2 {
                invalid_path_indexes.push(i);
            }
        }

        // If there were invalid paths 
        if invalid_path_indexes.len() > 0 {
            return Err(
                ProcError::SequenceContainsInvalidPaths(invalid_path_indexes))
        }

        // Setup counters and sequence. The target must be 1 not 0 as a segment
        // is defined backwards, i.e. between the target and previous points.
        self.path_sequence = seq;
        self.path_index = 0;
        self.target_point_index = 1;

        // Switch into Heading Adjust mode.
        self.mode = Mode::HeadingAdjust;

        Ok(())
    }

    /// Abort the currently executing path sequence.
    ///
    /// This will transfer the mode into sequence finished so that on the next
    /// call to `proc` a stop command is issued and the path sequence cleared.
    pub fn abort_path_sequence(&mut self) -> Result<(), ProcError> {

        // If there's already a loaded path exit, otherwise don't do anything
        if self.path_sequence.len() > 0 {
            self.mode = Mode::SequenceFinished;
        }

        Ok(())
    }

    /// Mode not executing.
    ///
    /// No actions are taken in this mode. To move from NotExec to 
    /// HeadingAdjust the user must call `begin_path_sequence`.
    fn mode_not_exec(&mut self)-> Result<(), ProcError> {
        Ok(())
    }

    /// Mode following path
    ///
    /// In this mode TrajCtrl will command LocoCtrl to execute the current path.
    fn mode_follow_path(&mut self) -> Result<(), ProcError> {
        
        // ---- TARGET MANAGEMENT ----

        // Find longitudonal error to next target
        let long_err_m = self.get_long_error();

        // If the error is positive we have passed the target, so increment to 
        // the next target
        if long_err_m > 0f64 {
            self.target_point_index += 1;
        }

        // If the target point index is at or greater than the length of the
        // path then switch to the next path sequence.
        if self.target_point_index 
            >= 
            self.path_sequence[self.path_index].get_num_points()
        {
            self.path_index += 1;
        }

        // If the path index is now greater than the length of the sequence
        // switch to the squence finished mode and leave execution here.
        if self.path_index >= self.path_sequence.len() {
            self.mode = Mode::SequenceFinished;
            return Ok(())
        }

        // ---- COMMAND GENERATION ----

        // Get the current path segment
        //
        // Can safely unwrap here as target management already performed
        let segment = self.path_sequence[self.path_index]
            .get_segment_to_target(self.target_point_index)
            .unwrap();

        // Get the command
        let mnvr_cmd = self.controllers.get_ackerman_cmd(
            &segment, &self.input_data.pose, &mut self.report, &self.params);

        // Check for error exceedance
        if self.report.lat_error_limit_exceeded 
            || 
            self.report.head_error_limit_exceeded 
        {
            // Switch to sequence exceeded mode immediately so that we are 
            // stopped as close to the path as possible.
            self.mode = Mode::SequenceFinished;
            self.mode_seq_finished()?;
        }
        else {
            // Set the command in the output data
            self.output_data.mnvr_cmd = Some(mnvr_cmd);
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
    fn mode_head_adjust(&mut self) -> Result<(), ProcError> {

        // Start by calculating the heading error
        let segment = self.path_sequence[self.path_index]
            .get_segment_to_target(self.target_point_index)
            .unwrap();
        let head_err_rad = self.input_data.pose.get_heading() 
            - segment.slope_m.atan();
        
        // If the error is less than the threshold the adjustment is complete
        if head_err_rad.abs() < self.params.head_adjust_threshold_rad {
            // Issue a stop command and switch to follow path mode
            self.output_data.mnvr_cmd = Some(MnvrCommand {
                mnvr_type: MnvrType::Stop,
                curvature_m: None,
                speed_ms: None,
                turn_rate_rads: None
            });

            self.mode = Mode::FollowingPath;
        }
        else {
            // Set the turn speed. The sense of the heading error is the same
            // as that of the turn rate, therefore if there is a positive error
            // we need a negative turn rate to decrease that error.
            self.output_data.mnvr_cmd = Some(MnvrCommand {
                mnvr_type: MnvrType::PointTurn,
                curvature_m: None,
                speed_ms: None,
                turn_rate_rads: Some(-1f64 * head_err_rad.signum() 
                    * self.params.head_adjust_rate_rads)
            });
        }

        Ok(())
    }

    /// Mode sequence finished.
    ///
    /// This mode is run when all the paths in the current sequence have been
    /// completed and the rover should come to a full stop. It will set the
    /// output command to `Stop` and clear the path sequence.
    fn mode_seq_finished(&mut self) -> Result<(), ProcError> {
        
        // Set the stop command
        self.output_data.mnvr_cmd = Some(MnvrCommand {
            mnvr_type: MnvrType::Stop,
            curvature_m: None,
            speed_ms: None,
            turn_rate_rads: None
        });

        // Clear the path sequence
        self.path_sequence = vec![];
        self.path_index = 0;
        self.target_point_index = 0;

        // Switch to NotExecuting mode
        self.mode = Mode::NotExecuting;

        // Return
        Ok(())
    }

    /// Get the longitudonal error to the current path segment.
    ///
    /// Positive errors indiciate that the rover hasn't reached the target yet.
    /// Negative errors indicate the target has been reached.
    fn get_long_error(&mut self) -> f64 {

        // Get the current path segment
        //
        // The unwrap here should be OK since we will init in a safe location
        // from a previous path and should not be beyond the current end of
        // the path.
        let segment = self.path_sequence[self.path_index]
            .get_segment_to_target(self.target_point_index) 
            .unwrap();

        // Get the slope and intercept of the line that passes through the 
        // rover's position and is perpendicular to the segment.
        let lat_slope_m = - 1f64 / segment.slope_m;
        let lat_intercept_m = self.input_data.pose.position_m_lm[1] 
            - lat_slope_m * self.input_data.pose.position_m_lm[0];

        // Find the point of intersection by equating the lines for the segment
        // and the lateral.
        let mut isect_m_lm = [0f64; 2];
        isect_m_lm[0] = (lat_intercept_m - segment.intercept_m)
            / (lat_slope_m - segment.slope_m);
        isect_m_lm[1] = segment.slope_m * isect_m_lm[0] + segment.intercept_m;

        // The longitudonal error is then the distance between the intersection
        // and the target. To deal with the sign issue we use the dot product
        // to find the angle between the vectors intersect->target and 
        // start->target. If the angle is 0 the distance is positive, otherwise
        // it's negative.
        let mut long_err_m = norm(
            &isect_m_lm, &segment.target_m_lm)
            .unwrap();
        
        // Get vectors
        let isect_vec = [
            segment.target_m_lm[0] - isect_m_lm[0],
            segment.target_m_lm[1] - isect_m_lm[1]
        ];
        let seg_vec = [
            segment.target_m_lm[0] - segment.start_m_lm[0],
            segment.target_m_lm[1] - segment.start_m_lm[0]
        ];

        // Get dot product
        let vec_dot = isect_vec[0] * seg_vec[0] + isect_vec[1] * seg_vec[1];

        // Get modulus of each 
        let isect_mod = norm(&[0f64; 2], &isect_vec).unwrap();
        let seg_mod = norm(&[0f64; 2], &seg_vec).unwrap(); 

        // Calculate angle
        let vec_angle_rad = (vec_dot/(isect_mod * seg_mod)).acos();

        // If the angle is not 0 invert the long_err
        if vec_angle_rad > std::f64::consts::PI / 2f64 {
            long_err_m *= -1f64;
        }
        
        // Set the error in the status report
        self.report.long_error_m = long_err_m;

        long_err_m
    }
}