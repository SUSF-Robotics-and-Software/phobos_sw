//! # Traverse Manager
//!
//! Controls the traverse of the rover across the global cost map.

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use std::{
    sync::{
        mpsc::{channel, Receiver, RecvError, SendError, Sender},
        Arc, PoisonError, RwLock,
    },
    thread::{self, JoinHandle},
};

use cell_map::CellMapParams;
use comms_if::{eqpt::perloc::DepthImage, tc::loco_ctrl::MnvrCmd};
use log::{error, info, warn};
use util::params::{load as load_params, LoadError};

use crate::auto::{
    auto_mgr::{
        states::ImgStop,
        AutoMgrError, AutoMgrOutput, AutoMgrState, StackAction, StepOutput,
    },
    loc::Pose,
    map::{CostMap, CostMapError, CostMapParams, TerrainMap},
    path::Path,
    per::{PerError, PerMgr},
    traj_ctrl::{StatusReport, TrajCtrl, TrajCtrlError},
};

use self::{
    local_target::LocalTarget,
    params::TravMgrParams,
    worker::{worker_thread, WorkerSignal},
};

use super::{path_planner::PathPlanner, NavError, NavPose};

// -----------------------------------------------------------------------------------------------
// MODULES
// -----------------------------------------------------------------------------------------------

pub mod escape_boundary;
mod local_target;
pub mod params;
mod worker;

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

/// The traverse manager, which controls how the rover moves autonomously in check and goto modes.
#[derive(Debug)]
pub struct TravMgr {
    shared: Arc<Shared>,

    worker_jh: JoinHandle<Result<(), TravMgrError>>,

    worker_sender: Sender<WorkerSignal>,
    worker_reciever: Receiver<WorkerSignal>,

    recalc_running: bool,
    depth_img_request_sent: bool,
    img_proc_task_started: bool,

    pub traj_ctrl: TrajCtrl,
}

/// Output from the traverse manager
#[derive(Debug)]
pub struct TravMgrOutput {
    /// Output to pass to the automanager
    pub step_output: StepOutput,

    /// Status report from traj ctrl
    pub traj_ctrl_status: Option<StatusReport>,

    /// Optionally a new global terrain map
    pub new_global_terr_map: Option<TerrainMap>,

    /// Optionally a new global cost map
    pub new_global_cost_map: Option<CostMap>,

    /// The primary path to be driven by traj_ctrl
    pub primary_path: Option<Path>,

    /// The secondary path to be driven once primary_path is complete
    pub secondary_path: Option<Path>,
}

#[derive(Debug)]
struct Shared {
    pub params: TravMgrParams,
    pub cost_map_params: CostMapParams,

    pub traverse_state: RwLock<TraverseState>,
    pub secondary_is_final: RwLock<bool>,

    pub global_target: RwLock<Option<NavPose>>,
    pub local_target: RwLock<Option<LocalTarget>>,

    pub per_mgr: RwLock<PerMgr>,
    pub global_terr_map: RwLock<TerrainMap>,
    pub global_cost_map: RwLock<CostMap>,
    pub path_planner: RwLock<PathPlanner>,

    pub ground_path: RwLock<Option<Path>>,
    pub primary_path: RwLock<Option<Path>>,
    pub secondary_path: RwLock<Option<Path>>,
}

// -----------------------------------------------------------------------------------------------
// ENUMS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub enum TraverseState {
    /// Manager is off
    Off,
    /// Perform a kickstart to get terrain and cost data but don't plan a path
    KickStart,
    /// First stop, plan both primary and secondary path at the same time before traversing
    FirstStop,
    /// Traversing the primary path and planning the secondary path
    Traverse,
    /// Navigation update stop, getting new depth image
    Stop,
}

#[derive(Debug, thiserror::Error)]
pub enum TravMgrError {
    #[error("Couldn't load parameters: {0}")]
    ParamLoadError(LoadError),

    #[error("Sync primitive is poisoned")]
    PoisonError,

    #[error("Perception error: {0}")]
    PerError(PerError),

    #[error("CostMap error: {0}")]
    CostMapError(CostMapError),

    #[error("Nav error: {0}")]
    NavError(NavError),

    #[error("TrajCtrl error: {0}")]
    TrajCtrlError(TrajCtrlError),

    #[error("Worker thread expected a primary path to plan from but there was none")]
    NoPrimaryPath,

    #[error("Path planner failed to produce the correct number of paths")]
    PathPlannerFailed,

    #[error("Failed to send signal {0:?} between threads")]
    SendError(WorkerSignal),

    #[error("Failed to receive signal between threads")]
    RecvError,

    #[error("Cannot start another traverse as a previous traverse hasn't finished")]
    AlreadyTraversing,

    #[error("Couldn't find escape boundary in local cost map")]
    NoEscapeBoundary,

    #[error("Couldn't find any populated cost map cells along local X axis - no escape boundary calculated")]
    EscBoundaryInvalidCentreline,

    #[error("A point on the escape boundary was outside the local cost map")]
    PointOutsideMap,

    #[error("Rover is outside of the local cost map")]
    RoverOutsideMap,

    #[error("Couldn't get a valid target to plot path towards")]
    NoValidTarget,
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl TravMgr {
    /// Create a new traverse manager instance.
    pub fn new() -> Result<Self, TravMgrError> {
        // Load parameters
        let params: TravMgrParams = load_params("trav_mgr.toml")?;
        let traj_ctrl_params = load_params("traj_ctrl.toml")?;
        let per_mgr_params = load_params("per_mgr.toml")?;
        let path_planner_params = load_params("path_planner.toml")?;
        let cost_map_params: CostMapParams = load_params("cost_map.toml")?;

        // Create submodules
        let traj_ctrl = TrajCtrl::new(traj_ctrl_params);

        let per_mgr = PerMgr::new(per_mgr_params);
        let path_planner = PathPlanner::new(path_planner_params);
        let global_terr_map = TerrainMap::new(CellMapParams {
            cell_size: params.map_cell_size,
            ..Default::default()
        });
        let global_cost_map = CostMap::new(
            CellMapParams {
                cell_size: params.map_cell_size,
                ..Default::default()
            },
            cost_map_params.clone(),
        );

        // Create shared data
        let shared = Arc::new(Shared {
            params,
            cost_map_params,
            traverse_state: RwLock::new(TraverseState::Off),
            secondary_is_final: RwLock::new(false),
            per_mgr: RwLock::new(per_mgr),
            global_target: RwLock::new(None),
            local_target: RwLock::new(None),
            global_terr_map: RwLock::new(global_terr_map),
            global_cost_map: RwLock::new(global_cost_map),
            path_planner: RwLock::new(path_planner),
            primary_path: RwLock::new(None),
            secondary_path: RwLock::new(None),
            ground_path: RwLock::new(None),
        });
        let shared_worker = shared.clone();

        // Create channels
        let (worker_sender, rx) = channel();
        let (tx, worker_reciever) = channel();

        // Start worker thread
        let worker_jh = thread::Builder::new()
            .name("trav_mgr::worker".into())
            .spawn(move || worker_thread(shared_worker, tx, rx))
            .unwrap();

        // Create self
        Ok(Self {
            shared,
            worker_jh,
            worker_sender,
            worker_reciever,
            traj_ctrl,
            recalc_running: false,
            depth_img_request_sent: false,
            img_proc_task_started: false,
        })
    }

    /// Perform a kickstart to populate the global terrain and cost maps
    ///
    /// Note the user must call step before any actions will be taken
    pub fn kickstart(&mut self) -> Result<(), TravMgrError> {
        // If we're not in OFF raise error
        if !matches!(*self.shared.traverse_state.read()?, TraverseState::Off) {
            Err(TravMgrError::AlreadyTraversing)
        } else {
            *self.shared.traverse_state.write()? = TraverseState::KickStart;

            Ok(())
        }
    }

    /// Start a traverse towards the given final target
    ///
    /// Note the user must call step before any actions will be taken
    pub fn start_goto(&mut self, target: NavPose) -> Result<(), TravMgrError> {
        // If we're not in OFF raise error
        if !matches!(*self.shared.traverse_state.read()?, TraverseState::Off) {
            Err(TravMgrError::AlreadyTraversing)
        } else {
            *self.shared.traverse_state.write()? = TraverseState::FirstStop;
            *self.shared.global_target.write()? = Some(target);

            // Recompute the global cost map with no gpp
            self.worker_sender
                .send(WorkerSignal::RecalcGlobalCost(None))?;
            self.recalc_running = true;

            Ok(())
        }
    }

    /// Start a traverse following the given ground path
    ///
    /// Note the user must call step before any actions will be taken
    pub fn start_check(&mut self, ground_path: Path) -> Result<(), TravMgrError> {
        // If we're not in OFF raise error
        if !matches!(*self.shared.traverse_state.read()?, TraverseState::Off) {
            Err(TravMgrError::AlreadyTraversing)
        } else {
            *self.shared.traverse_state.write()? = TraverseState::FirstStop;
            *self.shared.ground_path.write()? = Some(ground_path.clone());
            *self.shared.local_target.write()? = Some(LocalTarget::new(
                self.shared.params.local_target_exclusion_distance_m,
                self.shared.params.local_target_max_distance_m,
            ));

            // Recompute the global cost map with a gpp
            self.worker_sender
                .send(WorkerSignal::RecalcGlobalCost(Some(ground_path)))?;
            self.recalc_running = true;

            Ok(())
        }
    }

    /// Returns whether or not the manager is off
    pub fn is_off(&self) -> bool {
        match self.shared.traverse_state.read() {
            Ok(lock) => matches!(*lock, TraverseState::Off),
            _ => false,
        }
    }

    /// Stops the current traverse
    pub fn stop(&mut self) -> Result<TravMgrOutput, TravMgrError> {
        // Set the traverse state to off
        *self.shared.traverse_state.write()? = TraverseState::Off;

        // Clear the primary and secondary paths
        *self.shared.primary_path.write()? = None;
        *self.shared.secondary_path.write()? = None;

        // Stop traj ctrl
        self.traj_ctrl.abort_path_sequence()?;

        // Reset camera state
        self.depth_img_request_sent = false;
        self.img_proc_task_started = false;

        Ok(TravMgrOutput {
            step_output: StepOutput {
                action: StackAction::None,
                data: AutoMgrOutput::LocoCtrlMnvr(MnvrCmd::Stop),
            },
            ..Default::default()
        })
    }

    /// Replan after failing to calculate a valid secondary path
    fn replan(&mut self) -> Result<TravMgrOutput, TravMgrError> {
        info!("Beginning replan");

        // Stop the traverse
        let out = self.stop()?;

        let gpp = self.shared.ground_path.write()?.take();
        let target = self.shared.global_target.write()?.take();

        // If there's a ground planned path do check mode again
        if let Some(gpp) = gpp {
            self.start_check(gpp).map(|_| out)
        }
        // Otherwise this is a goto
        else if let Some(target) = target {
            self.start_goto(target).map(|_| out)
        } else {
            unreachable!("No ground planned path or target");
        }
    }

    fn end_traverse(&mut self) -> Result<TravMgrOutput, TravMgrError> {
        // Set self to off
        *self.shared.traverse_state.write()? = TraverseState::Off;

        // Clear the ground path and target if they exist, but don't clear the maps
        *self.shared.ground_path.write()? = None;
        *self.shared.global_target.write()? = None;
        *self.shared.local_target.write()? = None;
        *self.shared.secondary_is_final.write()? = false;
        *self.shared.primary_path.write()? = None;
        *self.shared.secondary_path.write()? = None;

        // Set flags in self ready for next traverse
        self.depth_img_request_sent = false;
        self.img_proc_task_started = false;
        self.recalc_running = false;

        // It is the mode's responsibility to add a stop. This can be detected by checking if
        //is off.
        Ok(TravMgrOutput {
            step_output: StepOutput::none(),
            ..Default::default()
        })
    }

    /// Step the traverse manager
    pub fn step(
        &mut self,
        depth_img: Option<&DepthImage>,
        pose: &Pose,
    ) -> Result<TravMgrOutput, TravMgrError> {
        // Get a read only copy of the traverse state so we can modify it during the state machine
        let trav_state = { *self.shared.traverse_state.read()? };

        // Check for signal from the worker
        let mut signal = match self.worker_reciever.try_recv() {
            Ok(s) => Some(s),
            Err(e) => match e {
                std::sync::mpsc::TryRecvError::Empty => None,
                std::sync::mpsc::TryRecvError::Disconnected => {
                    error!("Worker has stopped, aborting");
                    return Ok(TravMgrOutput {
                        step_output: StepOutput {
                            action: StackAction::Abort,
                            data: AutoMgrOutput::None,
                        },
                        ..Default::default()
                    });
                }
            },
        };

        // Get a copy of the new maps if they've been computed
        let (new_global_terr_map, new_global_cost_map) = if let Some(ref sig) = signal {
            match sig {
                WorkerSignal::GlobalMapsUpdated => {
                    // Mark the signal as None so the state machine won't process this one
                    signal = None;
                    (
                        Some(self.shared.global_terr_map.read()?.clone()),
                        Some(self.shared.global_cost_map.read()?.clone()),
                    )
                }
                _ => (None, None),
            }
        } else {
            (None, None)
        };

        // Main state machine
        match trav_state {
            TraverseState::Off => Ok(TravMgrOutput {
                step_output: StepOutput::none(),
                ..Default::default()
            }),
            TraverseState::Traverse => {
                // Check for worker completion
                if let Some(ref signal) = signal {
                    match signal {
                        WorkerSignal::Complete => info!("Secondary path calculated"),
                        WorkerSignal::Error(e) => {
                            error!("Failed to calculate secondary path: {}", e);

                            return self.replan();
                        }
                        s => warn!("Unexpected signal from worker: {:?}", s),
                    }
                }

                // Do traj ctrl processing on the primary path and the current pose
                let (loco_ctrl_cmd, traj_ctrl_status) = self.traj_ctrl.proc(&pose)?;

                // Check for TrajCtrl finishing
                if traj_ctrl_status.sequence_finished {
                    if traj_ctrl_status.sequence_aborted {
                        error!("TrajCtrl aborted the path, aborting traverse");
                        return Ok(TravMgrOutput {
                            step_output: StepOutput {
                                action: StackAction::Abort,
                                data: AutoMgrOutput::None,
                            },
                            ..Default::default()
                        });
                    }
                    info!("TrajCtrl reached end of path");

                    // Check if we're at the end of the traverse, which is if secondary_is_final is
                    // true but there is no secondary path. In this case we finish the traverse
                    // now.
                    let secondary_is_none = { self.shared.secondary_path.read()?.is_none() };
                    if *self.shared.secondary_is_final.read()? && secondary_is_none {
                        info!("Traverse complete");

                        return self.end_traverse();
                    }
                    // Set into stop mode if we shouldn't end the traverse
                    else {
                        *self.shared.traverse_state.write()? = TraverseState::Stop;
                    }
                }

                // Output the loco_ctrl command
                Ok(TravMgrOutput {
                    step_output: match loco_ctrl_cmd {
                        Some(mnvr) => StepOutput {
                            action: StackAction::None,
                            data: AutoMgrOutput::LocoCtrlMnvr(mnvr),
                        },
                        None => StepOutput::none(),
                    },
                    new_global_terr_map,
                    new_global_cost_map,
                    primary_path: self.shared.primary_path.read()?.clone(),
                    secondary_path: self.shared.secondary_path.read()?.clone(),
                    ..Default::default()
                })
            }
            TraverseState::Stop | TraverseState::FirstStop | TraverseState::KickStart => {
                let mut end_of_stop = false;

                // If this is a normal stop and we have a secondary path plotted promote it to the
                // primary path
                {
                    let mut secondary = self.shared.secondary_path.write()?;
                    if matches!(trav_state, TraverseState::Stop) && secondary.is_some() {
                        let mut primary = self.shared.primary_path.write()?;
                        *primary = secondary.take();

                        info!("Secondary path promoted to primary");

                        // If at the end of the traverse
                        if *self.shared.secondary_is_final.read()? {
                            info!("New primary is last path of traverse");
                            *self.shared.traverse_state.write()? = TraverseState::Traverse;
                            self.traj_ctrl
                                .begin_path_sequence(vec![primary.as_ref().unwrap().clone()])?;

                            return Ok(TravMgrOutput {
                                step_output: StepOutput::none(),
                                new_global_terr_map,
                                new_global_cost_map,
                                primary_path: secondary.clone(),
                                secondary_path: primary.clone(),
                                ..Default::default()
                            });
                        }
                    }
                }

                // Check for optional recalc complete
                if self.recalc_running {
                    if let Some(ref signal) = signal {
                        match signal {
                            WorkerSignal::Complete => {
                                info!("Global cost map recalculated");
                                self.recalc_running = false;
                            }
                            WorkerSignal::Error(e) => {
                                error!("Failed to recalculate global cost map: {}", e);

                                // TODO: more graceful recovery such as trying the image again?
                                return Ok(TravMgrOutput {
                                    step_output: StepOutput {
                                        action: StackAction::Abort,
                                        data: AutoMgrOutput::None,
                                    },
                                    new_global_terr_map,
                                    new_global_cost_map,
                                    primary_path: self.shared.primary_path.read()?.clone(),
                                    secondary_path: self.shared.secondary_path.read()?.clone(),
                                    ..Default::default()
                                });
                            }
                            s => warn!("Unexpected signal from worker: {:?}", s),
                        }
                    } else {
                        return Ok(TravMgrOutput {
                            step_output: StepOutput::none(),
                            new_global_terr_map,
                            new_global_cost_map,
                            primary_path: self.shared.primary_path.read()?.clone(),
                            secondary_path: self.shared.secondary_path.read()?.clone(),
                            ..Default::default()
                        });
                    }
                }

                // Push an image stop onto the stack
                if !self.depth_img_request_sent {
                    println!();
                    info!("---- NAV STOP ----");
                    self.depth_img_request_sent = true;
                    return Ok(TravMgrOutput {
                        step_output: StepOutput {
                            action: StackAction::PushAbove(AutoMgrState::ImgStop(
                                ImgStop::default(),
                            )),
                            data: AutoMgrOutput::None,
                        },
                        new_global_terr_map,
                        new_global_cost_map,
                        primary_path: self.shared.primary_path.read()?.clone(),
                        secondary_path: self.shared.secondary_path.read()?.clone(),
                        ..Default::default()
                    });
                }

                // Once the depth image has been requested wait for it in the depth image pointer
                if depth_img.is_none() {
                    return Ok(TravMgrOutput {
                        step_output: StepOutput::none(),
                        new_global_terr_map,
                        new_global_cost_map,
                        primary_path: self.shared.primary_path.read()?.clone(),
                        secondary_path: self.shared.secondary_path.read()?.clone(),
                        ..Default::default()
                    });
                }

                // Once we have the image send it and the current pose to the worker, with a flag
                // if it's for a kickstart or not
                if !self.img_proc_task_started {
                    info!("Starting background processing of depth image");
                    self.worker_sender.send(WorkerSignal::NewDepthImg(
                        Box::new(depth_img.unwrap().clone()),
                        *pose,
                        matches!(trav_state, TraverseState::KickStart),
                    ))?;
                    self.img_proc_task_started = true;
                }

                // If we're in stop mode once the image is sent on we can return to traverse,
                // otherwise we have to wait for the worker to send the "complete" signal
                match trav_state {
                    TraverseState::Stop => {
                        *self.shared.traverse_state.write()? = TraverseState::Traverse;
                        end_of_stop = true;
                    }
                    TraverseState::FirstStop | TraverseState::KickStart => {
                        if let Some(ref signal) = signal {
                            match signal {
                                WorkerSignal::Complete => {
                                    // Switch into Traverse if in the FirstStop, but if it's a
                                    // KickStart switch to Off
                                    *self.shared.traverse_state.write()? = match trav_state {
                                        TraverseState::FirstStop => {
                                            info!("Primary and secondary paths calculated");
                                            TraverseState::Traverse
                                        }
                                        TraverseState::KickStart => TraverseState::Off,
                                        _ => unreachable!("Unexpected traverse state"),
                                    };
                                    end_of_stop = true;
                                }
                                WorkerSignal::Error(e) => {
                                    error!("Error processing last depth image: {:?}", e);

                                    // TODO: more graceful recovery such as trying the image again?
                                    return self.stop();
                                }
                                s => {
                                    warn!("Unexpected signal from worker: {:?}", s);
                                }
                            }
                        }
                    }
                    _ => unreachable!("Unexpected traverse state"),
                };

                // Reset internal flags if the stop is over
                if end_of_stop {
                    self.depth_img_request_sent = false;
                    self.img_proc_task_started = false;
                }

                // If we have switched into traverse during the state machine start traj_ctrl on
                // the primary path
                if matches!(*self.shared.traverse_state.read()?, TraverseState::Traverse) {
                    if let Some(ref primary) = *self.shared.primary_path.read()? {
                        info!("Traversing primary path");
                        self.traj_ctrl.begin_path_sequence(vec![primary.clone()])?;
                    }
                }

                Ok(TravMgrOutput {
                    step_output: StepOutput::none(),
                    new_global_terr_map,
                    new_global_cost_map,
                    primary_path: self.shared.primary_path.read()?.clone(),
                    secondary_path: self.shared.secondary_path.read()?.clone(),
                    ..Default::default()
                })
            }
        }
    }
}

impl From<LoadError> for TravMgrError {
    fn from(e: LoadError) -> Self {
        Self::ParamLoadError(e)
    }
}

impl From<TrajCtrlError> for TravMgrError {
    fn from(e: TrajCtrlError) -> Self {
        Self::TrajCtrlError(e)
    }
}

impl From<TravMgrError> for AutoMgrError {
    fn from(e: TravMgrError) -> Self {
        match e {
            TravMgrError::ParamLoadError(l) => AutoMgrError::ParamLoadError(l),
            _ => panic!("Unrecoverable error: {}", e),
        }
    }
}

impl<G> From<PoisonError<G>> for TravMgrError {
    fn from(_: PoisonError<G>) -> Self {
        Self::PoisonError
    }
}

impl From<SendError<WorkerSignal>> for TravMgrError {
    fn from(e: SendError<WorkerSignal>) -> Self {
        Self::SendError(e.0)
    }
}

impl From<RecvError> for TravMgrError {
    fn from(_: RecvError) -> Self {
        Self::RecvError
    }
}

impl Default for TravMgrOutput {
    fn default() -> Self {
        Self {
            step_output: StepOutput::none(),
            traj_ctrl_status: None,
            new_global_terr_map: None,
            new_global_cost_map: None,
            primary_path: None,
            secondary_path: None,
        }
    }
}
