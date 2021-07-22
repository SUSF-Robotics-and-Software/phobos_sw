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
use comms_if::eqpt::perloc::DepthImage;
use log::{error, info, warn};
use util::params::{load as load_params, LoadError};

use crate::auto::{
    auto_mgr::{
        states::ImgStop, AutoMgrError, AutoMgrOutput, AutoMgrState, StackAction, StepOutput,
    },
    loc::Pose,
    map::{CostMap, CostMapError, CostMapParams, TerrainMap},
    path::Path,
    per::{PerError, PerMgr},
    traj_ctrl::{StatusReport, TrajCtrl, TrajCtrlError},
};

use self::{
    params::TravMgrParams,
    worker::{worker_thread, WorkerSignal},
};

use super::{path_planner::PathPlanner, NavError, NavPose};

// -----------------------------------------------------------------------------------------------
// MODULES
// -----------------------------------------------------------------------------------------------

pub mod escape_boundary;
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

    depth_img_request_sent: bool,
    worker_task_started: bool,

    pub traj_ctrl: TrajCtrl,

    /// The final target of the traverse
    pub target: Option<NavPose>,
}

/// Output from the traverse manager
#[derive(Debug)]
pub struct TravMgrOutput {
    /// Output to pass to the automanager
    pub step_output: StepOutput,

    /// Status report from traj ctrl
    pub traj_ctrl_status: Option<StatusReport>,
}

#[derive(Debug)]
struct Shared {
    pub params: TravMgrParams,
    pub cost_map_params: CostMapParams,

    pub traverse_state: RwLock<TraverseState>,

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
    EscapeBoundaryPointOutsideMap,
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
            per_mgr: RwLock::new(per_mgr),
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
            depth_img_request_sent: false,
            worker_task_started: false,
            target: None,
        })
    }

    /// Start a traverse towards the given final target
    ///
    /// Note the user must call step before any actions will be taken
    pub fn start(
        &mut self,
        target: NavPose,
        ground_path: Option<Path>,
    ) -> Result<(), TravMgrError> {
        // If we're not in OFF raise error
        if !matches!(*self.shared.traverse_state.read()?, TraverseState::Off) {
            Err(TravMgrError::AlreadyTraversing)
        } else {
            self.target = Some(target);
            *self.shared.traverse_state.write()? = TraverseState::FirstStop;
            *self.shared.ground_path.write()? = ground_path;

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

    /// Step the traverse manager
    pub fn step(
        &mut self,
        depth_img: Option<&DepthImage>,
        pose: &Pose,
    ) -> Result<TravMgrOutput, TravMgrError> {
        // Get a read only copy of the traverse state so we can modify it during the state machine
        let trav_state = { *self.shared.traverse_state.read()? };

        // Main state machine
        match trav_state {
            TraverseState::Off => Ok(TravMgrOutput {
                step_output: StepOutput::none(),
                traj_ctrl_status: None,
            }),
            TraverseState::Traverse => {
                // Check for signal from the worker
                if let Some(signal) = match self.worker_reciever.try_recv() {
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
                                traj_ctrl_status: None,
                            });
                        }
                    },
                } {
                    match signal {
                        WorkerSignal::Complete => info!("Secondary path calculated"),
                        WorkerSignal::Error(e) => {
                            error!("Failed to calculate secondary path: {}", e);

                            // TODO: more graceful recovery such as trying the image again?
                            return Ok(TravMgrOutput {
                                step_output: StepOutput {
                                    action: StackAction::Abort,
                                    data: AutoMgrOutput::None,
                                },
                                traj_ctrl_status: None,
                            });
                        }
                        s => warn!("Unexpected signal from worker: {:?}", s),
                    }
                }

                // Do traj ctrl processing on the primary path and the current pose
                let (loco_ctrl_cmd, traj_ctrl_status) = self.traj_ctrl.proc(&pose)?;

                // Check for TrajCtrl finishing
                if traj_ctrl_status.sequence_finished {
                    if traj_ctrl_status.sequence_aborted {
                        error!("TrajCtrl aborted the path sequence");
                    }
                    info!("TrajCtrl reached end of path");

                    // Check if we're at the end of the traverse

                    // Set into stop mode
                    *self.shared.traverse_state.write()? = TraverseState::Stop;
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
                    traj_ctrl_status: Some(traj_ctrl_status),
                })
            }
            TraverseState::Stop | TraverseState::FirstStop => {
                let mut end_of_stop = false;

                // If this is a normal stop and we have a secondary path plotted promote it to the
                // primary path
                {
                    let mut secondary = self.shared.secondary_path.write()?;
                    if matches!(trav_state, TraverseState::Stop) && secondary.is_some() {
                        let mut primary = self.shared.primary_path.write()?;
                        *primary = secondary.take();
                        info!("Secondary path promoted to primary");
                    }
                }

                // Push an image stop onto the stack
                if !self.depth_img_request_sent {
                    self.depth_img_request_sent = true;
                    return Ok(TravMgrOutput {
                        step_output: StepOutput {
                            action: StackAction::PushAbove(AutoMgrState::ImgStop(
                                ImgStop::default(),
                            )),
                            data: AutoMgrOutput::None,
                        },
                        traj_ctrl_status: None,
                    });
                }

                // Once the depth image has been requested wait for it in the depth image pointer
                if depth_img.is_none() {
                    return Ok(TravMgrOutput {
                        step_output: StepOutput::none(),
                        traj_ctrl_status: None,
                    });
                }

                // Once we have the image send it and the current pose to the worker
                if !self.worker_task_started {
                    info!("Starting background processing of depth image");
                    self.worker_sender
                        .send(WorkerSignal::NewDepthImg(depth_img.unwrap().clone(), *pose))?;
                    self.worker_task_started = true;
                }

                // If we're in stop mode once the image is sent on we can return to traverse,
                // otherwise we have to wait for the worker to send the "complet" signal
                let mut output = Ok(StepOutput::none());
                match trav_state {
                    TraverseState::Stop => {
                        *self.shared.traverse_state.write()? = TraverseState::Traverse;
                        end_of_stop = true;
                    }
                    TraverseState::FirstStop => {
                        let signal = match self.worker_reciever.try_recv() {
                            Ok(s) => Some(s),
                            Err(e) => {
                                if matches!(e, std::sync::mpsc::TryRecvError::Disconnected) {
                                    output = Err(TravMgrError::RecvError);
                                }
                                None
                            }
                        };

                        if let Some(signal) = signal {
                            match signal {
                                WorkerSignal::Complete => {
                                    *self.shared.traverse_state.write()? = TraverseState::Traverse;
                                    end_of_stop = true;
                                }
                                WorkerSignal::Error(e) => {
                                    error!("Error processing last depth image: {:?}", e);

                                    // TODO: more graceful recovery such as trying the image again?
                                    output = Ok(StepOutput {
                                        action: StackAction::Abort,
                                        data: AutoMgrOutput::None,
                                    });
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
                    self.worker_task_started = false;
                }

                Ok(TravMgrOutput {
                    step_output: output?,
                    traj_ctrl_status: None,
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
