//! # AutoMgr module
//!
//! This module implements the [`AutoMgr`] state machine, which is responsible for perfoming the
//! autonomy actions of the rover.

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

mod auto_mnvr;
mod check;
mod follow;
mod params;
mod pause;
mod stop;
pub mod tm;
mod wait_new_pose;

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::{fmt::Display, unimplemented};

pub use self::{params::AutoMgrParams, tm::AutoTm};

use super::{
    loc::{LocMgr, LocSource},
    map::TerrainMap,
    path::PathError,
    traj_ctrl::TrajCtrlError,
};

// ------------------------------------------------------------------------------------------------
// EXPORTS
// ------------------------------------------------------------------------------------------------

pub mod states {
    // pub use super::off::Off;
    pub use super::auto_mnvr::AutoMnvr;
    pub use super::check::Check;
    pub use super::follow::Follow;
    pub use super::pause::Pause;
    pub use super::stop::Stop;
    pub use super::wait_new_pose::WaitNewPose;
}

use cell_map::CellMapParams;
use comms_if::tc::{auto::AutoCmd, loco_ctrl::MnvrCmd};
use log::{error, info, warn};
use states::*;
use util::session::Session;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Autonomy Manager
///
/// This struct is responsible for managing the state of the Autonomy system, including the current
/// operating state of the system.
pub struct AutoMgr {
    /// Parameters for the AutoMgr and all it's states.
    pub params: AutoMgrParams,

    /// Persistant data of the AutoMgr.
    ///
    /// This is data which is valid over all states, such as the global terrain map. This allows the
    /// global map to not be lost when a new state is entered.
    pub persistant: AutoMgrPersistantData,

    /// The data returned by the most recently stepped state.
    ///
    /// This can be used to obtain information from the last state, for example images from an
    /// ImgStop.
    pub last_stack_data: StackData,

    /// The stack of states in the system.
    ///
    /// The states in the manager are stackable, which allows maximum reusability of similar
    /// operations. For example, Follow, Check, and Goto all need to perform ImgStops at particular
    /// points, so one state, [`AutoMgrState::ImgStop`] is written which performs all these required
    /// actions. When one of the higher level states wants to perform an ImgStop, they just push the
    /// ImgStop state above them on the stack.
    ///
    /// This also allows easy actions to be performed at the end of a mode, for example Stop can be
    /// pushed below the current state, so that when it is poped the Stop state will be executed
    /// next.
    ///
    /// See [`AutoMgrStack`] for more information.
    stack: AutoMgrStack,
}

pub struct AutoMgrPersistantData {
    /// Global terrain map, the map describing all terrain previously observed by the rover.
    pub global_terr_map: TerrainMap,

    /// Instance of the [`LocMgr`] module, providing localisation source.
    pub loc_mgr: LocMgr,

    /// Telemetry packet to be sent by the TM server, summarising the autonomy state.
    pub auto_tm: AutoTm,

    /// A copy of the global session data.
    pub session: Session,
}

/// State stacking abstraction.
#[derive(Debug)]
pub struct AutoMgrStack(Vec<AutoMgrState>);

/// Output of a state's step function.
pub struct StepOutput {
    /// Action to perform on the stack itself
    pub action: StackAction,

    /// Data to pass to the state below this one
    pub data: StackData,
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Errors that can occur in the autonomy manager.
#[derive(Debug, thiserror::Error)]
pub enum AutoMgrError {
    #[error("Failed to load AutoMgrParams: {0:?}")]
    ParamLoadError(util::params::LoadError),

    #[error("Attempted to resume from pause but no mode was found to switch to, aborting.")]
    NoModeToResumeTo,

    #[error("Could not get a pose from the localisation module.")]
    PoseUnavailable,

    #[error("State of the Mnvr mode is not set.")]
    MnvrStateNotInit,

    // #[error("Navigation error: {0}")]
    // NavError(NavError),
    #[error("Navigation control type hasn't been initialised")]
    NavCtrlStateNotInit,

    #[error("Error in TrajCtrl: {0}")]
    TrajCtrlError(TrajCtrlError),

    #[error("Error in Path processing: {0}")]
    PathError(PathError),
}

#[derive(Debug)]
pub enum AutoMgrState {
    Stop(Stop),
    Pause(Pause),
    WaitNewPose(WaitNewPose),
    ImgStop,
    AutoMnvr(AutoMnvr),
    Follow(Follow),
    Check(Check),
    Goto,
}

/// Actions that can be performed on the Stack at the end of a state's step function.
#[derive(Debug)]
pub enum StackAction {
    None,
    Abort,
    Clear,
    PushAbove(AutoMgrState),
    PushBelow(AutoMgrState),
    Pop,
    Replace(AutoMgrState),
}

/// Possible data that can be passed out of a state's step function.
pub enum StackData {
    None,

    LocoCtrlMnvr(MnvrCmd),
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl AutoMgr {
    pub fn init(params_path: &str, session: Session) -> Result<Self, AutoMgrError> {
        // Load parameters
        let params: AutoMgrParams = match util::params::load(params_path) {
            Ok(p) => p,
            Err(e) => return Err(AutoMgrError::ParamLoadError(e)),
        };

        Ok(Self {
            params: params.clone(),
            last_stack_data: StackData::None,
            persistant: AutoMgrPersistantData::new(
                params.terrain_map_params.clone(),
                params.loc_mgr.source,
                session,
            )?,
            stack: AutoMgrStack::new(),
        })
    }

    pub fn step(&mut self, cmd: Option<AutoCmd>) -> Result<Option<MnvrCmd>, AutoMgrError> {
        // Get a reference to the current top state
        let top = self.stack.top();

        // Step the top, and get the action required by the state
        let output = match top {
            // Call the top's step function
            Some(top) => top.step(&self.params, &mut self.persistant, cmd),
            // If there is no top the mgr is off, but we can still accept some commands to change
            // state.
            None => {
                match cmd {
                    Some(AutoCmd::Manouvre(m)) => {
                        self.stack
                            .push_above(AutoMgrState::AutoMnvr(AutoMnvr::new(m)));
                        StepOutput::none()
                    }
                    Some(AutoCmd::Follow(p)) => {
                        self.stack.push_above(AutoMgrState::Follow(Follow::new(p)?));
                        StepOutput::none()
                    }
                    Some(AutoCmd::Check(p)) => {
                        self.stack.push_above(AutoMgrState::Check(Check::new(p)?));
                        StepOutput::none()
                    }
                    Some(_) => {
                        warn!("Cannot pause, resume, or abort Autonomy execution as the AutoMgr is Off");
                        return Ok(None);
                    }
                    None => return Ok(None),
                }
            }
        };

        let is_action = output.action.is_some();

        // Perform any actions required by the top state
        match output.action {
            StackAction::None => (),
            StackAction::Clear => self.stack.clear(),
            StackAction::Abort => {
                self.stack.clear();
                self.stack.push_above(AutoMgrState::Stop(Stop::new()))
            }
            StackAction::PushAbove(s) => self.stack.push_above(s),
            StackAction::PushBelow(s) => self.stack.push_below(s),
            StackAction::Pop => {
                self.stack.pop();
            }
            StackAction::Replace(s) => {
                self.stack.pop();
                self.stack.push_above(s)
            }
        }

        if self.stack.top().is_some() && is_action {
            info!("AutoMgr state change to: {}", self.stack.top().unwrap());
        }

        // Output data to loco_ctrl
        Ok(match output.data {
            StackData::None => None,
            StackData::LocoCtrlMnvr(m) => Some(m),
        })
    }

    pub fn is_off(&self) -> bool {
        self.stack.is_empty()
    }

    pub fn is_on(&self) -> bool {
        !self.stack.is_empty()
    }

    pub fn get_tm(&self) -> AutoTm {
        self.persistant.auto_tm.clone()
    }
}

impl AutoMgrStack {
    /// Create a new empty stack
    pub fn new() -> Self {
        Self(Vec::new())
    }

    /// Returns true if the stack is empty (has no states)
    pub fn is_empty(&self) -> bool {
        self.0.len() == 0
    }

    /// Returns a mutable reference of the top state in the stack. Returns None if the stack is
    /// empty.
    pub fn top(&mut self) -> Option<&mut AutoMgrState> {
        self.0.last_mut()
    }

    /// Pushes a new state onto the stack above the curren top
    pub fn push_above(&mut self, new: AutoMgrState) {
        self.0.push(new)
    }

    /// Pushes a new state onto the stack below the current top. If the stack is empty this is
    /// equivalent of [`AutoMgrStack::push_above()`].
    pub fn push_below(&mut self, new: AutoMgrState) {
        if self.is_empty() {
            self.0.push(new)
        } else {
            self.0.insert(self.0.len() - 1, new)
        }
    }

    /// Pops the current top of the stack, removing it. Returns None if the stack is empty.
    pub fn pop(&mut self) -> Option<AutoMgrState> {
        self.0.pop()
    }

    pub fn clear(&mut self) {
        self.0.clear()
    }
}

impl Display for AutoMgrState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AutoMgrState::Stop(_) => write!(f, "AutoMgrState::Stop"),
            AutoMgrState::Pause(_) => write!(f, "AutoMgrState::Pause"),
            AutoMgrState::WaitNewPose(_) => write!(f, "AutoMgrState::WaitNewPose"),
            AutoMgrState::ImgStop => write!(f, "AutoMgrState::ImgStop"),
            AutoMgrState::AutoMnvr(_) => write!(f, "AutoMgrState::AutoMnvr"),
            AutoMgrState::Follow(_) => write!(f, "AutoMgrState::Follow"),
            AutoMgrState::Check(_) => write!(f, "AutoMgrState::Check"),
            AutoMgrState::Goto => write!(f, "AutoMgrState::Goto"),
        }
    }
}

impl AutoMgrState {
    fn step(
        &mut self,
        params: &AutoMgrParams,
        persistant: &mut AutoMgrPersistantData,
        cmd: Option<AutoCmd>,
    ) -> StepOutput {
        let out = match self {
            AutoMgrState::Stop(stop) => stop.step(params, persistant, cmd),
            AutoMgrState::Pause(pause) => pause.step(params, persistant, cmd),
            AutoMgrState::WaitNewPose(wait) => wait.step(params, persistant, cmd),
            AutoMgrState::AutoMnvr(auto_mnvr) => auto_mnvr.step(params, persistant, cmd),
            AutoMgrState::Follow(follow) => follow.step(params, persistant, cmd),
            AutoMgrState::Check(check) => check.step(params, persistant, cmd),
            _ => unimplemented!(),
        };

        // If an output is an error, we print it to the screen but we actually abort, keeping the
        // system working
        match out {
            Ok(o) => o,
            Err(e) => {
                error!("{}", e);
                StepOutput {
                    action: StackAction::Abort,
                    data: StackData::None,
                }
            }
        }
    }
}

impl AutoMgrPersistantData {
    pub fn new(
        terr_map_params: CellMapParams,
        loc_source: LocSource,
        session: Session,
    ) -> Result<Self, AutoMgrError> {
        Ok(Self {
            global_terr_map: TerrainMap::new(terr_map_params),
            loc_mgr: LocMgr::new(loc_source),
            auto_tm: AutoTm::default(),
            session,
        })
    }
}

impl StepOutput {
    pub fn none() -> Self {
        Self {
            action: StackAction::None,
            data: StackData::None,
        }
    }
}

impl StackAction {
    pub fn is_some(&self) -> bool {
        match self {
            StackAction::None => false,
            _ => true,
        }
    }
}
