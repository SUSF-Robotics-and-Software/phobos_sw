//! # Check Mode
//!
//! Implements the check navigation mode, in which the rover attempts to follow a path uploaded by
//! ground and evaluates the cost of that path, to avoid obstacles along the path itself.

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use comms_if::tc::auto::{AutoCmd, PathSpec};

use crate::auto::{path::Path, traj_ctrl::TrajCtrl};

use super::{AutoMgrError, AutoMgrPersistantData, StepOutput, params::AutoMgrParams};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug)]
pub struct Check {
    traj_ctrl: TrajCtrl,
    ground_path_spec: PathSpec,
    ground_path: Option<Path>
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl Check {
    pub fn new(ground_path_spec: PathSpec) -> Result<Self, AutoMgrError> {
        // Create TrajCtrl instnace
        let traj_ctrl = TrajCtrl::init(
            "traj_ctrl.toml"
        ).map_err(|e| AutoMgrError::TrajCtrlError(e))?;

        Ok(Self {
            traj_ctrl,
            ground_path_spec,
            ground_path: None
        })
    }

    pub fn step(
        &mut self,
        _params: &AutoMgrParams,
        persistant: &mut AutoMgrPersistantData,
        cmd: Option<AutoCmd>
    ) -> Result<StepOutput, AutoMgrError> {

        todo!()
    }
}