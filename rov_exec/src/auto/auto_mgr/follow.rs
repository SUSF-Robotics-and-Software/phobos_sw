//! # [`Follow`] AutoMgr state

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use comms_if::tc::auto::{AutoCmd, PathSpec};
use log::{error, info, warn};

use crate::auto::{path::Path, traj_ctrl::TrajCtrl};

use super::{
    AutoMgrError, 
    AutoMgrPersistantData, 
    StepOutput, 
    StackData, 
    StackAction, 
    AutoMgrState, 
    params::AutoMgrParams,
    states::{
        Stop,
        Pause,
        WaitNewPose
    }
};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug)]
pub struct Follow {
    traj_ctrl: TrajCtrl,
    path_spec: PathSpec,
    path: Option<Path>
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl Follow {
    pub fn new(path_spec: PathSpec) -> Result<Self, AutoMgrError> {
        // Create TrajCtrl instnace
        let traj_ctrl = TrajCtrl::init(
            "traj_ctrl.toml"
        ).map_err(|e| AutoMgrError::TrajCtrlError(e))?;

        Ok(Self {
            traj_ctrl,
            path_spec,
            path: None
        })
    }

    pub fn step(
        &mut self,
        params: &AutoMgrParams,
        persistant: &mut AutoMgrPersistantData,
        cmd: Option<AutoCmd>
    ) -> Result<StepOutput, AutoMgrError> {
        // Check for pause or abort commands
        match cmd {
            Some(AutoCmd::Pause) => {
                return Ok(StepOutput {
                    action: StackAction::PushAbove(AutoMgrState::Pause(Pause::new())),
                    data: StackData::None
                })
            },
            Some(AutoCmd::Abort) => return Ok(StepOutput {
                action: StackAction::Abort,
                data: StackData::None
            }),
            Some(_) => warn!("Only Pause and Abort commands are accepted in Follow state"),
            _ => ()
        };

        // Get the pose
        let current_pose = match persistant.loc_mgr.get_pose() {
            Some(p) => p,
            // If no pose push a wait for pose state
            None => return Ok(StepOutput {
                action: StackAction::PushAbove(AutoMgrState::WaitNewPose(WaitNewPose::new())),
                data: StackData::None,
            })
        };

        // If the path hasn't been calculated yet set it
        if self.path.is_none() {
            let path = Path::from_path_spec(self.path_spec.clone(), &current_pose)
                .map_err(|e| AutoMgrError::PathError(e))?;

            // Set the path in TrajCtrl. TrajCtrl accepts a path sequence, a vec of paths, and can
            // do heading adjustments between each path. We will just load our path as a single
            // path to simplify things.
            self.traj_ctrl.begin_path_sequence(vec![path.clone()])
                .map_err(|e| AutoMgrError::TrajCtrlError(e))?;

            self.path = Some(path);
        }

        // Step TrajCtrl
        let (loco_ctrl_cmd, traj_ctrl_status) = self.traj_ctrl.proc(
            &current_pose
        ).map_err(|e| AutoMgrError::TrajCtrlError(e))?;

        // Check for TrajCtrl finishing
        if traj_ctrl_status.sequence_finished {
            if traj_ctrl_status.sequence_aborted {
                error!("TrajCtrl aborted the path sequence");
            }
            info!("TrajCtrl sequence finished, exiting Follow mode");

            return Ok(StepOutput {
                action: StackAction::Replace(AutoMgrState::Stop(Stop::new())),
                data: StackData::None
            })
        }

        // Output the loco_ctrl command
        match loco_ctrl_cmd {
            Some(mnvr) => Ok(StepOutput {
                action: StackAction::None,
                data: StackData::LocoCtrlMnvr(mnvr)
            }),
            None => Ok(StepOutput::none())
        }
        
    }
}