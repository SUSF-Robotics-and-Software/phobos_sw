//! Provides kickstart operations which allows the global terrain and cost map to be populated
//! without planning a path

use comms_if::tc::auto::AutoCmd;
use log::{info, warn};

use crate::auto::{
    auto_mgr::{
        states::Pause, wait_new_pose::WaitNewPose, AutoMgrOutput, AutoMgrState, StackAction,
        StepOutput,
    },
    nav::trav_mgr,
};

use super::{AutoMgrError, AutoMgrParams, AutoMgrPersistantData};

// ------------------------------------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Default)]
pub struct KickStart {
    kickstart_started: bool,
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl KickStart {
    pub fn new() -> Self {
        Self {
            kickstart_started: false,
        }
    }

    pub fn step(
        &mut self,
        _params: &AutoMgrParams,
        persistant: &mut AutoMgrPersistantData,
        cmd: Option<AutoCmd>,
    ) -> Result<StepOutput, AutoMgrError> {
        // Check for pause or abort commands
        match cmd {
            Some(AutoCmd::Pause) => {
                return Ok(StepOutput {
                    action: StackAction::PushAbove(AutoMgrState::Pause(Pause::new())),
                    data: AutoMgrOutput::None,
                })
            }
            Some(AutoCmd::Abort) => {
                return Ok(StepOutput {
                    action: StackAction::Abort,
                    data: AutoMgrOutput::None,
                })
            }
            Some(_) => warn!("Only Pause and Abort commands are accepted in ImgStop state"),
            _ => (),
        };

        // Get the pose
        let current_pose = match persistant.loc_mgr.get_pose() {
            Some(p) => p,
            // If no pose push a wait for pose state
            None => {
                return Ok(StepOutput {
                    action: StackAction::PushAbove(AutoMgrState::WaitNewPose(WaitNewPose::new())),
                    data: AutoMgrOutput::None,
                })
            }
        };

        // Set the pose in the TM
        persistant.auto_tm.pose = Some(current_pose);

        // Start the kickstart
        if persistant.trav_mgr.is_off() && !self.kickstart_started {
            info!("Performing Kickstart");
            persistant.trav_mgr.kickstart()?;
            self.kickstart_started = true;
        }

        // Step the traverse manager and return its output
        let mut trav_mgr_output = persistant
            .trav_mgr
            .step(persistant.depth_img.as_ref(), &current_pose)?;

        // If there's traj ctrl status set it in the tm
        persistant.auto_tm.traj_ctrl_status = trav_mgr_output.traj_ctrl_status.take();

        // If theres a new cost map set it in the tm
        persistant.auto_tm.global_cost_map = trav_mgr_output.new_global_cost_map.take();

        // If travmgr has switched off we should pop ourself off the stack
        if persistant.trav_mgr.is_off() {
            info!("Kickstart complete");
            Ok(StepOutput {
                action: StackAction::Pop,
                data: AutoMgrOutput::None,
            })
        }
        // Otherwise output the step data
        else {
            Ok(trav_mgr_output.step_output)
        }
    }
}
