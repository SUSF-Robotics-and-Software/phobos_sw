//! # [`AutoMgrState::WaitNewPose`] implementation



// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use super::{
    AutoMgrError, 
    AutoMgrPersistantData, 
    AutoMgrState, 
    StackAction, 
    StackData, 
    StepOutput, 
    params::AutoMgrParams, 
    states::{
        Stop
    }
};
use comms_if::tc::auto::AutoCmd;
use log::{debug, error, warn};
use serde::Deserialize;
use util::session;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// WaitNewPose state.
///
/// This state is designed to wait for the LocMgr to be able to provide a new pose. It should be
/// used if the [`LocMgr::get_pose()`] function returns `None`. It will wait up to the provided time
/// duration before aborting.
#[derive(Debug)]
pub struct WaitNewPose {
    start_time: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct WaitNewPoseParams {
    /// Maximum duration to wait before aborting
    pub max_wait_time_s: f64
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl WaitNewPose {

    pub fn new() -> Self {
        Self {
            start_time: session::get_elapsed_seconds()
        }
    }

    pub fn step(
        &mut self, 
        params: &AutoMgrParams,
        persistant: &mut AutoMgrPersistantData, 
        cmd: Option<AutoCmd>
    ) -> Result<StepOutput, AutoMgrError> {
        // Get the current time
        let time_s = session::get_elapsed_seconds();

        // Check for pause or abort commands
        match cmd {
            Some(AutoCmd::Pause) => return Ok(StepOutput {
                action: StackAction::PushAbove(AutoMgrState::Pause),
                data: StackData::None
            }),
            Some(AutoCmd::Abort) => return Ok(StepOutput {
                action: StackAction::Replace(AutoMgrState::Stop(Stop::new())),
                data: StackData::None
            }),
            Some(_) => warn!("Only Pause and Abort commands are accepted in AutoMnvr state"),
            _ => ()
        };

        // Check if we have a new pose yet 
        if persistant.loc_mgr.get_pose().is_some() {
            debug!("Pose lock obtained, took {} s", time_s - self.start_time);
            // If we do pop self off the stack
            Ok(StepOutput {
                action: StackAction::Pop,
                data: StackData::None
            })
        }
        // Otherwise, check if the time we've waited is less than the time we're supposed to wait.
        else {
            if time_s - self.start_time > params.wait_new_pose.max_wait_time_s {
                // Clear the stack to abort
                error!(
                    "Couldn't get pose lock within {} s, aborting", 
                    params.wait_new_pose.max_wait_time_s
                );

                Ok(StepOutput {
                    action: StackAction::Clear,
                    data: StackData::None
                })
            }
            else {
                // Otherwise just output no action until we reach one of the other two conditions
                Ok(StepOutput::none())
            }
        }
    }
}