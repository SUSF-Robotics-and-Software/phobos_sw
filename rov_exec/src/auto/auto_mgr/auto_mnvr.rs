//! # [`AutoMgr<AutoMnvr>`] implementation

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use comms_if::tc::auto::{AutoCmd, AutoMnvrCmd};
use log::warn;

use crate::auto::loc::Pose;

use super::{
    AutoMgrError, 
    AutoMgrPersistantData, 
    AutoMgrState, 
    StackAction, 
    StackData, 
    StepOutput, 
    params::AutoMgrParams, 
    states::{
        WaitNewPose,
        Stop
    }
};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Debug)]
pub struct AutoMnvr {
    cmd: AutoMnvrCmd,
    start_pose: Option<Pose>
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl AutoMnvr {
    pub fn new(cmd: AutoMnvrCmd) -> Self {
        Self {
            cmd,
            start_pose: None
        }
    }

    pub fn step(
        &mut self, 
        params: &AutoMgrParams,
        persistant: &mut AutoMgrPersistantData, 
        cmd: Option<AutoCmd>
    ) ->  Result<StepOutput, AutoMgrError> {
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

        // Get the pose
        let pose = match persistant.loc_mgr.get_pose() {
            Some(p) => p,
            // If no pose push a wait for pose state
            None => return Ok(StepOutput {
                action: StackAction::PushAbove(AutoMgrState::WaitNewPose(WaitNewPose::new())),
                data: StackData::None,
            })
        };

        // If there's no start pose set it
        if self.start_pose.is_none() {
            self.start_pose = Some(pose);
        }

        todo!()
    }
}