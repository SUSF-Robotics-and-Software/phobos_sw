//! # [`AutoMgr<Pause>`] implementation

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use comms_if::tc::auto::AutoCmd;
use log::warn;

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
pub struct Pause {
    stop_issued: bool
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl Pause {
    pub fn new() -> Self {
        Self {
            stop_issued: false
        }
    }

    pub fn step(
        &mut self, 
        _params: &AutoMgrParams,
        _persistant: &mut AutoMgrPersistantData, 
        cmd: Option<AutoCmd>
    ) ->  Result<StepOutput, AutoMgrError> {
        // The only command accepted in Pause is Abort or Resume, but the Stop must be completed
        // first. .
        match cmd {
            Some(AutoCmd::Abort) => {
                return Ok(StepOutput {
                    action: StackAction::Abort,
                    data: StackData::None
                })
            },
            Some(AutoCmd::Resume) => {
                return Ok(StepOutput {
                    action: StackAction::Pop,
                    data: StackData::None
                })
            },
            None => (),
            _ => {
                warn!(
                    "Only AutoCmd::Abort is accepted when in AutoMgrState::Pause, {:?} ignored", 
                    cmd
                );
            }
        }

        // Issue stop if not already done
        if !self.stop_issued {
            self.stop_issued = true;

            Ok(StepOutput {
                action: StackAction::PushAbove(AutoMgrState::Stop(Stop::new())),
                data: StackData::None
            })
        }
        else {
            Ok(StepOutput::none())
        }
    }
}