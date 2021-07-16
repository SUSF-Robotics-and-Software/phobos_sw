//! # [`AutoMgr<ImgStop>`] mode
//!
//! This mode is entered when the rover is stopped and acquires navigation data from the perloc
//! executable.

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use comms_if::{eqpt::perloc::PerlocCmd, tc::auto::AutoCmd};
use log::{info, warn};

use super::{
    states::{Pause, Stop},
    AutoMgrError, AutoMgrOutput, AutoMgrParams, AutoMgrPersistantData, AutoMgrState, StackAction,
    StepOutput,
};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Default)]
pub struct ImgStop {
    img_request_issued: bool,
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl ImgStop {
    pub fn new() -> Self {
        Self {
            img_request_issued: false,
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

        // If we aren't stopped push a new stop mode onto the stack
        if !persistant.is_stopped {
            return Ok(StepOutput {
                action: StackAction::PushAbove(AutoMgrState::Stop(Stop::new())),
                data: AutoMgrOutput::None,
            });
        }

        // Once stopped issue request for new images, clear the persistent depth image and request
        // a new one
        if !self.img_request_issued {
            info!("Requesting new depth image from perloc");
            persistant.depth_img = None;
            self.img_request_issued = true;
            return Ok(StepOutput {
                action: StackAction::None,
                data: AutoMgrOutput::PerlocCmd(PerlocCmd::AcqDepthFrame),
            });
        }

        // Wait until the depth image is set in the persistent data
        if let Some(ref depth_img) = persistant.depth_img {
            // Save the image data
            util::session::save_with_timestamp("depth_imgs/depth_img.json", depth_img.clone());

            Ok(StepOutput {
                action: StackAction::Pop,
                data: AutoMgrOutput::None,
            })
        }
        // If we havent got that just wait
        else {
            Ok(StepOutput {
                action: StackAction::None,
                data: AutoMgrOutput::None,
            })
        }
    }
}
