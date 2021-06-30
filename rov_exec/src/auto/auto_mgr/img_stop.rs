//! # [`AutoMgr<ImgStop>`] mode
//!
//! This mode is entered when the rover is stopped and acquires navigation data from the perloc
//! executable.

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use comms_if::tc::auto::AutoCmd;
use log::warn;

use super::{
    states::{Pause, Stop},
    AutoMgrError, AutoMgrOutput, AutoMgrParams, AutoMgrPersistantData, AutoMgrState, StackAction,
    StepOutput,
};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
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

        // Once stopped issue request for new images, clear the persistent depth image
        if !self.img_request_issued {
            persistant.depth_img = None;
            self.img_request_issued = true;
            return Ok(StepOutput {
                action: StackAction::None,
                data: AutoMgrOutput::RequestDepthImg,
            });
        }

        // Wait until the depth image is set in the persistent data
        if let Some(ref depth_img) = persistant.depth_img {
            // Create the iamge path and directory if needed
            let dir_path = persistant.session.session_root.clone().join("depth_imgs");
            std::fs::create_dir(&dir_path).expect("Couldn't create depth_imgs path");
            let depth_img_path =
                dir_path.join(format!("depth_img_{}.png", depth_img.timestamp.timestamp()));

            // Save the image
            if let Err(e) = depth_img.image.save(depth_img_path) {
                warn!("Couldn't save depth image: {}", e);
            }

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
