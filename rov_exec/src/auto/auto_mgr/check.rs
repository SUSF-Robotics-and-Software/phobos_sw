//! # Check Mode
//!
//! Implements the check navigation mode, in which the rover attempts to follow a path uploaded by
//! ground and evaluates the cost of that path, to avoid obstacles along the path itself.

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use comms_if::tc::auto::{AutoCmd, PathSpec};
use log::{info, warn};

use crate::auto::{
    auto_mgr::{
        states::Pause, stop::Stop, wait_new_pose::WaitNewPose, AutoMgrOutput, AutoMgrState,
        StackAction,
    },
    path::Path,
};

use super::{params::AutoMgrParams, AutoMgrError, AutoMgrPersistantData, StepOutput};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug)]
pub struct Check {
    ground_path_spec: PathSpec,
    ground_path: Option<Path>,
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl Check {
    pub fn new(ground_path_spec: PathSpec) -> Result<Self, AutoMgrError> {
        Ok(Self {
            ground_path_spec,
            ground_path: None,
        })
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
            Some(_) => warn!("Only Pause and Abort commands are accepted in Check state"),
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

        // Start the traverse if it hasn't been started yet
        if persistant.trav_mgr.is_off() {
            // Calculate the path from the current pose
            self.ground_path = Some(
                Path::from_path_spec(self.ground_path_spec.clone(), &current_pose)
                    .map_err(AutoMgrError::PathError)?,
            );

            // Start the traverse manager
            persistant
                .trav_mgr
                .start_check(self.ground_path.clone().unwrap())?;

            info!("Autonomous traverse started in Check mode");
        }

        // Step the traverse manager
        let mut trav_mgr_output = persistant
            .trav_mgr
            .step(persistant.depth_img.as_ref(), &current_pose)?;

        // If there's traj ctrl status set it in the tm
        persistant.auto_tm.traj_ctrl_status = trav_mgr_output.traj_ctrl_status.take();

        // If there's a new global cost map set it in the tm
        persistant.auto_tm.global_cost_map = trav_mgr_output.new_global_cost_map.take();

        // Take paths into the TM
        persistant.auto_tm.path = trav_mgr_output.primary_path.take();
        persistant.auto_tm.secondary_path = trav_mgr_output.secondary_path.take();

        // If travmgr has switched off we should replace ourself with a Stop mode
        if persistant.trav_mgr.is_off() {
            info!("Check traverse complete");
            Ok(StepOutput {
                action: StackAction::Replace(AutoMgrState::Stop(Stop::new())),
                data: AutoMgrOutput::None,
            })
        }
        // Otherwise output the step data
        else {
            Ok(trav_mgr_output.step_output)
        }
    }
}
