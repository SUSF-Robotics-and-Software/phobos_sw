//! #  [`AutoMgr<Stop>`] implementation

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use super::{
    params::AutoMgrParams, states::WaitNewPose, AutoMgrError, AutoMgrOutput, AutoMgrPersistantData,
    AutoMgrState, StackAction, StepOutput,
};
use crate::auto::loc::Pose;
use comms_if::tc::{auto::AutoCmd, loco_ctrl::MnvrCmd};
use log::{info, warn};
use serde::Deserialize;
use util::session;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Stop state of the AutoMgr.
///
/// Possible transitions, triggered on detection of stop from Loc:
/// - Off
/// - Pause
/// - AutoMnvr,
/// - Follow,
/// - Check,
/// - Goto,
/// - ImgStop
#[derive(Debug)]
pub struct Stop {
    /// Time at which the rover was first considered to be stationary
    stationary_start_time_s: f64,

    /// Pose at the last stationary check
    last_pose: Option<Pose>,

    /// Whether or not the stop command has been issued
    loco_ctrl_cmd_issued: bool,
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct StopParams {
    /// Time in seconds the rover must be stationary before it is considered stopped
    min_stationary_time_s: f64,

    /// Maximum position change magnitude that will still be considered stationary
    position_delta_max_magn_m: f64,

    /// Maximum attitude change magnitude that will be considered stationary
    attitude_delta_max_magn_rad: f64,
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl Stop {
    pub fn new() -> Self {
        Self {
            stationary_start_time_s: 0.0,
            last_pose: None,
            loco_ctrl_cmd_issued: false,
        }
    }

    pub fn step(
        &mut self,
        params: &AutoMgrParams,
        persistant: &mut AutoMgrPersistantData,
        cmd: Option<AutoCmd>,
    ) -> Result<StepOutput, AutoMgrError> {
        // The only command accepted in Stop is Abort, which will clear the stack.
        match cmd {
            Some(AutoCmd::Abort) => {
                return Ok(StepOutput {
                    action: StackAction::Clear,
                    data: AutoMgrOutput::None,
                })
            }
            None => (),
            _ => {
                warn!(
                    "Only AutoCmd::Abort is accepted when in AutoMgrState::Stop, {:?} ignored",
                    cmd
                );
            }
        }

        // Get the current pose.
        // If no pose request a WaitNewPose state be pushed
        let current_pose = match persistant.loc_mgr.get_pose() {
            Some(p) => p,
            None => {
                return Ok(StepOutput {
                    action: StackAction::PushAbove(AutoMgrState::WaitNewPose(WaitNewPose::new())),
                    data: AutoMgrOutput::None,
                })
            }
        };

        // Set the pose in the TM
        persistant.auto_tm.pose = Some(current_pose);

        // Get the current rover time
        let current_time_s = session::get_elapsed_seconds();

        // If the last pose is none we cannot be sure if we're stopped, so we just set the last pose
        // as the current and wait until the next cycle. Also set the start time to the current time
        let last_pose = match self.last_pose {
            Some(ref p) => p,
            None => {
                self.last_pose = Some(current_pose);
                self.stationary_start_time_s = current_time_s;

                return Ok(StepOutput::none());
            }
        };

        // Calculate the position change since last pose
        let pos_delta_magn_m = (current_pose.position_m - last_pose.position_m).norm();

        // If the position change is greater than given in the parameter the rover has moved, reset
        // the stationary time
        if pos_delta_magn_m > params.stop.position_delta_max_magn_m {
            self.stationary_start_time_s = current_time_s;
        }

        // Calculate the angle rotated between the last and current pose
        let att_delta_magn_rad = last_pose.attitude_q.angle_to(&current_pose.attitude_q);

        // If that angle is greater than the limit the rover has moved, reset the stationary time
        if att_delta_magn_rad > params.stop.attitude_delta_max_magn_rad {
            self.stationary_start_time_s = current_time_s;
        }

        // Update the last pose to the current one
        self.last_pose = Some(current_pose);

        // If the difference between the current time and the stationary start time is greater than
        // the limit in the parameters the rover has been stationary for the required number of
        // seconds, and we can exit successfully.
        let stationary_time_s = current_time_s - self.stationary_start_time_s;
        if stationary_time_s > params.stop.min_stationary_time_s {
            info!(
                "Rover stationary for {} s, AutoMgrState::Stop complete successfully",
                stationary_time_s
            );

            // Set the stopped flag in the persistent data
            persistant.is_stopped = true;

            Ok(StepOutput {
                action: StackAction::Pop,
                data: AutoMgrOutput::None,
            })
        }
        // Otherwise we exit, waiting for the next cycle to check again
        else {
            // If the loco ctrl command hasn't been issued we set it now
            if !self.loco_ctrl_cmd_issued {
                self.loco_ctrl_cmd_issued = true;
                Ok(StepOutput {
                    action: StackAction::None,
                    data: AutoMgrOutput::LocoCtrlMnvr(MnvrCmd::Stop),
                })
            } else {
                Ok(StepOutput::none())
            }
        }
    }
}
