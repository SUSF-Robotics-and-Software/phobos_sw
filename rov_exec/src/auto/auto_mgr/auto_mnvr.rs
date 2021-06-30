//! # [`AutoMgr<AutoMnvr>`] implementation

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::Deserialize;

use comms_if::tc::{
    auto::{AutoCmd, AutoMnvrCmd},
    loco_ctrl::MnvrCmd,
};
use log::{info, warn};

use crate::auto::loc::Pose;

use super::{
    params::AutoMgrParams,
    states::{Pause, Stop, WaitNewPose},
    AutoMgrError, AutoMgrOutput, AutoMgrPersistantData, AutoMgrState, StackAction, StepOutput,
};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Debug)]
pub struct AutoMnvr {
    cmd: AutoMnvrCmd,
    start_pose: Option<Pose>,
    last_pose: Option<Pose>,

    /// Rather than displacement between the current pose and start pose we want to track linear
    /// distance covered, i.e. the circumfrence of our Ackermann not the distance between the start
    /// and end points.
    linear_distance_m: f64,

    loco_ctrl_cmd_issued: bool,
}

#[derive(Deserialize, Clone, Debug)]
pub struct AutoMnvrParams {
    /// The threshold within which the linear distance requirement will be considered fulfilled.
    pub linear_distance_threshold_m: f64,

    /// The threshold within which the angular distance requirement will be considered fulfilled.
    pub angular_distance_threshold_rad: f64,
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl AutoMnvr {
    pub fn new(cmd: AutoMnvrCmd) -> Self {
        Self {
            cmd,
            start_pose: None,
            last_pose: None,
            linear_distance_m: 0.0,
            loco_ctrl_cmd_issued: false,
        }
    }

    pub fn step(
        &mut self,
        params: &AutoMgrParams,
        persistant: &mut AutoMgrPersistantData,
        cmd: Option<AutoCmd>,
    ) -> Result<StepOutput, AutoMgrError> {
        // Check for pause or abort commands
        match cmd {
            Some(AutoCmd::Pause) => {
                // Set loco_ctrl_cmd_issued to false, so that when we resume we'll resend it
                self.loco_ctrl_cmd_issued = false;
                return Ok(StepOutput {
                    action: StackAction::PushAbove(AutoMgrState::Pause(Pause::new())),
                    data: AutoMgrOutput::None,
                });
            }
            Some(AutoCmd::Abort) => {
                return Ok(StepOutput {
                    action: StackAction::Abort,
                    data: AutoMgrOutput::None,
                })
            }
            Some(_) => warn!("Only Pause and Abort commands are accepted in AutoMnvr state"),
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

        // Stack output to return
        let mut output = StepOutput::none();

        // Calculate the linear distance change since last pose
        let lin_dist_delta_m = match self.last_pose {
            Some(last_pose) => (current_pose.position_m_lm - last_pose.position_m_lm).norm(),
            None => {
                warn!("No last pose, linear distance change is 0 m");
                0.0
            }
        };

        // Accumulate the linear distance
        self.linear_distance_m += lin_dist_delta_m;

        // If there's no start pose set it
        if self.start_pose.is_none() {
            self.start_pose = Some(current_pose);
        }

        // Calculate the angular distance between the current and start poses
        let angular_distance_rad = match self.start_pose {
            Some(start_pose) => start_pose
                .attitude_q_lm
                .angle_to(&current_pose.attitude_q_lm),
            None => unreachable!(),
        };

        // Check end conditions
        match self.cmd {
            AutoMnvrCmd::Ackerman {
                speed_ms,
                crab_rad,
                curv_m,
                dist_m,
            } => {
                // If the error between the current distance and the target distance is lower than
                // the threshold the manouvre is complete and a stop can be issued.
                if dist_m - self.linear_distance_m < params.auto_mnvr.linear_distance_threshold_m {
                    info!("AutoMnvr complete");
                    output.action = StackAction::Replace(AutoMgrState::Stop(Stop::new()))
                }
                // If not at the target and the loco_ctrl manouvre hasn't been issued yet send it
                else if !self.loco_ctrl_cmd_issued {
                    output.data = AutoMgrOutput::LocoCtrlMnvr(MnvrCmd::Ackerman {
                        speed_ms,
                        crab_rad,
                        curv_m,
                    });

                    self.loco_ctrl_cmd_issued = true;
                }
            }
            AutoMnvrCmd::PointTurn {
                rate_rads,
                dist_rad,
            } => {
                // If the error between the current distance and the target distance is lower than
                // the threshold the manouvre is complete and a stop can be issued.
                if dist_rad - angular_distance_rad < params.auto_mnvr.angular_distance_threshold_rad
                {
                    info!("AutoMnvr complete");
                    output.action = StackAction::Replace(AutoMgrState::Stop(Stop::new()))
                }
                // If not at the target and the loco_ctrl manouvre hasn't been issued yet send it
                else if !self.loco_ctrl_cmd_issued {
                    output.data = AutoMgrOutput::LocoCtrlMnvr(MnvrCmd::PointTurn { rate_rads });

                    self.loco_ctrl_cmd_issued = true;
                }
            }
        }

        // Update last pose
        self.last_pose = Some(current_pose);

        Ok(output)
    }
}
