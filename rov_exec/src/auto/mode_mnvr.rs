//! # Manouvre Mode Module

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use log::{debug, info, warn};
use comms_if::tc::{auto::AutoMnvrCmd, loco_ctrl::MnvrCmd};
use crate::auto::{AutoMgr, AutoCmd, AutoMgrError, AutoMgrMode};

use super::{params::MnvrModeParams, loc::Pose};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

pub struct MnvrState {
    params: MnvrModeParams,

    auto_mnvr_cmd: AutoMnvrCmd,

    loco_ctrl_mnvr_issued: bool,

    last_pose: Option<Pose>,

    linear_distance_m: f64,

    angular_distance_rad: f64
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl AutoMgr {
    /// Execute the manouvre mode.
    pub(crate) fn mode_mnvr(&mut self) -> Result<(), AutoMgrError> {
        match &self.auto_cmd {
            // Pause execution
            Some(AutoCmd::Pause) => self.pause()?,
            // Or abort
            Some(AutoCmd::Abort) => self.abort()?,
            // Or execute the manouvre
            Some(AutoCmd::Manouvre(m)) => {
                // If the manouvre state is uninitialised
                if self.mnvr_state.is_none() {
                    self.mnvr_state = Some(MnvrState::init(
                        self.params.mnvr_mode_params.clone(), 
                        m.clone()
                    )?)
                }

                // Process the manouvre state
                self.output_mnvr_cmd = match self.mnvr_state {
                    Some(ref mut m) => m.proc()?,
                    None => return Err(AutoMgrError::MnvrStateNotInit)
                };

                // If stop clear the manouvre state and change into stop mode
                match self.output_mnvr_cmd {
                    Some(MnvrCmd::Stop) => {
                        info!("End of manouvre reached, stopping");
                        self.mnvr_state = None;
                        self.set_mode(AutoMgrMode::Stop);
                    },
                    _ => ()
                }
            },
            Some(_) => {
                warn!("Cannot execute AutoCmd as another is already running, abort before issuing\
                a new command");
            }
            // Do nothing for no command
            None => ()
        }
    
        Ok(())
    }
}

impl MnvrState {
    /// Intitialise the manouvre state
    fn init(params: MnvrModeParams, cmd: AutoMnvrCmd) -> Result<Self, AutoMgrError> {
        // Get the current pose
        Ok(Self {
            params,
            auto_mnvr_cmd: cmd,
            loco_ctrl_mnvr_issued: false,
            last_pose: crate::sim_client::rov_pose_lm(),
            linear_distance_m: 0.0,
            angular_distance_rad: 0.0,
        })
    }

    /// Process the manouvre state.
    ///
    /// This is done by calculating the linear and angular distance between the current and last 
    /// pose, then comparing this to the 
    fn proc(&mut self) -> Result<Option<MnvrCmd>, AutoMgrError> {
        // Manouvre command to output
        let mut mnvr_cmd: Option<MnvrCmd> = None;

        // Get current pose
        let current_pose = match crate::sim_client::rov_pose_lm() {
            Some(p) => p,
            None => return Err(AutoMgrError::PoseUnavailable)
        };

        // Calculate the linear distance delta
        let lin_dist_delta_m = match self.last_pose {
            Some(last_pose) => {
                util::maths::norm(
                    &current_pose.position_m_lm, 
                    &last_pose.position_m_lm
                ).unwrap()
            },
            None => {
                warn!("No last pose, linear distance change is 0 m");
                0.0
            }
        };

        // Calculate the angular distance delta
        let ang_dist_delta_rad = match self.last_pose {
            Some(last_pose) => {
                (current_pose.get_heading() - last_pose.get_heading()).abs()
            },
            None => {
                warn!("No last pose, angular distance change is 0 rad");
                0.0
            }
        };

        // Accumulate distances
        self.linear_distance_m += lin_dist_delta_m;
        self.angular_distance_rad += ang_dist_delta_rad;

        // Check end conditions
        match self.auto_mnvr_cmd {
            AutoMnvrCmd::Ackerman { 
                speed_ms,
                crab_rad,
                curv_m,
                dist_m
            } => {
                debug!("Linear distance error: {} m", dist_m - self.linear_distance_m);
                // If the error between the current distance and the target distance is lower than
                // the threshold the manouvre is complete and a stop can be issued.
                if dist_m - self.linear_distance_m
                    < 
                    self.params.linear_distance_threshold_m 
                {
                    mnvr_cmd = Some(MnvrCmd::Stop);
                }
                // If not at the target and the loco_ctrl manouvre hasn't been issued yet send it
                else if !self.loco_ctrl_mnvr_issued {
                    mnvr_cmd = Some(MnvrCmd::Ackerman {
                        speed_ms,
                        crab_rad,
                        curv_m
                    });

                    self.loco_ctrl_mnvr_issued = true;
                }
            },
            AutoMnvrCmd::PointTurn {
                rate_rads,
                dist_rad
            } => {
                debug!("Angular distance error: {} rad", dist_rad - self.angular_distance_rad);
                // If the error between the current distance and the target distance is lower than
                // the threshold the manouvre is complete and a stop can be issued.
                if dist_rad - self.angular_distance_rad
                    < 
                    self.params.angular_distance_threshold_rad 
                {
                    mnvr_cmd = Some(MnvrCmd::Stop);
                }
                // If not at the target and the loco_ctrl manouvre hasn't been issued yet send it
                else if !self.loco_ctrl_mnvr_issued {
                    mnvr_cmd = Some(MnvrCmd::PointTurn {
                        rate_rads
                    });

                    self.loco_ctrl_mnvr_issued = true;
                }
            }
        }

        // Update last pose
        self.last_pose = Some(current_pose);

        // Issue the command
        Ok(mnvr_cmd)
    }
}