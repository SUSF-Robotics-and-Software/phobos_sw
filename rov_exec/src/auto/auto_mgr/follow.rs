//! # [`Follow`] AutoMgr state

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use comms_if::tc::auto::{AutoCmd, PathSpec};
use csv::Writer;
use log::{error, info, warn};
use std::fs::File;

use crate::auto::{path::Path, traj_ctrl::TrajCtrl};

use super::{
    params::AutoMgrParams,
    states::{Pause, Stop, WaitNewPose},
    AutoMgrError, AutoMgrOutput, AutoMgrPersistantData, AutoMgrState, StackAction, StepOutput,
};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug)]
pub struct Follow {
    traj_ctrl: TrajCtrl,
    path_spec: PathSpec,
    path: Option<Path>,
    tuning_writer: Option<Writer<File>>,
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl Follow {
    pub fn new(path_spec: PathSpec) -> Result<Self, AutoMgrError> {
        // Create TrajCtrl instnace
        let traj_ctrl =
            TrajCtrl::init("traj_ctrl.toml").map_err(|e| AutoMgrError::TrajCtrlError(e))?;

        Ok(Self {
            traj_ctrl,
            path_spec,
            path: None,
            tuning_writer: None,
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
            Some(_) => warn!("Only Pause and Abort commands are accepted in Follow state"),
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

        // If the path hasn't been calculated yet set it
        if self.path.is_none() {
            let path = Path::from_path_spec(self.path_spec.clone(), &current_pose)
                .map_err(|e| AutoMgrError::PathError(e))?;

            // Set the path in TrajCtrl. TrajCtrl accepts a path sequence, a vec of paths, and can
            // do heading adjustments between each path. We will just load our path as a single
            // path to simplify things.
            self.traj_ctrl
                .begin_path_sequence(vec![path.clone()])
                .map_err(|e| AutoMgrError::TrajCtrlError(e))?;

            // Set the path in the tm
            persistant.auto_tm.path = Some(path.clone());

            self.path = Some(path);

            // // Create a new CSV to store the tuning output for this path
            // self.tuning_writer = Some({
            //     let file_name = format!(
            //         "{:?}/traj_ctrl_tuning_{:.0}.csv",
            //         persistant.session.arch_root,
            //         util::session::get_elapsed_seconds()
            //     );

            //     Writer::from_path(&file_name).unwrap()
            // });
        }

        // Step TrajCtrl
        let (loco_ctrl_cmd, traj_ctrl_status) = self
            .traj_ctrl
            .proc(&current_pose)
            .map_err(|e| AutoMgrError::TrajCtrlError(e))?;

        // Set the traj_ctrl status in the tm
        persistant.auto_tm.traj_ctrl_status = Some(traj_ctrl_status);

        // Write tuning data out
        // match self.tuning_writer {
        //     Some(w) => w.serialize(&self.traj_ctrl.tuning_output),
        //     None => ()
        // }

        // Check for TrajCtrl finishing
        if traj_ctrl_status.sequence_finished {
            if traj_ctrl_status.sequence_aborted {
                error!("TrajCtrl aborted the path sequence");
            }
            info!("TrajCtrl sequence finished, exiting Follow mode");

            return Ok(StepOutput {
                action: StackAction::Replace(AutoMgrState::Stop(Stop::new())),
                data: AutoMgrOutput::None,
            });
        }

        // Output the loco_ctrl command
        match loco_ctrl_cmd {
            Some(mnvr) => Ok(StepOutput {
                action: StackAction::None,
                data: AutoMgrOutput::LocoCtrlMnvr(mnvr),
            }),
            None => Ok(StepOutput::none()),
        }
    }
}
