//! # Data Store

use comms_if::{
    eqpt::{cam::CamImage, mech::MechDems},
    tc::auto::AutoCmd,
};
use log::{info, warn};

use crate::{
    auto::{auto_mgr::tm::AutoTm, loc::Pose},
    loco_ctrl,
};

// ---------------------------------------------------------------------------
// ENUMS
// ---------------------------------------------------------------------------

/// Gives the reason the rover has been put into safe mode
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum SafeModeCause {
    MakeSafeTc,
    TcClientNotConnected,
    MechClientNotConnected,
}

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Global data store for the executable.
#[derive(Default)]
pub struct DataStore {
    // Cycle management
    /// Number of cycles already executed
    pub num_cycles: u128,

    /// True if this cycle falls on a 1Hz boundary
    pub is_1_hz_cycle: bool,

    /// Simulation elapsed time
    pub sim_time_s: f64,

    // Safe mode variables
    /// Determines if the rover is in safe mode.
    pub safe: bool,

    /// Gives the reason for the rover being in safe mode.
    pub safe_cause: Option<SafeModeCause>,

    // Camera images
    pub left_cam_image: Option<CamImage>,
    pub right_cam_image: Option<CamImage>,

    // Localisation
    pub rov_pose_lm: Option<Pose>,

    // LocoCtrl
    pub loco_ctrl: loco_ctrl::LocoCtrl,
    pub loco_ctrl_input: loco_ctrl::InputData,
    pub loco_ctrl_output: MechDems,
    pub loco_ctrl_status_rpt: loco_ctrl::StatusReport,

    // Autonomy
    pub auto_cmd: Option<AutoCmd>,

    pub auto_tm: Option<AutoTm>,

    // Monitoring Counters
    /// Number of consecutive cycle overruns
    pub num_consec_cycle_overruns: u64,

    /// Number of consecutive mechanisms client recieve errors
    pub num_consec_mech_recv_errors: u64,
}

// ---------------------------------------------------------------------------
// IMPLS
// ---------------------------------------------------------------------------

impl DataStore {
    /// Puts the rover into safe mode with the given cause.
    pub fn make_safe(&mut self, cause: SafeModeCause) {
        if !self.safe {
            warn!("Make safe requested, cause: {:?}", cause);
            self.safe = true;
            self.safe_cause = Some(cause);

            // Make loco_ctrl safe
            self.loco_ctrl.make_safe();
        }
    }

    /// Attempts to disable the safe mode by clearing the given cause.
    ///
    /// Returns `Ok(())` if this cause was cleared and safe mode was disabled, or `Err(())`
    /// otherwise. To remove safe mode the provided cause must match the initial reason for safe
    /// mode being enabled.
    ///
    /// If safe mode was not enabled `Ok(())` is returned
    pub fn make_unsafe(&mut self, cause: SafeModeCause) -> Result<(), ()> {
        if !self.safe {
            return Ok(());
        }

        match self.safe_cause {
            Some(root_cause) => {
                if cause == root_cause {
                    self.safe = false;
                    self.safe_cause = None;
                    info!("Make unsafe requested, root cause match, safe mode disabled");
                    Ok(())
                } else {
                    // info!(
                    //     "Make unsafe requested, root cause ({:?}) differs from response ({:?}), \
                    //     rejected",
                    //     root_cause,
                    //     cause
                    // );
                    Err(())
                }
            }
            None => Ok(()),
        }
    }

    /// Perform actions required at the start of a cycle.
    ///
    /// Clears those items that need clearing at the start of a cycle, and sets the 1Hz cycle flag.
    pub fn cycle_start(&mut self, cycle_frequency_hz: f64) {
        if self.num_cycles % (cycle_frequency_hz as u128) == 0 {
            self.is_1_hz_cycle = true;
        } else {
            self.is_1_hz_cycle = false;
        }

        self.loco_ctrl_input = loco_ctrl::InputData::default();
        self.loco_ctrl_output = MechDems::default();
        self.loco_ctrl_status_rpt = loco_ctrl::StatusReport::default();

        self.sim_time_s = util::session::get_elapsed_seconds();
    }
}
