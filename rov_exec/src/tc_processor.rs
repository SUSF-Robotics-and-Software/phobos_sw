//! # Telecommand processor module
//!
//! The telecommand processor handles various TCs coming from any source.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use log::{debug, warn};

// Internal
use crate::data_store::{DataStore, SafeModeCause};
use comms_if::tc::Tc;

// ---------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ---------------------------------------------------------------------------

/// Execute a telecommand.
///
/// Mutates the datastore to send commands to different modules.
pub(crate) fn exec(ds: &mut DataStore, tc: &Tc) {
    // Handle different Tcs
    match tc {
        Tc::MakeSafe => {
            debug!("Recieved MakeSafe command");
            ds.make_safe(SafeModeCause::MakeSafeTc);
        }
        Tc::MakeUnsafe => {
            debug!("Recieved MakeUnsafe command");
            ds.make_unsafe(SafeModeCause::MakeSafeTc).ok();
        }
        Tc::LocoCtrlMnvr(m) => ds.loco_ctrl_input.cmd = Some(*m),
        Tc::ArmCmd(m) => ds.arm_ctrl_input.cmd = Some(m.clone()),
        Tc::Autonomy(_) => {
            warn!("Autonomy command is not yet supported");
        }
    }
}
