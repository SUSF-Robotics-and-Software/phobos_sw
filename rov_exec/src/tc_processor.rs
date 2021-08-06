//! # Telecommand processor module
//!
//! The telecommand processor handles various TCs coming from any source.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use log::{debug, info};

// Internal
use crate::data_store::{DataStore, SafeModeCause};
use comms_if::tc::Tc;

// ---------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ---------------------------------------------------------------------------

/// Execute a telecommand.
///
/// Mutates the datastore to send commands to different modules.
pub fn exec(ds: &mut DataStore, tc: &Tc) {
    info!("New TC: {:#?}", tc);

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
        Tc::Autonomy(a) => {
            ds.auto_cmd = Some(a.clone());
        }
    }
}
