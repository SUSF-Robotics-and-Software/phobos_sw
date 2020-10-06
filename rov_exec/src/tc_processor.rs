//! # Telecommand processor module
//!
//! The telecommand processor handles various TCs coming from any source.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use log::{warn, debug};

// Internal
use comms_if::tc::{Tc, TcType, TcPayload};
use super::DataStore;
use crate::loco_ctrl::MnvrCommand;

// ---------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ---------------------------------------------------------------------------

/// Execute a telecommand.
///
/// Mutates the datastore to send commands to different modules.
pub(crate) fn exec(ds: &mut DataStore, tc: &Tc) {
    // Handle different TCs
    match tc.tc_type {
        // Do nothing
        TcType::None => (),
        TcType::LocoCtrlManouvre => {
            exec_loco_ctrl_mnvr(ds, tc);
        },
        TcType::MakeSafe => {
            debug!("Recieved MakeSafe command");
            ds.make_safe(crate::SafeModeCause::MakeSafeTc);
        },
        TcType::MakeUnsafe => {
            debug!("Recieved MakeUnsafe command");
            ds.make_unsafe(crate::SafeModeCause::MakeSafeTc).ok();
        }
    }
}

// ---------------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ---------------------------------------------------------------------------

fn exec_loco_ctrl_mnvr(ds: &mut DataStore, tc: &Tc) {
    // Parse the payload into a MnvrCommand struct
    let mnvr_cmd = match tc.payload {
        TcPayload::Json(ref p) => {
            match MnvrCommand::from_json(p) {
                Ok(m) => m,
                Err(e) => {
                    warn!(
                        "Cannot parse TC into a loco_ctrl::MnvrCommand: {:?}", 
                        e)
                    ;
                    return
                }
            }
        },
        TcPayload::None => {
            warn!("Expected payload with a LocoCtrlManouvre command");
            return
        }
    };

    debug!("Recieved LocoCtrlManouvre command: {:#?}", mnvr_cmd);

    // Set the command in LocoCtrl's input
    ds.loco_ctrl_input.cmd = Some(mnvr_cmd);
}