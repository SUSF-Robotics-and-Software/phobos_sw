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
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Telecommand processor
pub struct TcProcessor {

}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl TcProcessor {
    pub fn new() -> Self {
        TcProcessor {}
    }

    pub(crate) fn exec(&self, ds: &mut DataStore, tc: &Tc) {
        // Handle different TCs
        match tc.tc_type {
            // No action required
            TcType::None => (),
            TcType::Heartbeat => {
                // TODO: Respond with a heartbeat TM
                debug!("Recieved Heartbeat command");
            },
            TcType::LocoCtrlManouvre => {
                // Parse the payload into a MnvrCommand struct
                let mnvr_cmd = match tc.payload {
                    TcPayload::Json(ref p) => {
                        match MnvrCommand::from_json(p) {
                            Ok(m) => m,
                            Err(e) => {
                                warn!("Cannot parse TC into a loco_ctrl::MnvrCommand: {:?}", e);
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
            },
            TcType::MakeSafe => {
                // TODO: Raise safe flag
                debug!("Recieved MakeSafe command");
                ds.make_safe = true;
            },
            TcType::MakeUnsafe => {
                // TODO: Unset safe flag
                debug!("Recieved MakeUnsafe command");
                ds.make_safe = false;
            }
        }
    }
}