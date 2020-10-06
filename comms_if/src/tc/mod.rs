//! # Telecommand module
//!
//! This module provdies a unified definition of a telecommand

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

pub mod loco_ctrl;

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::{Serialize, Deserialize};

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Telecommand
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum Tc {
    /// Set the rover into safe mode, disabling all motion of the vehicle. To re-enable the system
    /// the `MakeUnsafe` command must be issued.
    MakeSafe,

    /// Disable the rover's safe mode.
    MakeUnsafe,

    /// Send a direct manouvre command to locomotion control.
    LocoCtrlMnvr(loco_ctrl::MnvrCmd)
}

/// Response to an issued telecommand
#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub enum TcResponse {
    /// The TC was accepted and will be executed
    Ok,

    /// The TC message was invalid and could not be parsed
    Invalid,

    /// The TC cannot be executed because the rover is:
    /// 1. in safe mode
    CannotExecute
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl Tc {
    /// Parse a TC from a given json string
    pub fn from_json(json_str: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json_str)
    }
}