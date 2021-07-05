//! # Telecommand module
//!
//! This module provdies a unified definition of a telecommand

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

pub mod auto;
pub mod loco_ctrl;

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use log::info;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use structopt::{clap::AppSettings, StructOpt};

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Telecommand
#[derive(Debug, Clone, Serialize, Deserialize, StructOpt)]
#[structopt(
    name = "tc",
    about = "Parse a telecommand to be sent to the rover",
    setting(AppSettings::NoBinaryName),
    global_setting(AppSettings::DeriveDisplayOrder),
    global_setting(AppSettings::DisableVersion),
    global_setting(AppSettings::DontCollapseArgsInUsage),
    global_setting(AppSettings::VersionlessSubcommands),
    global_setting(AppSettings::AllowNegativeNumbers)
)]
pub enum Tc {
    /// Set the rover into safe mode, disabling all motion of the vehicle. To re-enable the system
    /// the `MakeUnsafe` command must be issued.
    #[structopt(name = "safe")]
    MakeSafe,

    /// Disable the rover's safe mode.
    #[structopt(name = "unsafe")]
    MakeUnsafe,

    /// Send a direct manouvre command to locomotion control.
    #[structopt(name = "mnvr")]
    LocoCtrlMnvr(loco_ctrl::MnvrCmd),

    /// Perform a autonomous command.
    #[structopt(name = "auto")]
    Autonomy(auto::AutoCmd),
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
    CannotExecute,
}

/// Errors that can occur during parsing
#[derive(Debug, thiserror::Error, Serialize, Deserialize)]
pub enum TcParseError {
    #[error("Invalid JSON: {0}")]
    JsonError(String),

    #[error("Raw TC format error: {0}")]
    RawTcError(String),
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl Tc {
    /// Parse a TC from a given json string
    pub fn from_json(json_str: &str) -> Result<Self, TcParseError> {
        // Parse the JSON string to a value
        let json_value: Value = match serde_json::from_str(json_str) {
            Ok(v) => v,
            Err(e) => return Err(TcParseError::JsonError(e.to_string())),
        };

        // Print the value
        info!("{:#?}", json_value);

        // If the value is an object whos' only key is "raw_tc", the TC needs
        // processing
        if json_value.is_object() {
            let json_obj = json_value.as_object().unwrap();
            if json_obj.len() == 1 && json_obj.contains_key("raw_tc") {
                let raw_tc = json_obj.get("raw_tc").unwrap().as_str().unwrap();

                // Strip any spaces off the tc
                let raw_tc = raw_tc.trim();

                // Split on spaces to parse with structopt
                let cmd: Vec<&str> = raw_tc.split(' ').collect();

                // Get the clap matches for this TC
                let tc = match Tc::from_iter_safe(cmd) {
                    Ok(m) => Ok(m),
                    Err(e) => Err(TcParseError::RawTcError(format!("{:#}", e))),
                };

                return tc;
            }
        }

        serde_json::from_str(json_str).map_err(|e| TcParseError::JsonError(e.to_string()))
    }
}
