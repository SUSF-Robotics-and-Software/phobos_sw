//! Locomotion control module

// ---------------------------------------------------------------------------
// MODULES
// ---------------------------------------------------------------------------

mod cmd;
mod loco_config;
mod params;
mod state;
mod calc_ackerman;
mod calc_point_turn;
mod calc_skid_steer;

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use thiserror::Error;

// Internal
pub use cmd::*;
pub use loco_config::*;
pub use params::*;
pub use state::*;

// ---------------------------------------------------------------------------
// CONSTANTS
// ---------------------------------------------------------------------------

/// The number of drive axes on the rover.
pub const NUM_DRV_AXES: usize = 6;

/// The number of steer axes on the rover (note although there are only 4 
/// active steer axes we use the same number of drive axes here so we can 
/// easily iterate through each pair of drive and steer axes).
pub const NUM_STR_AXES: usize = 6;

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Possible errors that can occur during LocoCtrl operation.
#[derive(Debug, Error)]
pub enum LocoCtrlError {
    #[error("Action not yet supported: {0}")]
    NotYetSupported(String),

    #[error("Expected there to be a manouvre command but couldn't find one")]
    NoMnvrCmd,

    #[error("Recieved an invalid manouvre command: {0:#?}")]
    InvalidMnvrCmd(MnvrCommand),
}
