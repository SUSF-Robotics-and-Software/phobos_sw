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

// Internal
pub use cmd::*;
pub use loco_config::*;
pub use params::*;
pub use state::*;

// ---------------------------------------------------------------------------
// CONSTANTS
// ---------------------------------------------------------------------------

/// The number of drive axes on the rover.
const NUM_DRV_AXES: usize = 6;

/// The number of steer axes on the rover (note although there are only 4 
/// active steer axes we use the same number of drive axes here so we can 
/// easily iterate through each pair of drive and steer axes).
const NUM_STR_AXES: usize = 6;

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Possible errors that can occur during LocoCtrl operation.
#[derive(Debug)]
pub enum Error {
    NotYetSupported(String),
    NoMnvrCmd,
    InvalidMnvrCmd(MnvrCommand),
}
