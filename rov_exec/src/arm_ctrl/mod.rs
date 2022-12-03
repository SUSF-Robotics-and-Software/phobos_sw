//! Arm control module

// ---------------------------------------------------------------------------
// MODULES
// ---------------------------------------------------------------------------

mod inverse_kinematics;
mod params;
mod state;

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// Internal
pub use params::*;
pub use state::*;

// ---------------------------------------------------------------------------
// CONSTANTS
// ---------------------------------------------------------------------------

/// The number of rotational axes on the rover arm.
pub const NUM_ROT_AXES: usize = 5;

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Possible errors that can occur during ArmCtrl operation.
#[derive(Debug, thiserror::Error)]
pub enum ArmCtrlError {
    #[error("Action not yet supported: {0}")]
    NotYetSupported(String),

    #[error("Expected there to be an arm command but couldn't find one")]
    NoArmCmd,

    #[error("Recieved an invalid arm command")]
    InvalidArmCmd,
}
