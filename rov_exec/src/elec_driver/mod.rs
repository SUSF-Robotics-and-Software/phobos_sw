//! # Electronics driver module
//!
//! This module interfaces with the rover's motors and enables commands to be 
//! sent to them. It takes in a `loco_ctrl::OutputData` struct with the motor
//! demands and will issue those demands using the Adafruit ServoKit python 
//! library.

// ---------------------------------------------------------------------------
// MODULES
// ---------------------------------------------------------------------------

pub mod state;
pub mod params;

// ---------------------------------------------------------------------------
// EXPORTS
// ---------------------------------------------------------------------------

pub use state::*;
pub use params::*;