//! # Rover library.
//!
//! This library allows other crates in the workspace to access items defined inside the rover 
//! crate.

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

/// Autonomy module - provides autonomy mode control and high level autonomy functionality.
pub mod auto;

/// Camera client - requests and recieves images from the camera server
pub mod cam_client;

/// Electronics driver - controls the motors via the mot borads
#[deprecated(
    since = "0.2",
    note = "Replaced by MechExec/sim in architecture rework"
)]
pub mod elec_driver;

/// Locomotion control module - converts high level manouvre commands into individual wheel commands
pub mod loco_ctrl;

/// Telecommand client - recieves telecommands from the tc server
pub mod tc_client;

/// Mechanisms client - sends actuator demands to the mechanisms server
#[cfg(feature = "mech")]
pub mod mech_client;

/// Simulation client - provides data directly from the simulation (webots)
#[cfg(feature = "sim")]
pub mod sim_client;
