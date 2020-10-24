//! # Rover library.
//!
//! This library allows other crates in the workspace to access items defined inside the rover 
//! crate.

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

/// Camera client - requests and recieves images from the camera server
pub mod cam_client;

/// Electronics driver - controls the motors via the mot borads
#[deprecated(
    since = "0.2",
    note = "Replaced by MechExec/sim in architecture rework"
)]
pub mod elec_driver;

/// Localisation module - provides the rover with an idea of where it is in the world
pub mod loc;

/// Locomotion control module - converts high level manouvre commands into individual wheel commands
pub mod loco_ctrl;

/// Trajectory control module - keeps the rover on the given path
pub mod traj_ctrl;

/// Telecommand client - recieves telecommands from the tc server
pub mod tc_client;

/// Mechanisms client - sends actuator demands to the mechanisms server
#[cfg(feature = "mech")]
pub mod mech_client;

/// Simulation client - provides data directly from the simulation (webots)
pub mod sim_client;
