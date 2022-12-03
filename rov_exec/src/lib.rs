//! # Rover library.
//!
//! This library allows other crates in the workspace to access items defined inside the rover
//! crate.

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

/// Data Store - holds state of the entire rover software
pub mod data_store;

/// Camera client - requests and recieves images from the camera server
pub mod cam_client;

/// Localisation module - provides the rover with an idea of where it is in the world
pub mod loc;

/// Locomotion control module - converts high level manouvre commands into individual wheel commands
pub mod loco_ctrl;

// Arm control module - converts high level arm commands into individual joint commands
pub mod arm_ctrl;

/// Trajectory control module - keeps the rover on the given path
pub mod traj_ctrl;

/// Telecommand client - recieves telecommands from the tc server
pub mod tc_client;

/// Telemetry server - publishes telemetry
pub mod tm_server;

/// Mechanisms client - sends actuator demands to the mechanisms server
#[cfg(feature = "mech")]
pub mod mech_client;

/// Simulation client - provides data directly from the simulation (webots)
#[cfg(feature = "sim")]
pub mod sim_client;

// ---------------------------------------------------------------------------
// CONSTANTS
// ---------------------------------------------------------------------------

/// Target period of one cycle.
pub const CYCLE_PERIOD_S: f64 = 0.10;

/// Number of cycles per second
pub const CYCLE_FREQUENCY_HZ: f64 = 1.0 / CYCLE_PERIOD_S;

/// Limit of the number of times recieve errors from the mech server can be created consecutively
/// before safe mode will be engaged.
pub const MAX_MECH_RECV_ERROR_LIMIT: u64 = 5;
