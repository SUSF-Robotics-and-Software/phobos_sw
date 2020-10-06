//! # Rover Executable Parameters
//!
//! This module provide parameters for the rover executable.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::{Serialize, Deserialize};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Serialize, Deserialize)]
pub struct RovExecParams {

    /// Network endpoint for the mechanisms demands socket
    pub mech_dems_endpoint: String,

    /// Network endpoint for the mechanisms sensor data socket
    pub mech_sens_endpoint: String,

    /// Network endpoint for the camera socket
    pub cam_endpoint: String,

    /// Network endpoint for the telecommand client
    pub tc_endpoint: String
}