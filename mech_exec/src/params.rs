//! # Mechanisms Executable Parameters

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::Deserialize;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Deserialize, Default)]
pub struct MechExecParams {

    /// Endpoint for the demands socket
    pub demands_endpoint: String,

    /// Endpoint for the sensor data socket
    pub sensor_data_endpoint: String,
}