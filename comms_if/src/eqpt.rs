//! # Equipment Interface
//!
//! This module defines the interface structures which will be sent to equipment servers/clients.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::{Serialize, Deserialize};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// TODO
#[derive(Serialize, Deserialize, Debug)]
pub struct MechDems;

/// TODO
#[derive(Serialize, Deserialize, Debug)]
pub struct MechSensData;

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Response from the mechanisms server based on the demands sent by the client.
#[derive(Serialize, Deserialize, Debug)]
pub enum MechDemsResponse {
    /// Demands were valid and will be executed
    DemsOk,

    /// Demands were invalid and have been rejected
    DemsInvalid,

    /// Equipment is invalid so demands cannot be actuated
    EqptInvalid
}