//! # Equipment Interface
//!
//! This module defines the interface structures which will be sent to equipment servers/clients.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::{Serialize, Deserialize};
use std::collections::HashMap;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------


#[derive(Serialize, Deserialize, Debug)]
pub struct MechDems {
	pub pos_rad: HashMap<ActId, f64>,
	pub speed_rad_s: HashMap<ActId, f64>
}

/// TODO
#[derive(Serialize, Deserialize, Debug)]
pub struct MechSensData;



// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(Serialize, Deserialize, Debug, Hash, Eq, PartialEq)]
pub enum ActId {
	DrvFL,
	DrvML,
	DrvRL,
	DrvFR,
	DrvMR,
	DrvRR,
	StrFL,
	StrML,
	StrRL,
	StrFR,
	StrMR,
	StrRR,
	ArmBase,
	ArmShoulder,
	ArmElbow,
	ArmWrist,
	ArmGrabber
}

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