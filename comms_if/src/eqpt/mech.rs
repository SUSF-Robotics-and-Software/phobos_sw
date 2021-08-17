//! # Mechanisms Equipment Commands

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::{Deserialize, Serialize};
use std::{collections::HashMap, str::FromStr};
use structopt::StructOpt;

// ------------------------------------------------------------------------------------------------
// CONSTANTS
// ------------------------------------------------------------------------------------------------

const ARM_IDS: [ActId; 5] = [
    ActId::ArmBase,
    ActId::ArmShoulder,
    ActId::ArmElbow,
    ActId::ArmWrist,
    ActId::ArmGrabber,
];

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Demands that are sent from the MechClient to the MechServer
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MechDems {
    /// The demanded position of an actuator in radians.
    pub pos_rad: HashMap<ActId, f64>,

    /// The demanded speed of an actuator in radians
    pub speed_rads: HashMap<ActId, f64>,
}

/// Sensor data returned by the MechServer to the MechClient
/// TODO
#[derive(Serialize, Deserialize, Debug)]
pub struct MechSensData;

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// IDs of all actuators available to the rover
#[derive(Serialize, Deserialize, Debug, Hash, Eq, PartialEq, Copy, Clone)]
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
    ArmGrabber,
}

/// Response from the mechanisms server based on the demands sent by the client.
#[derive(Serialize, Deserialize, Debug)]
pub enum MechDemsResponse {
    /// Demands were valid and will be executed
    DemsOk,

    /// Demands were invalid and have been rejected
    DemsInvalid,

    /// Equipment is invalid so demands cannot be actuated
    EqptInvalid,
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl ActId {
    pub fn arm_ids() -> &'static [Self] {
        &ARM_IDS
    }
}

impl Default for MechDems {
    fn default() -> Self {
        let mut pos_rad = HashMap::new();
        let mut speed_rad_s = HashMap::new();

        pos_rad.insert(ActId::StrFL, 0.0);
        pos_rad.insert(ActId::StrML, 0.0);
        pos_rad.insert(ActId::StrRL, 0.0);
        pos_rad.insert(ActId::StrFR, 0.0);
        pos_rad.insert(ActId::StrMR, 0.0);
        pos_rad.insert(ActId::StrRR, 0.0);

        pos_rad.insert(ActId::ArmBase, 0.0);
        pos_rad.insert(ActId::ArmShoulder, 0.0);
        pos_rad.insert(ActId::ArmElbow, 0.0);
        pos_rad.insert(ActId::ArmWrist, 0.0);
        pos_rad.insert(ActId::ArmGrabber, 0.0);

        speed_rad_s.insert(ActId::DrvFL, 0.0);
        speed_rad_s.insert(ActId::DrvML, 0.0);
        speed_rad_s.insert(ActId::DrvRL, 0.0);
        speed_rad_s.insert(ActId::DrvFR, 0.0);
        speed_rad_s.insert(ActId::DrvMR, 0.0);
        speed_rad_s.insert(ActId::DrvRR, 0.0);

        Self {
            pos_rad,
            speed_rads: speed_rad_s,
        }
    }
}

impl MechDems {
    /// Merges `other` into `self`. If `other` contains duplicate keys to `self`, the values from
    /// `self` are used instead.
    pub fn merge(&mut self, other: &Self) {
        for (&act_id, &pos) in other.pos_rad.iter() {
            if !self.pos_rad.contains_key(&act_id) {
                self.pos_rad.insert(act_id, pos);
            }
        }

        for (&act_id, &speed) in other.speed_rads.iter() {
            if !self.speed_rads.contains_key(&act_id) {
                self.speed_rads.insert(act_id, speed);
            }
        }
    }
}
