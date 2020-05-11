//! Commands passed into LocoCtrl

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External imports
use serde::{Serialize, Deserialize};
use serde_json::{self, Value};
use thiserror::Error;
use eyre::WrapErr;
use color_eyre::Report;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A command to execute a particular manouvre
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct MnvrCommand {
    
    /// The type of manouvre to perform
    pub mnvr_type: MnvrType,

    /// The curvature (1/ radius of turn) to use. Used for Ackerman and Skid
    /// Steer
    /// 
    /// Units: 1/meters
    #[serde(alias = "curv_m")]
    #[serde(default)]
    pub curvature_m: Option<f64>,

    /// The speed with which the Rover body should move relative to the terrain.
    /// Used for Ackerman and Skid Steer.
    /// 
    /// Units: meters/second
    #[serde(default)]
    pub speed_ms: Option<f64>,

    /// The rate at which the rover shall turn about it's centre. Used for
    /// Point Turn only.
    /// 
    /// Units: radians/second
    #[serde(alias = "rate_rads")]
    #[serde(default)]
    pub turn_rate_rads: Option<f64>,
}

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Possible manouvres to be executed by LocoCtrl.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum MnvrType {
    /// No manouvre - interpreted as continue with last manouvre.
    #[serde(alias = "NONE")]
    None,

    /// Stop - bring the rover to a full stop.
    #[serde(alias = "STOP")]
    Stop,

    /// Ackerman - perform an ackerman coordinated turn (COR outside wheelbase)
    #[serde(alias = "ACKERMAN")]
    Ackerman,

    /// Point turn - perform a turn about the centre of the wheelbase
    #[serde(alias = "POINT_TURN")]
    PointTurn,

    /// Skid steer - tank like steering using differential speed between left 
    /// and right wheels.
    #[serde(alias = "SKID_STEER")]
    SkidSteer
}

#[derive(Debug, Error)]
pub enum ParseError {
    #[error("Command contains invalid JSON: {0}")]
    InvalidJson(serde_json::Error),

    #[error("Command is missing a manouvre type")]
    NoMnvrType,

    #[error("Invalid manouvre type {0}")]
    InvalidMnvrType(String),

    #[error("Invalid {0}: {1} (expected float)")]
    InvalidParam(String, String)
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl MnvrCommand {

    /// Determine if the command is valid (i.e. contains all required data).
    pub fn is_valid(&self) -> bool {
        match self.mnvr_type {
            MnvrType::Stop | MnvrType::None => true,
            MnvrType::Ackerman | MnvrType::SkidSteer => 
                !vec![self.curvature_m, self.speed_ms].contains(&None),
            MnvrType::PointTurn => 
                self.turn_rate_rads.is_some()
        }
    }

    /// Parse the command from a JSON payload
    pub fn from_json(json_str: &str) -> Result<Self, Report> {

        // Parse the json
        let cmd: MnvrCommand = serde_json::from_str(json_str)
            .wrap_err("Failed to deserialise manouvre command")?;

        Ok(cmd)

        // // Parse the json
        // let val: Value = serde_json::from_str(json_str)
        //     .wrap_err("Invalid JSON")?;
        //
        // // Get the command type
        // let mnvr_type = match val["mnvr_type"].is_string() {
        //     true => match MnvrType::from_str(
        //         &val["mnvr_type"].as_str().unwrap()) 
        //     {
        //         Some(t) => t,
        //         None => return Err(
        //             ParseError::InvalidMnvrType(String::from(
        //                 val["mnvr_type"].as_str().unwrap()))
        //         ).wrap_err("Unrecognised manouvre type.")
        //     },
        //     false => return Err(
        //         ParseError::InvalidMnvrType(format!("{}", val["mnvr_type"]))
        //     ).wrap_err("Expected a string for manouvre type")
        // };

        // // Fill in the parameters. It is LocoCtrl's job to verify that these
        // // are correct.
        // let curv_m = match val["curv_m"].is_f64() {
        //     true => Some(val["curv_m"].as_f64().unwrap()),
        //     false => match val["curv_m"].is_null() {
        //         true => None,
        //         false => return Err(ParseError::InvalidParam(
        //             String::from("Curvature"),
        //             format!("{}", val["curv_m"].to_string())
        //         ))
        //     }
        // };

        // let speed_ms = match val["speed_ms"].is_f64() {
        //     true => Some(val["speed_ms"].as_f64().unwrap()),
        //     false => match val["speed_ms"].is_null() {
        //         true => None,
        //         false => return Err(ParseError::InvalidParam(
        //             String::from("Speed"),
        //             format!("{}", val["speed_ms"].to_string())
        //         ))
        //     }
        // };

        // let rate_rads = match val["rate_rads"].is_f64() {
        //     true => Some(val["rate_rads"].as_f64().unwrap()),
        //     false => match val["rate_rads"].is_null() {
        //         true => None,
        //         false => return Err(ParseError::InvalidParam(
        //             String::from("Turn rate"),
        //             format!("{}", val["rate_rads"].to_string())
        //         ))
        //     }
        // };

        // Ok(Self {
        //     mnvr_type,
        //     curvature_m: curv_m,
        //     speed_ms,
        //     turn_rate_rads: rate_rads
        // })
    }
}

// impl MnvrType {
//     fn from_str(s: &str) -> Option<Self> {
//         match s {
//             "NONE" => Some(MnvrType::None),
//             "STOP" => Some(MnvrType::Stop),
//             "ACKERMAN" => Some(MnvrType::Ackerman),
//             "POINT_TURN" => Some(MnvrType::PointTurn),
//             "SKID_STEER" => Some(MnvrType::SkidSteer),
//             _ => None
//         }
//     }
// }