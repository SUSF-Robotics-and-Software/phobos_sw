//! # Telecommand module
//!
//! This module provides telecommand functionality to the communications 
//! interface.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use serde::{Serialize, Deserialize};
use serde_json::{self, Value};
use thiserror::Error;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A telecommand, i.e. an instruction sent to the rover by the ground station.
#[derive(Serialize, Deserialize)]
pub struct Tc
{
    /// The type of the telecommand
    pub tc_type: TcType,

    /// The payload associated with this TC
    pub payload: TcPayload
}

// ---------------------------------------------------------------------------
// STATICS
// ---------------------------------------------------------------------------

static TYPE_HAS_NO_PAYLOAD: [TcType; 3] = [
    TcType::None,
    TcType::MakeSafe,
    TcType::MakeUnsafe
];

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Telecommand types. 
///
/// The type is used to identify the purpose of the telecommand, and should be
/// used by the rover's telecommand processor to determine where to send the 
/// command.
#[derive(Debug, Serialize, Deserialize, Eq, PartialEq)]
pub enum TcType {
    None,
    Heartbeat,
    MakeSafe,
    MakeUnsafe,
    LocoCtrlManouvre,
}

/// Telecommand payload.
///
/// The payload allows the data contained in the TC to be serialised in may 
/// ways. The payload only indicates which serialisation format the data is in.
/// It is up to the user to properly deserialise the data contained within it.
#[derive(Debug, Serialize, Deserialize)]
pub enum TcPayload {
    None,
    Json(String)
}

/// Possible parsing errors.
#[derive(Debug, Error)]
pub enum TcParseError {
    #[error("TC contains invalid JSON: {0}")]
    InvalidJson(serde_json::Error),

    #[error("TC has an invalid type ({0})")]
    InvalidType(String),

    #[error("TC of type {0:?} is expected to have a payload but it doesn't")]
    MissingPayload(TcType)
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Tc {

    /// Parse a new TC from a JSON packet
    pub fn from_json(json_str: &str) -> Result<Self, TcParseError> {
        // Parse the JSON string into a value
        let val: Value = match serde_json::from_str(json_str) {
            Ok(v) => v,
            Err(e) => return Err(TcParseError::InvalidJson(e))
        };

        // Get the type of the TC
        let tc_type = match TcType::from_str(
            match val["type"].as_str() {
                Some(s) => s,
                None => return Err(TcParseError::InvalidType(String::from(
                    "Expected \"type\" to be a string"
                )))
            })
        {
            Some(t) => t,
            None => return Err(TcParseError::InvalidType(
                format!(
                    "{} is not a recognised TC type", 
                    val["type"].as_str().unwrap())
            ))
        };
        
        // Get the payload. If it's null and the type does not have a payload 
        // then an error is returned
        if val["payload"].is_null()
            &&
            !TYPE_HAS_NO_PAYLOAD.contains(&tc_type)
        {
            return Err(TcParseError::MissingPayload(tc_type))
        }

        Ok(Tc {
            tc_type,
            payload: TcPayload::Json(val["payload"].to_string())
        })
    }
}

impl TcType {
    fn from_str(s: &str) -> Option<Self> {
        match s {
            "NONE" => Some(TcType::None),
            "HEARTBEAT" => Some(TcType::Heartbeat),
            "SAFE" => Some(TcType::MakeSafe),
            "UNSAFE" => Some(TcType::MakeUnsafe),
            "MNVR" => Some(TcType::LocoCtrlManouvre),
            _ => None
        }
    }
}