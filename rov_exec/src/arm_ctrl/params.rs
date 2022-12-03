//! Parameters structure for ArmCtrl

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use super::NUM_ROT_AXES;
use serde::{Serialize, Deserialize};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Parameters for Arm control.
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct Params {
    // ---- GEOMETRY ----
    /// The length of shoulder.
    ///
    /// Units: meters.
    pub shoulder_length_m: f64,

    /// The length of the elbow.
    ///
    /// Units: meters.
    pub elbow_length_m: f64,

    // ---- CAPABILITIES ----
    /// Maximum rotational axis absolute position (highest positive value)
    ///
    /// Units: radians
    pub max_abs_pos_rad: [f64; NUM_ROT_AXES],

    /// Minimum rotational axis absolute position (lowest negative value)
    ///
    /// Units: radians
    pub min_abs_pos_rad: [f64; NUM_ROT_AXES],

    /// Maximum axis rotation rate (highest positive value)
    ///
    /// Units: radians/second
    pub max_abs_rate_rads: [f64; NUM_ROT_AXES],

    /// Minimum axis rotation rate (lowest negative value)
    ///
    /// Units: radians/second
    pub min_abs_rate_rads: [f64; NUM_ROT_AXES],

    /// Default SAFE position of the arm motors.
    ///
    /// Units: radians
    pub default_pos_rad: [f64; NUM_ROT_AXES],
}
