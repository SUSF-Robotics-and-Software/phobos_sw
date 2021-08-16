//! Parameters structure for LocoCtrl

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use serde::Deserialize;
use super::{NUM_STR_AXES, NUM_DRV_AXES};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Parameters for Locomotion control.
#[derive(Debug, Default, Deserialize)]
pub struct Params {

    // ---- GEOMETRY ----

    /// The radius of the rover's wheels.
    ///
    /// Units: meters.
    pub wheel_radius_m: f64,

    /// The position of the steer axes in the rover body frame.
    ///
    /// Units: meters,
    /// Frame: Rover body
    pub str_axis_pos_m_rb: [[f64; 3]; NUM_STR_AXES],

    /// The position of the drive axes in the rover body frame.
    ///
    /// Units: meters,
    /// Frame: Rover body
    pub drv_axis_pos_m_rb: [[f64; 3]; NUM_DRV_AXES],

    // ---- CAPABILITIES ----

    /// Maximum steer axis absolute position (highest positive value)
    ///
    /// Units: radians
    pub str_max_abs_pos_rad: [f64; NUM_STR_AXES],

    /// Minimum steer axis absolute position (lowest negative value)
    ///
    /// Units: radians
    pub str_min_abs_pos_rad: [f64; NUM_STR_AXES],

    /// Maximum drive axis rate (highest positive value)
    ///
    /// Units: radians/second
    pub drv_max_abs_rate_rads: [f64; NUM_DRV_AXES],

    /// Minimum drive axis rate (lowest negative value)
    ///
    /// Units: radians/second
    pub drv_min_abs_rate_rads: [f64; NUM_DRV_AXES],

    /// Minimum curvature possible under an ackerman command.
    ///
    /// Units: 1/meters
    pub ackerman_min_curvature_m: f64,

    /// Maximum curvature possible under an ackerman command.
    ///
    /// Units: 1/meters
    pub ackerman_max_curvature_m: f64

}