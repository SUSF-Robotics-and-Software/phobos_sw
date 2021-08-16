//! Arm Configuration structure

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use serde::Serialize;

// Internal
use super::NUM_ROT_AXES;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

// Stores a arm configuration - the positions of all rotational axes and
/// speed of all rotational axes.
#[derive(Clone, Copy, Serialize)]
pub struct ArmConfig {

    /// All possible rotational axis
    pub rot_axes: [AxisData; NUM_ROT_AXES],
}

/// Data about a particular axis
#[derive(Clone, Copy, Default, Serialize)]
pub struct AxisData {
    /// Absolute position of the axis in radians relative to the axis's
    /// predefined "zero" position. For ROT, it is the position of the
    /// axis when the system was initialised.
    ///
    /// Units: radians
    pub abs_pos_rad: f64,

    /// Rate of the axis.
    ///
    /// Units: radians/second
    pub rate_rads: f64,
}
