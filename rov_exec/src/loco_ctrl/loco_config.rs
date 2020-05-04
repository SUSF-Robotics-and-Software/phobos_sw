//! Locomotion Configuration structure

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use serde::Serialize;

// Internal
use super::{NUM_DRV_AXES, NUM_STR_AXES};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

// Stores a locomotion configuration - the positions of all steer axes and 
/// speed of all drive axes.
#[derive(Clone, Copy, Serialize)]
pub struct LocoConfig {
    
    /// All possible steer axes
    pub str_axes: [AxisData; NUM_STR_AXES],

    /// All possible drive axes
    pub drv_axes: [AxisData; NUM_DRV_AXES]
}

/// Data about a particular axis
#[derive(Clone, Copy, Default, Serialize)]
pub struct AxisData {
    /// Absolute position of the axis in radians relative to the axis's 
    /// predefined "zero" position. For STR this is straight ahead. For DRV it
    /// is the position of the axis when the system was initialised.
    /// 
    /// Units: radians
    pub abs_pos_rad: f64,

    /// Rate of the axis.
    /// 
    /// Units: radians/second for Absolute, N/A for normalised
    pub rate: AxisRate
}

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// An enumeration which allows axis rates to be specifed as either Normalised
/// rates (-1 to +1) or absolute rates.
#[derive(Clone, Copy, Serialize, Debug)]
pub enum AxisRate {
    Normalised(f64),
    Absolute(f64)
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Default for AxisRate {
    fn default() -> Self {
        AxisRate::Normalised(0.0)
    }
}

impl AxisRate {
    pub fn invert(self) -> AxisRate {
        match self {
            AxisRate::Absolute(r) => AxisRate::Absolute(-r),
            AxisRate::Normalised(n) => AxisRate::Normalised(-n)
        }
    }
}