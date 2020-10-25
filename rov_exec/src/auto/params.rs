//! # Autonomy Manager Parameters

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use serde::Deserialize;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Parameters for the autonomy manager
#[derive(Deserialize, Clone)]
pub struct Params {
    
    /// Parameters for the manouvre mode.
    pub mnvr_mode_params: MnvrModeParams
    
}

#[derive(Deserialize, Clone)]
pub struct MnvrModeParams {
    /// The threshold within which the linear distance requirement will be considered fulfilled.
    pub linear_distance_threshold_m: f64,

    /// The threshold within which the angular distance requirement will be considered fulfilled.
    pub angular_distance_threshold_rad: f64
}