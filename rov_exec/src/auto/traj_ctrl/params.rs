//! Trajectory control parameters

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use serde::Deserialize;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Parameters for trajectory control
#[derive(Deserialize, Debug, Clone)]
pub struct Params {
    
    /// Lateral controller proportional gain
    pub lat_k_p: f64,

    /// Lateral controller integral gain
    pub lat_k_i: f64,

    /// Lateral controller derivative gain
    pub lat_k_d: f64,

    /// Heading controller proportional gain
    pub head_k_p: f64,

    /// Heading controller integral gain
    pub head_k_i: f64,

    /// Heading controller derivative gain
    pub head_k_d: f64,

    /// Curvature demand minimum limit
    pub min_curv_dem_m: f64,

    /// Curvature demand minimum limit
    pub max_curv_dem_m: f64,

    /// Crab demand minimum limit
    pub min_crab_dem_rad: f64,

    /// Crab demand maximum limit
    pub max_crab_dem_rad: f64,

    /// Curvature to speed map coefficients
    /// 
    /// The order of these coefficients is highest power first, i.e if there
    /// are 3 coefficients it's a 2nd order polynomial with c[0]*x^2 + c[1]*x
    /// + c[2].
    pub curv_speed_map_coeffs: Vec<f64>,

    /// Minimum speed demand
    pub min_speed_dem_ms: f64,

    /// Maximum speed demand
    pub max_speed_dem_ms: f64,

    /// The limit on lateral error. Above this limit the path sequence will
    /// be aborted.
    pub lat_error_limit_m: f64,

    /// The limit on heading error. Above this limit the path sequence will
    /// be aborted.
    pub head_error_limit_rad: f64,

    /// The rate at which to turn the rover during a heading adjustment 
    /// manouvre.
    pub head_adjust_rate_rads: f64,

    /// The threshold under which a heading adjustment will be considered 
    /// complete.
    pub head_adjust_threshold_rad: f64
}