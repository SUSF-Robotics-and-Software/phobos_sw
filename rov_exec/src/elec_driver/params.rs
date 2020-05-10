//! # Electronics driver parameters

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use serde::Deserialize;
use crate::loco_ctrl;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

#[derive(Deserialize, Default)]
pub struct Params {
    /// Number of channels connected to the motor driver boards
    pub num_channels: u64,

    /// Map between loco_ctrl indexes ([LF, LM, LR, RF, RM, RR]) and the board
    /// motors for drive axes.
    pub drv_idx_map: [u64; loco_ctrl::NUM_DRV_AXES],

    /// Map between loco_ctrl indexes ([LF, LM, LR, RF, RM, RR]) and the board
    /// motors for steer axes.
    pub str_idx_map: [u64; loco_ctrl::NUM_STR_AXES],

    /// Polynomial coefficients to convert normalised drive axis rates to 
    /// ServoKit values.
    ///
    /// The order of these coefficients is highest power first, i.e if there
    /// are 3 coefficients it's a 2nd order polynomial with c[0]*x^2 + c[1]*x
    /// + c[2].
    pub drv_rate_norm_to_sk_coeffs: [Vec<f64>; loco_ctrl::NUM_DRV_AXES],

    /// Polynomial coefficients to convert steer axis angles to ServoKit values.
    ///
    /// The order of these coefficients is highest power first, i.e if there
    /// are 3 coefficients it's a 2nd order polynomial with c[0]*x^2 + c[1]*x
    /// + c[2].
    pub str_ang_rad_to_sk_coeffs: [Vec<f64>; loco_ctrl::NUM_DRV_AXES],
}