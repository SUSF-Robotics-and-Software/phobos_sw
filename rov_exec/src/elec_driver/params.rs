//! # Electronics driver parameters

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use serde::Deserialize;
use thiserror::Error;

// Internal
use crate::loco_ctrl;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

#[derive(Deserialize, Default)]
pub struct Params {
    
    /// Number of boards connected to the pi
    pub num_boards: u64,
    
    /// Number of channels connected to the motor driver boards
    pub num_channels: Vec<u64>,
    
    /// The addresses of each board. Must have `num_boards` elements.
    pub board_addresses: Vec<u64>,

    /// Map between loco_ctrl indexes ([LF, LM, LR, RF, RM, RR]) and the motors.
    /// The first element is the board index, second is motor index.
    pub drv_idx_map: [[u64; 2]; loco_ctrl::NUM_DRV_AXES],

    /// Map between loco_ctrl indexes ([LF, LM, LR, RF, RM, RR]) and the motors.
    /// The first element is the board index, second is motor index.
    pub str_idx_map: [[u64; 2]; loco_ctrl::NUM_STR_AXES],

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

    /// Minimum drive axis rate limits.
    pub drv_rate_min_sk: [f64; loco_ctrl::NUM_DRV_AXES],

    /// Maxmimum drive axis rate limits.
    pub drv_rate_max_sk: [f64; loco_ctrl::NUM_DRV_AXES],

    /// Minimum steer axis angle..
    pub str_ang_min_sk: [f64; loco_ctrl::NUM_STR_AXES],

    /// Maximum steer axis angle.
    pub str_ang_max_sk: [f64; loco_ctrl::NUM_STR_AXES],
}

#[derive(Debug, Error)]
pub enum ParamsError {
    #[error("Wrong number of boards ({0})")]
    NumBoardsError(String),

    #[error("Not all baords have a unique address")]
    NonUniqueBoardAddresses,

    #[error("Not all axes have unique indexes")]
    NonUniqueAxisIndex
}

impl Params {

    /// Determines if the parameters are valid.
    pub fn are_valid(&self) -> Result<(), ParamsError> {
        // Wrong number of addresses
        if self.num_boards as usize != self.board_addresses.len() {
            return Err(ParamsError::NumBoardsError(
                format!("There should be {} board addresses but found {}", 
                self.num_boards, self.board_addresses.len())));
        }

        // Non unique addresses
        let mut non_unique = false;

        for i in 0..self.board_addresses.len() {
            if self.board_addresses
                .iter()
                .filter(|&a| *a==self.board_addresses[i])
                .count() > 1 
            {
                non_unique = true;         
            }
        }

        if non_unique {
            return Err(ParamsError::NonUniqueBoardAddresses)
        }

        // Non unique axis maps
        for i in 0..self.drv_idx_map.len() {
            if self.drv_idx_map
                .iter()
                .filter(|&a| *a==self.drv_idx_map[i])
                .count() > 1 
            {
                non_unique = true;         
            }

            if self.str_idx_map
                .iter()
                .filter(|&a| *a==self.drv_idx_map[i])
                .count() > 1 
            {
                non_unique = true;         
            }
        }

        for i in 0..self.str_idx_map.len() {
            if self.drv_idx_map
                .iter()
                .filter(|&a| *a==self.str_idx_map[i])
                .count() > 1 
            {
                non_unique = true;         
            }

            if self.str_idx_map
                .iter()
                .filter(|&a| *a==self.str_idx_map[i])
                .count() > 1 
            {
                non_unique = true;         
            }
        }

        if non_unique {
            return Err(ParamsError::NonUniqueAxisIndex)
        }

        Ok(())
    }
}