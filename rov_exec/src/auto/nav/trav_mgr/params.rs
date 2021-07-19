//! Parameters for the traverse manager

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use nalgebra::Vector2;
use serde::Deserialize;

use super::escape_boundary::EscapeBoundaryParams;

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Deserialize, Clone)]
pub struct TravMgrParams {
    /// Size of each cell in maps
    pub map_cell_size: Vector2<f64>,

    /// Escape boundary parameters
    pub escape_boundary: EscapeBoundaryParams,
}
