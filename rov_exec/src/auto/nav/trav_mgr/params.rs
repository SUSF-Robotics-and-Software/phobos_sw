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
    // pub escape_boundary: EscapeBoundaryParams,

    /// The minimum distance between the local target and an unpopulated cell.
    pub local_target_exclusion_distance_m: f64,

    /// Maximum distance between the rover's current pose and the local target.
    pub local_target_max_distance_m: f64,

    /// The threshold distance used to determine if the traverse can end.
    pub traverse_end_threshold_m: f64,
}
