//! # Map
//!
//! This module implements the [`TerrainMap`] and [`NavMap`] types, both of which are based upon the
//! generic [`GridMap`] type. [`GridMap`] is inspired by
//! [grid_map](https://github.com/ANYbotics/grid_map) by ANYbotics. The
//! [paper](https://www.researchgate.net/publication/284415855_A_Universal_Grid_Map_Library_Implementation_and_Use_Case_for_Rough_Terrain_Navigation)
//! gives a good intro the concepts behind `grid_map`.

// ------------------------------------------------------------------------------------------------
// MODS
// ------------------------------------------------------------------------------------------------

// /// Main [`GridMap`] base implementation
// mod grid_map;

/// Implements the [`TerrainMap`] type
mod terrain_map;

/// Implements the [`CostMap`] type
mod cost_map;

// ------------------------------------------------------------------------------------------------
// EXPORTS
// ------------------------------------------------------------------------------------------------

use std::{fs::File, path::Path};

use cell_map::{CellMap, Layer};
pub use cost_map::{CostMap, CostMapData, CostMapError, CostMapLayer, CostMapParams};
use nalgebra::{Matrix1x2, Matrix2, Matrix2x1, Point2, Vector2};
use serde::Serialize;
pub use terrain_map::{TerrainMap, TerrainMapLayer};

// ------------------------------------------------------------------------------------------------
// TRAITS
// ------------------------------------------------------------------------------------------------

pub trait CellMapExt: Serialize {
    /// Saves self
    ///
    /// # Safety
    /// Panics if the operation fails
    fn save<P: AsRef<Path>>(&self, path: P) {
        serde_json::to_writer_pretty(&File::create(path).expect("Failed to create file"), &self)
            .expect("Failed to serialize as JSON");
    }
}

impl CellMapExt for CostMap {}
impl CellMapExt for TerrainMap {}

// ------------------------------------------------------------------------------------------------
// FUNCTIONS
// ------------------------------------------------------------------------------------------------

fn bilinterp<L: Layer, T, F: Fn(&T) -> Option<f64>>(
    map: &CellMap<L, T>,
    layer: L,
    position: Point2<f64>,
    conv_func: F,
) -> Option<f64> {
    // Get index of the position in self, if it doesn't exist we'll skip
    if let Some(idx) = map.index(position) {
        // Get the indices of the surrounding values in self
        let (idx_sw, idx_nw, idx_ne, idx_se) = (
            idx,
            idx + Vector2::new(0, 1),
            idx + Vector2::new(1, 1),
            idx + Vector2::new(1, 0),
        );

        // Get the values surrounding this position
        let (sw, nw, ne, se) = (
            map.get(layer.clone(), idx_sw),
            map.get(layer.clone(), idx_nw),
            map.get(layer.clone(), idx_ne),
            map.get(layer, idx_se),
        );

        // Check if we know all locations, if we don't skip
        if sw.and(nw).and(ne).and(se).is_none() {
            return None;
        }

        // Unwrap values for easier computation
        let sw = conv_func(sw.unwrap());
        let nw = conv_func(nw.unwrap());
        let ne = conv_func(ne.unwrap());
        let se = conv_func(se.unwrap());

        // Check that the values are actually populated
        if sw.and(nw).and(ne).and(se).is_none() {
            return None;
        }

        // Unwrap values for easier computation
        let sw = sw.unwrap();
        let nw = nw.unwrap();
        let ne = ne.unwrap();
        let se = se.unwrap();

        // Get the positions of these values, unwrap is ok since we checked for Noneness
        // earlier. We actually only need the position of the sw and ne points, since nw
        // and se will have combinations of the x and y positions of these first two.
        let (pos_sw, pos_ne) = (map.position(idx_sw).unwrap(), map.position(idx_ne).unwrap());

        // Interpolate the point value
        Some(
            (Matrix1x2::new(pos_ne.x - position.x, position.x - pos_sw.x)
                * Matrix2::new(sw, nw, ne, se)
                * Matrix2x1::new(pos_ne.y - position.y, position.y - pos_sw.y)
                / ((pos_ne.x - pos_sw.x) * (pos_ne.y - pos_sw.y)))
                .x,
        )
    } else {
        None
    }
}
