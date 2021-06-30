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

// pub use grid_map::{GridMap, GridMapError, Point2};
pub use cost_map::{CostMap, CostMapLayer, CostMapParams};
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
