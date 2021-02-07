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

/// Main [`GridMap`] base implementation
mod grid_map;

/// Implements the [`TerrainMap`] type
mod terrain_map;

/// Implements the [`CostMap`] type
mod cost_map;

// ------------------------------------------------------------------------------------------------
// EXPORTS
// ------------------------------------------------------------------------------------------------

pub use grid_map::{GridMap, GridMapError, Point2};
pub use terrain_map::{TerrainMap, TerrainMapLayer};
pub use cost_map::{CostMap, CostMapLayer};