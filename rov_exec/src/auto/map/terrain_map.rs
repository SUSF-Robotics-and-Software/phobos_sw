//! # Terrain Map

// ------------------------------------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------------------------------------

use std::ops::Deref;

use super::{GridMap, GridMapError, Point2};
use noise::{Perlin, NoiseFn};
use serde::{Serialize, Deserialize};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Terrain Map
pub struct TerrainMap(pub(super) GridMap<Option<f64>, TerrainMapLayer>);


// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Possible layers in a [`TerrainMap`]
#[derive(PartialEq, Eq, Clone, Copy, Hash, Debug, Serialize, Deserialize)]
pub enum TerrainMapLayer {
    Height
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl TerrainMap {
    /// Create a new empty terrain map with the given grid information.
    pub fn new(
        cell_size: Point2<f64>, 
        num_cells: Point2<usize>, 
        centre_position: Point2<f64>
    ) -> Result<Self, GridMapError> {
        let map = GridMap::new(
            cell_size,
            num_cells,
            centre_position,
            &[TerrainMapLayer::Height],
            None
        )?;

        Ok(Self(map))
    }

    /// Generate a random terrain map using a Perlin noise system
    pub fn generate_random(
        cell_size: Point2<f64>, 
        num_cells: Point2<usize>, 
        centre_position: Point2<f64>,
        perlin_scale: Point2<f64>,
        perlin_offset: Point2<f64>
    ) -> Result<Self, GridMapError> {
        let mut map = GridMap::new(
            cell_size,
            num_cells,
            centre_position,
            &[TerrainMapLayer::Height],
            None
        )?;

        let perlin = Perlin::new();

        map = map
            .map(
                TerrainMapLayer::Height, 
                |_, pos, _| {
                    Some(perlin.get([
                        pos.x()*perlin_scale.x() + perlin_offset.x(), 
                        pos.y()*perlin_scale.y() + perlin_offset.y()
                    ]))
                }
            )?;

        Ok(Self(map))
    }
}

impl Deref for TerrainMap {
    type Target = GridMap<Option<f64>, TerrainMapLayer>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
