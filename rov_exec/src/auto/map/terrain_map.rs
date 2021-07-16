//! # Terrain Map

// ------------------------------------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------------------------------------

use std::ops::{Deref, DerefMut, Range};

use cell_map::{CellMap, CellMapParams, Error as CellMapError, Layer};
use nalgebra::Point2;
use noise::{NoiseFn, Perlin};
use serde::{Deserialize, Serialize};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Terrain Map
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TerrainMap(pub(super) CellMap<TerrainMapLayer, Option<f64>>);

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Possible layers in a [`TerrainMap`]
#[derive(PartialEq, Eq, Clone, Copy, Hash, Debug, Serialize, Deserialize, Layer)]
pub enum TerrainMapLayer {
    Height,
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl TerrainMap {
    /// Create a new empty terrain map with the given grid information.
    pub fn new(params: CellMapParams) -> Self {
        Self(CellMap::new(params))
    }

    /// Merge `other` into `self`, modifying `self`.
    pub fn merge(&mut self, other: &Self) {}

    /// Generate a random terrain map using a Perlin noise system
    pub fn generate_random(
        params: CellMapParams,
        perlin_scale: Point2<f64>,
        perlin_offset: Point2<f64>,
    ) -> Result<Self, CellMapError> {
        let mut map = CellMap::<TerrainMapLayer, Option<f64>>::new(params);

        let perlin = Perlin::new();

        for ((_, pos), h) in map.iter_mut().positioned() {
            *h = Some(perlin.get([
                pos.x * perlin_scale.x + perlin_offset.x,
                pos.y * perlin_scale.y + perlin_offset.y,
            ]))
        }

        Ok(Self(map))
    }

    /// Returns the range of height in the terrain, or `None` if the map is empty
    pub fn range(&self) -> Option<Range<f64>> {
        let mut min = None;
        let mut max = None;

        for &height in self.iter() {
            match (height, min) {
                (Some(h), None) => min = Some(h),
                (Some(h), Some(m)) => {
                    if h < m {
                        min = Some(h)
                    }
                }
                (None, _) => (),
            }
            match (height, max) {
                (Some(h), None) => max = Some(h),
                (Some(h), Some(m)) => {
                    if h > m {
                        max = Some(h)
                    }
                }
                (None, _) => (),
            }
        }

        match (min, max) {
            (Some(min), Some(max)) => Some(min..max),
            _ => None,
        }
    }

    /// Dummy test function that simulates the rover's field of view of the terrain.
    pub fn clip_to_rov_view(
        &mut self,
        position: Point2<f64>,
        heading: f64,
        view_range: Range<f64>,
        field_of_view: f64,
    ) {
        let mut in_fov;

        for ((_, pos), height) in self.iter_mut().positioned() {
            // Leave any unknown heights unchanged
            if height.is_none() {
                continue;
            }

            // Reset in_fov
            in_fov = true;

            // Calculate if the position is in the field of view
            let diff = pos - position;

            // Polar coordinates between the rover and the pos.
            let radius = (diff.x.powi(2) + diff.y.powi(2)).sqrt();
            let theta = diff.y.atan2(diff.x) + heading;

            // Check radius is within the view range
            if radius < view_range.start || radius > view_range.end {
                in_fov = false;
            }

            // Check the theta is within the field of view
            if theta.abs() > field_of_view / 2.0 {
                in_fov = false;
            }

            // If the cell is in the FOV we keep it, if it isn't we set it to None
            if !in_fov {
                *height = None;
            }
        }
    }
}

impl Deref for TerrainMap {
    type Target = CellMap<TerrainMapLayer, Option<f64>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for TerrainMap {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
