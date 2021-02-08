//! # Terrain Map

// ------------------------------------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------------------------------------

use std::ops::{Deref, Range};

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

    /// Returns the range of heighgt in the terrain, or an error if the map is empty
    pub fn range(&self) -> Result<Range<f64>, GridMapError> {
        let mut min = None;
        let mut max = None;

        for x in 0..self.num_cells.x() {
            for y in 0..self.num_cells.y() {
                let height = self.get(
                    TerrainMapLayer::Height, 
                    &Point2::new(x, y)
                )?;

                match (height, min) {
                    (Some(h), None) => min = Some(h),
                    (Some(h), Some(m)) => {
                        if h < m {
                            min = Some(h)
                        }
                    },
                    (None, _) => ()
                }
                match (height, max) {
                    (Some(h), None) => max = Some(h),
                    (Some(h), Some(m)) => {
                        if h > m {
                            max = Some(h)
                        }
                    },
                    (None, _) => ()
                }
            }
        }

        match (min, max) {
            (Some(min), Some(max)) => Ok(min..max),
            _ => Err(GridMapError::Empty)
        }
    }

    /// Dummy test function that simulates the rover's field of view of the terrain.
    pub fn clip_to_rov_view(
        &mut self, 
        position: Point2<f64>, 
        heading: f64, 
        view_range: Range<f64>,
        field_of_view: f64
    ) -> Result<(), GridMapError> {
        self.0 = self.0.map(
            TerrainMapLayer::Height,
            |_, pos, height| {
                if height.is_none() {
                    return None;
                }

                // Calculate polar form of the difference between the rover's position and this
                // cell's position.
                let diff = Point2::new(
                    pos.x() - position.x(), 
                    pos.y() - position.y()
                );

                let radius = (diff.x().powi(2) + diff.y().powi(2)).sqrt();
                let theta = diff.y().atan2(diff.x()) + heading;

                // Check radius is whithin the view range
                if radius < view_range.start || radius > view_range.end {
                    return None;
                }

                // Check the theta is within the field of view
                if theta.abs() > field_of_view/2.0 {
                    return None;
                }

                // Finally if all tests have passed return the height
                return height
            }
        )?;

        Ok(())
    }
}

impl Deref for TerrainMap {
    type Target = GridMap<Option<f64>, TerrainMapLayer>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
