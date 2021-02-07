//! # Cost Map

// ------------------------------------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------------------------------------

use std::ops::Deref;

use super::{GridMap, GridMapError, Point2, TerrainMap, TerrainMapLayer};
use ndarray::Array2;
use serde::{Serialize, Deserialize};
use util::maths::lin_map;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Cost Map
pub struct CostMap(pub(super) GridMap<CostMapData, CostMapLayer>);

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Possible layers in a [`CostMap`]
#[derive(PartialEq, Eq, Clone, Copy, Hash, Debug, Serialize, Deserialize)]
pub enum CostMapLayer {
    Total,
    Gradient
}

/// Possible values of the cost map
#[derive(PartialEq, Clone, Copy, Debug, Serialize, Deserialize)]
pub enum CostMapData {
    /// This cell is empty (has not been analysed - do not plan path)
    None,
    
    /// This cell cannot be travesed as it is unsafe
    Unsafe,

    /// General cost associated with an analysed safe cell. Values are between 0 and 1 (inclusive).
    /// 0 represents the lowest cost, and 1 the highest. Any values above 1 would be converted to
    /// Unsafe. Any values below zero are not possible, though could be thought of as being
    /// un-surveyed or None.
    Cost(f64)
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------


impl CostMap {
    /// Create a new empty cost map with the given grid information.
    pub fn new(
        cell_size: Point2<f64>, 
        num_cells: Point2<usize>, 
        centre_position: Point2<f64>
    ) -> Result<Self, GridMapError> {
        let map = GridMap::new(
            cell_size,
            num_cells,
            centre_position,
            &[CostMapLayer::Total, CostMapLayer::Gradient],
            CostMapData::None
        )?;

        Ok(Self(map))
    }

    /// Calculate the cost map from the given terrain map
    pub fn calculate(terrain_map: &TerrainMap) -> Result<Self, GridMapError> {
        let mut cost_map = Self::new(
            terrain_map.0.cell_size.clone(),
            terrain_map.0.num_cells.clone(),
            terrain_map.0.centre_position.clone()
        )?;

        cost_map.0.set_layer(
            CostMapLayer::Gradient,
            Self::calculate_gradient(terrain_map)?
        )?;
        
        // Calculate minimum and maximum terrain height
        let min_height = terrain_map.min(TerrainMapLayer::Height)?
            .ok_or(GridMapError::Empty)?;
        let max_height = terrain_map.max(TerrainMapLayer::Height)?
            .ok_or(GridMapError::Empty)?;

        cost_map.0 = terrain_map.map_into(
            TerrainMapLayer::Height,
            CostMapLayer::Total,
            &cost_map.0,
            |_, _, height| {
                match height {
                    // Map heights between 0 and 1
                    Some(h) => CostMapData::Cost(lin_map(
                        (min_height, max_height), 
                        (0.0, 1.0), 
                        h
                    )),
                    None => CostMapData::None
                }
            }
        )?;

        Ok(cost_map)
    }

    /// Calculate the gradient cost of the given terrain map
    fn calculate_gradient(terrain_map: &TerrainMap) -> Result<Array2<CostMapData>, GridMapError> {
        Ok(Array2::from_elem(
            (terrain_map.num_cells.x(), terrain_map.num_cells.y()), 
            CostMapData::Unsafe
        ))
    }
}

impl Deref for CostMap {
    type Target = GridMap<CostMapData, CostMapLayer>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl PartialOrd for CostMapData {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        match self {
            CostMapData::None => None,
            CostMapData::Unsafe => None,
            CostMapData::Cost(s) => match other {
                CostMapData::None => None,
                CostMapData::Unsafe => None,
                CostMapData::Cost(o) => s.partial_cmp(o)
            }
        }
    }
}