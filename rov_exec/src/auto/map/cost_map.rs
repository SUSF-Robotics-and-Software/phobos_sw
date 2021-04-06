//! # Cost Map

// ------------------------------------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------------------------------------

use std::ops::Deref;

use super::{GridMap, GridMapError, Point2, TerrainMap, TerrainMapLayer, grid_map::SerializableGridMap};
use ndarray::{Array2, Zip, s};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Cost Map
#[derive(Clone, Debug)]
pub struct CostMap(pub(super) GridMap<CostMapData, CostMapLayer>);

#[derive(Debug, Clone, Deserialize)]
pub struct CostMapParams {
    /// Maximum safe gradient value, in dy/dx.
    pub max_safe_gradient: f64
}

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
    pub fn calculate(
        params: CostMapParams, 
        terrain_map: &TerrainMap
    ) -> Result<Self, GridMapError> {
        let mut cost_map = Self::new(
            terrain_map.0.cell_size.clone(),
            terrain_map.0.num_cells.clone(),
            terrain_map.0.centre_position.clone()
        )?;

        cost_map.0.set_layer(
            CostMapLayer::Gradient,
            Self::calculate_gradient(params, terrain_map)?
        )?;
        
        // Calculate the total layer by summing all other layers
        cost_map.0.set_layer(
            CostMapLayer::Total,
            cost_map.0.get_layer_owned(CostMapLayer::Gradient)?
        )?;

        Ok(cost_map)
    }

    /// Calculate the gradient cost of the given terrain map
    fn calculate_gradient(
        params: CostMapParams, 
        terrain_map: &TerrainMap
    ) -> Result<Array2<CostMapData>, GridMapError> {
        let mut gradient = Array2::from_elem(
            (terrain_map.num_cells.x(), terrain_map.num_cells.y()), 
            CostMapData::Unsafe
        );

        // First a simple finite differencing algorithm.
        Zip::from(terrain_map
                .get_layer(TerrainMapLayer::Height)?
                .windows((3, 3)) 
            )
            .and(gradient.slice_mut(
                s![1..terrain_map.num_cells.x() - 1, 1..terrain_map.num_cells.y() - 1]
            ))
            .for_each(|window, grad| {
                // Calculate dh/dx and dh/dy by comparing neighbours in each direction
                let dh_dx = match (window[[2, 1]], window[[0, 1]]) {
                    (Some(a), Some(b)) => Some((a - b)/(2.0 * terrain_map.cell_size.x())),
                    _ => None,
                };
                let dh_dy = match (window[[1, 2]], window[[1, 0]]) {
                    (Some(a), Some(b)) => Some((a - b)/(2.0 * terrain_map.cell_size.y())),
                    _ => None,
                };

                // Get gradient as the magnitude of the total vector, or as the magnitude
                // of the single vector if there's one missing, or none otherwise
                *grad = match (dh_dx, dh_dy) {
                    (Some(x), Some(y)) => {
                        // Check if cost is above max
                        let cost = (x*x + y*y).sqrt();
                        if cost > params.max_safe_gradient {
                            CostMapData::Unsafe
                        }
                        else {
                            CostMapData::Cost(cost)
                        }
                    },
                    (Some(x), None) => {
                        // Check if cost is above max
                        if x > params.max_safe_gradient {
                            CostMapData::Unsafe
                        }
                        else {
                            CostMapData::Cost(x)
                        }
                    },
                    (None, Some(y)) => {
                        // Check if cost is above max
                        if y > params.max_safe_gradient {
                            CostMapData::Unsafe
                        }
                        else {
                            CostMapData::Cost(y)
                        }
                    },
                    _ => CostMapData::None
                }
            });

        Ok(gradient)
    }
}

impl Deref for CostMap {
    type Target = GridMap<CostMapData, CostMapLayer>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Serialize for CostMap {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer 
    {
        // Convert to a SerializableCostMap
        let ser = SerializableGridMap::from_grid_map(&self);

        ser.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for CostMap {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de> 
    {
        // Deserialize into a SerializableGridMap
        let ser: SerializableGridMap<CostMapData, CostMapLayer> 
            = SerializableGridMap::deserialize(deserializer)?;

        // Convert to cost map
        Ok(CostMap(ser.to_grid_map().expect("Couldn't deserialize grid map to CostMap")))
    }
}