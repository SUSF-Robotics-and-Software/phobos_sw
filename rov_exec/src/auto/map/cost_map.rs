//! # Cost Map
//!
//! If a ground path is provided it will have two boundaries placed around it. Between the path and
//! the first boundary `cost_onset_semi_width_m` no additional cost is added to the cost map.
//! The added cost then increases from 0 to `max_added_cost` at `max_cost_semi_width_m`.
//! ```text
//! GROUND PATH
//!    ┌┐
//!    ││    │                             │
//! ──►├│    │     max_cost_semi_width     │◄──
//!    ││    │                             │
//! ──►├│    │◄── cost_onset_semi_width_m  │
//!    ││    │                             │
//! ```

// ------------------------------------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------------------------------------

use std::ops::{Deref, DerefMut};

use crate::auto::path::Path;

use super::{TerrainMap, TerrainMapLayer};
use cell_map::{CellMap, CellMapParams, Error as CellMapError, Layer};
use log::debug;
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};
use util::{convert::Convert, quadtree::Quad};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Cost Map
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CostMap {
    pub(super) map: CellMap<CostMapLayer, CostMapData>,

    pub(super) cost_map_params: CostMapParams,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CostMapParams {
    /// Maximum safe gradient value, in dy/dx.
    pub max_safe_gradient: f64,

    /// The factor applied to the gradient cost.
    pub gradient_cost_factor: f64,

    /// The distance (semi_width) from a potential ground planned path at which a cost will begin
    /// to be added to the cost map.
    pub gnd_path_cost_onset_semi_width_m: f64,

    /// The distance (semi_width) from a potential ground planned path at which the maximum added
    /// cost will be applied.
    pub max_gnd_path_cost_semi_width_m: f64,

    /// The maximum cost added to the cost map by a potential ground planned path.
    pub max_gnd_path_added_cost: f64,
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Possible layers in a [`CostMap`]
#[derive(PartialEq, Eq, Clone, Copy, Hash, Debug, Serialize, Deserialize, Layer)]
pub enum CostMapLayer {
    Total,
    Gradient,
    GroundPlannedPath,
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
    Cost(f64),
}

/// Errors that can arise from processing costs maps.
// TODO: make Clone when CellMapError is Clone.
#[derive(Debug, thiserror::Error)]
pub enum CostMapError {
    #[error("Error in underlying CellMap: {0}")]
    CellMapError(CellMapError),

    #[error(
        "Cannot process {0}, since this CostMap is of shape {1}, but the target is of shape {2}"
    )]
    ShapeMismatch(String, Vector2<usize>, Vector2<usize>),
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl Default for CostMapData {
    fn default() -> Self {
        Self::None
    }
}

impl CostMap {
    /// Create a new empty cost map with the given grid information.
    pub fn new(cell_map_params: CellMapParams, cost_map_params: CostMapParams) -> Self {
        let map = CellMap::new(cell_map_params);

        Self {
            map,
            cost_map_params,
        }
    }

    /// Calculate the cost map from the given terrain map
    pub fn calculate(
        cost_map_params: CostMapParams,
        terrain_map: &TerrainMap,
    ) -> Result<Self, CostMapError> {
        let mut map = Self::new(terrain_map.params(), cost_map_params);

        map.calculate_gradient(terrain_map)?;

        // cost_map
        //     .0
        //     .set_layer_value(CostMapLayer::GroundPlannedPath, CostMapData::Cost(0.0))?;

        map.calculate_total()?;

        Ok(map)
    }

    /// Applies the provided path to the cost map.
    pub fn apply_ground_planned_path(&mut self, path: &Path) -> Result<(), CostMapError> {
        // This is going to use a simple linear search over all path points for every cell in the
        // map, so no need to calc a quadtree at the start

        // Precompute the value for max cost, as we have to check if it's greater than one, in
        // which case it's considered unsafe
        let max_cost_val: CostMapData;
        if self.cost_map_params.max_gnd_path_added_cost >= 1.0 {
            max_cost_val = CostMapData::Unsafe;
        } else {
            max_cost_val = CostMapData::Cost(self.cost_map_params.max_gnd_path_added_cost);
        }

        for ((_, pos), cost) in self
            .map
            .iter_mut()
            .layer(CostMapLayer::GroundPlannedPath)
            .positioned()
        {
            // Start by checking if there are any path points in a quad
            // around the current point of half_width equal to the max semi_width
            let query_quad = Quad::new(
                pos.convert(),
                self.cost_map_params.max_gnd_path_cost_semi_width_m,
            );

            // Find all points within the max distance
            let mut points_in_quad = Vec::new();
            for point in path.points_m.iter() {
                if query_quad.contains(point) {
                    points_in_quad.push(*point);
                }
            }

            // If there are no points in that search quad the cost for this cell is the max
            // cost, since we're too far away from the path
            if points_in_quad.is_empty() {
                *cost = max_cost_val;
                continue;
            }

            // Otherwise we might be in range of the path, so find the closest point.
            let mut closest_point = (points_in_quad[0] - pos.convert()).norm();
            for &point in points_in_quad.iter() {
                let dist = (point - pos.convert()).norm();
                if dist < closest_point {
                    closest_point = dist
                }
            }

            // Assign cost based on the distance to the closest point. If it's less than the
            // onset distance the cost is zero, if it's greater than the max distance it's the
            // max cost. If it's between it's a linear map between the two.
            if closest_point < self.cost_map_params.gnd_path_cost_onset_semi_width_m {
                *cost = CostMapData::Cost(0.0);
            } else if closest_point > self.cost_map_params.max_gnd_path_cost_semi_width_m {
                *cost = max_cost_val;
            } else {
                let cost_val = util::maths::lin_map(
                    (
                        self.cost_map_params.gnd_path_cost_onset_semi_width_m,
                        self.cost_map_params.max_gnd_path_cost_semi_width_m,
                    ),
                    (0.0, self.cost_map_params.max_gnd_path_added_cost),
                    closest_point,
                );

                // Clamp cost to the unsafe value of 1.0
                if cost_val >= 1.0 {
                    *cost = CostMapData::Unsafe;
                } else {
                    *cost = CostMapData::Cost(cost_val);
                }
            }
        }

        self.calculate_total()
    }

    /// Calculate the gradient cost of the given terrain map.
    ///
    /// This function should only be used on a `terrain_map` which has the same parameters as `self`.
    fn calculate_gradient(&mut self, terrain_map: &TerrainMap) -> Result<(), CostMapError> {
        // Check self and terrain map are the same size
        if self.map.num_cells() != terrain_map.num_cells() {
            return Err(CostMapError::ShapeMismatch(
                "TerrainMap".into(),
                self.map.num_cells(),
                terrain_map.num_cells(),
            ));
        }

        // Zip together windows in the terrain map and cost map
        for (height, mut gradient) in terrain_map
            .window_iter(Vector2::new(1, 1))?
            .layer(TerrainMapLayer::Height)
            .zip(
                self.map
                    .window_iter_mut(Vector2::new(1, 1))?
                    .layer(CostMapLayer::Gradient),
            )
        {
            // Calculate dh/dx and dh/dy by comparing neighbours in each direction
            let dh_dx = match (height[[2, 1]], height[[0, 1]]) {
                (Some(a), Some(b)) => Some((a - b) / (2.0 * terrain_map.cell_size().x)),
                _ => None,
            };
            let dh_dy = match (height[[1, 2]], height[[1, 0]]) {
                (Some(a), Some(b)) => Some((a - b) / (2.0 * terrain_map.cell_size().y)),
                _ => None,
            };

            // Get gradient as the magnitude of the total vector, or as the magnitude
            // of the single vector if there's one missing, or none otherwise
            gradient[[1, 1]] = match (dh_dx, dh_dy) {
                (Some(x), Some(y)) => {
                    // Check if cost is above max
                    let cost = (x * x + y * y).sqrt() * self.cost_map_params.gradient_cost_factor;
                    if cost > self.cost_map_params.max_safe_gradient {
                        CostMapData::Unsafe
                    } else {
                        CostMapData::Cost(cost)
                    }
                }
                (Some(x), None) => {
                    // Check if cost is above max
                    let cost = x.abs() * self.cost_map_params.gradient_cost_factor;
                    if cost > self.cost_map_params.max_safe_gradient {
                        CostMapData::Unsafe
                    } else {
                        CostMapData::Cost(cost)
                    }
                }
                (None, Some(y)) => {
                    // Check if cost is above max
                    let cost = y.abs() * self.cost_map_params.gradient_cost_factor;
                    if cost > self.cost_map_params.max_safe_gradient {
                        CostMapData::Unsafe
                    } else {
                        CostMapData::Cost(cost)
                    }
                }
                _ => CostMapData::None,
            }
        }

        Ok(())
    }

    /// Computes the total cost of all layers in the map, and stores it in the `CostMapLayer::Total`
    /// layer.
    fn calculate_total(&mut self) -> Result<(), CostMapError> {
        let map_ptr: *const CellMap<CostMapLayer, CostMapData> = &self.map;

        for ((_, idx), cost) in self.map.iter_mut().layer(CostMapLayer::Total).indexed() {
            // Unwrap is safe here since the iterator is guaranteed to not produce any `idx` not in
            // the map.
            //
            // Use of unsafe: we're not mutating either the gradient or ground planned path layers,
            // so we can safely access these while holding a mutable reference to the cost layer.
            unsafe {
                let map = &*map_ptr;
                (*cost).add(&map.get(CostMapLayer::Gradient, idx).unwrap());
                (*cost).add(&map.get(CostMapLayer::GroundPlannedPath, idx).unwrap());
            }
        }

        Ok(())
    }

    /// Compute the cost of the given path through the map.
    ///
    /// If at any point the path crosses an unsafe cell `CostMapData::Unsafe` will be returned. If
    /// it ever crosses an unpopulated cell `CostMapData::None` will be returned.
    ///
    /// Costs calculated for a path are not bounded to 1.0, as they are the sum of the costs of all
    /// cells.
    pub fn get_path_cost(&self, path: &Path) -> Result<CostMapData, CellMapError> {
        // Accumulate cost for the path
        let mut cost = CostMapData::Cost(0.0);

        // Add all cells by traversing each segment
        for target in 1..(path.points_m.len() - 1) {
            for cell_cost in self.map.line_iter(
                path.points_m[target - 1].into(),
                path.points_m[target].into(),
            )? {
                // Sum the cost from this cell
                cost.add_without_max(cell_cost);
            }
        }

        Ok(cost)
    }
}

impl Deref for CostMap {
    type Target = CellMap<CostMapLayer, CostMapData>;

    fn deref(&self) -> &Self::Target {
        &self.map
    }
}

impl DerefMut for CostMap {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.map
    }
}

impl From<CellMapError> for CostMapError {
    fn from(e: CellMapError) -> Self {
        Self::CellMapError(e)
    }
}

impl CostMapData {
    /// Adds other to self, mutating self.
    ///
    /// Follows these rules:
    ///  - If either self or other is `None`, self becomes `None`.
    ///  - If either self or other is `Unsafe`, self becomes `Unsafe`.
    ///  - If both self and other have a `Cost`, add the costs together. If the cost is greater
    ///    than 1, self beocmes `Unsafe`.
    pub fn add(&mut self, other: &CostMapData) {
        use CostMapData::*;

        *self = match (self.clone(), other) {
            (None, _) => None,
            (_, None) => None,
            (Unsafe, _) => Unsafe,
            (_, Unsafe) => Unsafe,
            (Cost(s), Cost(o)) => {
                let sum = s + o;
                if sum >= 1.0 {
                    Unsafe
                } else {
                    Cost(sum)
                }
            }
        }
    }

    /// Adds other to self, mutating self.
    ///
    /// Follows these rules:
    ///  - If either self or other is `None`, self becomes `None`.
    ///  - If either self or other is `Unsafe`, self becomes `Unsafe`.
    ///  - If both self and other have a `Cost`, add the costs together.
    pub fn add_without_max(&mut self, other: &CostMapData) {
        use CostMapData::*;

        *self = match (self.clone(), other) {
            (None, _) => None,
            (_, None) => None,
            (Unsafe, _) => Unsafe,
            (_, Unsafe) => Unsafe,
            (Cost(s), Cost(o)) => Cost(s + o),
        }
    }
}
