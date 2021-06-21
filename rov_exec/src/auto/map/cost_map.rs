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

use std::ops::Deref;

use crate::auto::path::Path;

use super::{
    grid_map::SerializableGridMap, GridMap, GridMapError, Point2, TerrainMap, TerrainMapLayer,
};
use nalgebra::Vector2;
use ndarray::{s, Array2, Zip};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use util::quadtree::{Quad, QuadTree};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Cost Map
#[derive(Clone, Debug)]
pub struct CostMap(pub(super) GridMap<CostMapData, CostMapLayer>);

#[derive(Debug, Clone, Deserialize)]
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
#[derive(PartialEq, Eq, Clone, Copy, Hash, Debug, Serialize, Deserialize)]
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

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl CostMap {
    /// Create a new empty cost map with the given grid information.
    pub fn new(
        cell_size: Point2<f64>,
        num_cells: Point2<usize>,
        centre_position: Point2<f64>,
    ) -> Result<Self, GridMapError> {
        let map = GridMap::new(
            cell_size,
            num_cells,
            centre_position,
            &[
                CostMapLayer::Total,
                CostMapLayer::Gradient,
                CostMapLayer::GroundPlannedPath,
            ],
            CostMapData::None,
        )?;

        Ok(Self(map))
    }

    /// Calculate the cost map from the given terrain map
    pub fn calculate(
        params: &CostMapParams,
        terrain_map: &TerrainMap,
    ) -> Result<Self, GridMapError> {
        let mut cost_map = Self::new(
            terrain_map.cell_size.clone(),
            terrain_map.num_cells.clone(),
            terrain_map.centre_position.clone(),
        )?;

        cost_map.0.set_layer(
            CostMapLayer::Gradient,
            Self::calculate_gradient(params, terrain_map)?,
        )?;

        cost_map
            .0
            .set_layer_value(CostMapLayer::GroundPlannedPath, CostMapData::Cost(0.0))?;

        cost_map.calculate_total()?;

        Ok(cost_map)
    }

    pub fn apply_ground_planned_path(
        &mut self,
        params: &CostMapParams,
        path: &Path,
    ) -> Result<(), GridMapError> {
        // According to bench_cost_map linear search faster than my quadtree implementation. I
        // belive it's because my impl of quadtree is slow, but for now we're just going to go
        // with the linear approach instead of quadtree.
        self.apply_ground_planned_path_linear(params, path)
    }

    pub fn apply_ground_planned_path_quadtree(
        &mut self,
        params: &CostMapParams,
        path: &Path,
    ) -> Result<(), GridMapError> {
        // We're going to be testing all points in the map against all points in the path. Doing
        // this with linear searches would take a very long time, so we're going to use a quadtree
        // of the path points instead.
        let mut path_quadtree = QuadTree::new(Quad::new(
            self.centre_position.clone().into(),
            (self.cell_size.x() * self.num_cells.x() as f64)
                .max(self.cell_size.y() * self.num_cells.y() as f64)
                / 2.0,
        ));

        // Insert all points in the path. If any point won't go into the quadtree then we will
        // return an error saying the point isn't in the map
        for &point in path.points_m.iter() {
            path_quadtree
                .insert(point)
                .map_err(|_| GridMapError::OutsideMap)?;
        }

        // Precompute the value for max cost, as we have to check if it's greater than one, in
        // which case it's considered unsafe
        let max_cost_val: CostMapData;
        if params.max_gnd_path_added_cost >= 1.0 {
            max_cost_val = CostMapData::Unsafe;
        } else {
            max_cost_val = CostMapData::Cost(params.max_gnd_path_added_cost);
        }

        // Iterate over all points in the map, and determine if the point is within the bounds.
        self.0
            .map_in_place(CostMapLayer::GroundPlannedPath, |_, pos, _| {
                let pos_vec2: Vector2<f64> = pos.into();

                // Start by querying the quadtree to see if there are any path points in a quad
                // around the current point of half_width equal to the max semi_width
                let query_quad = Quad::new(pos_vec2, params.max_gnd_path_cost_semi_width_m);
                let points_in_quad = path_quadtree.query_in_quad(&query_quad);

                // If there are no points in that search quad the cost for this cell is the max
                // cost, since we're too far away from the path
                if points_in_quad.is_empty() {
                    return max_cost_val;
                }

                // Otherwise we might be in range of the path, so find the closest point.
                let mut closest_point = (points_in_quad[0] - pos_vec2).norm();
                for point in points_in_quad.iter() {
                    let dist = (point - pos_vec2).norm();
                    if dist < closest_point {
                        closest_point = dist
                    }
                }

                // Assign cost based on the distance to the closest point. If it's less than the
                // onset distance the cost is zero, if it's greater than the max distance it's the
                // max cost. If it's between it's a linear map between the two.
                if closest_point < params.gnd_path_cost_onset_semi_width_m {
                    CostMapData::Cost(0.0)
                } else if closest_point > params.max_gnd_path_cost_semi_width_m {
                    max_cost_val
                } else {
                    let cost = util::maths::lin_map(
                        (
                            params.gnd_path_cost_onset_semi_width_m,
                            params.max_gnd_path_cost_semi_width_m,
                        ),
                        (0.0, params.max_gnd_path_added_cost),
                        closest_point,
                    );

                    // Clamp cost to the unsafe value of 1.0
                    if cost >= 1.0 {
                        CostMapData::Unsafe
                    } else {
                        CostMapData::Cost(cost)
                    }
                }
            })?;

        self.calculate_total()
    }

    pub fn apply_ground_planned_path_linear(
        &mut self,
        params: &CostMapParams,
        path: &Path,
    ) -> Result<(), GridMapError> {
        // This is going to use a simple linear search over all path points for every cell in the
        // map, so no need to calc a quadtree at the start

        // Precompute the value for max cost, as we have to check if it's greater than one, in
        // which case it's considered unsafe
        let max_cost_val: CostMapData;
        if params.max_gnd_path_added_cost >= 1.0 {
            max_cost_val = CostMapData::Unsafe;
        } else {
            max_cost_val = CostMapData::Cost(params.max_gnd_path_added_cost);
        }

        self.0
            .map_in_place(CostMapLayer::GroundPlannedPath, |_, pos, _| {
                let pos_vec2: Vector2<f64> = pos.into();

                // Start by querying the quadtree to see if there are any path points in a quad
                // around the current point of half_width equal to the max semi_width
                let query_quad = Quad::new(pos_vec2, params.max_gnd_path_cost_semi_width_m);

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
                    return max_cost_val;
                }

                // Otherwise we might be in range of the path, so find the closest point.
                let mut closest_point = (points_in_quad[0] - pos_vec2).norm();
                for point in points_in_quad.iter() {
                    let dist = (point - pos_vec2).norm();
                    if dist < closest_point {
                        closest_point = dist
                    }
                }

                // Assign cost based on the distance to the closest point. If it's less than the
                // onset distance the cost is zero, if it's greater than the max distance it's the
                // max cost. If it's between it's a linear map between the two.
                if closest_point < params.gnd_path_cost_onset_semi_width_m {
                    CostMapData::Cost(0.0)
                } else if closest_point > params.max_gnd_path_cost_semi_width_m {
                    max_cost_val
                } else {
                    let cost = util::maths::lin_map(
                        (
                            params.gnd_path_cost_onset_semi_width_m,
                            params.max_gnd_path_cost_semi_width_m,
                        ),
                        (0.0, params.max_gnd_path_added_cost),
                        closest_point,
                    );

                    // Clamp cost to the unsafe value of 1.0
                    if cost >= 1.0 {
                        CostMapData::Unsafe
                    } else {
                        CostMapData::Cost(cost)
                    }
                }
            })?;

        self.calculate_total()
    }

    /// Calculate the gradient cost of the given terrain map
    fn calculate_gradient(
        params: &CostMapParams,
        terrain_map: &TerrainMap,
    ) -> Result<Array2<CostMapData>, GridMapError> {
        let mut gradient = Array2::from_elem(
            (terrain_map.num_cells.x(), terrain_map.num_cells.y()),
            CostMapData::Unsafe,
        );

        // First a simple finite differencing algorithm.
        Zip::from(
            terrain_map
                .get_layer(TerrainMapLayer::Height)?
                .windows((3, 3)),
        )
        .and(gradient.slice_mut(s![
            1..terrain_map.num_cells.x() - 1,
            1..terrain_map.num_cells.y() - 1
        ]))
        .for_each(|window, grad| {
            // Calculate dh/dx and dh/dy by comparing neighbours in each direction
            let dh_dx = match (window[[2, 1]], window[[0, 1]]) {
                (Some(a), Some(b)) => Some((a - b) / (2.0 * terrain_map.cell_size.x())),
                _ => None,
            };
            let dh_dy = match (window[[1, 2]], window[[1, 0]]) {
                (Some(a), Some(b)) => Some((a - b) / (2.0 * terrain_map.cell_size.y())),
                _ => None,
            };

            // Get gradient as the magnitude of the total vector, or as the magnitude
            // of the single vector if there's one missing, or none otherwise
            *grad = match (dh_dx, dh_dy) {
                (Some(x), Some(y)) => {
                    // Check if cost is above max
                    let cost = (x * x + y * y).sqrt() * params.gradient_cost_factor;
                    if cost > params.max_safe_gradient {
                        CostMapData::Unsafe
                    } else {
                        CostMapData::Cost(cost)
                    }
                }
                (Some(x), None) => {
                    // Check if cost is above max
                    let cost = x.abs() * params.gradient_cost_factor;
                    if cost > params.max_safe_gradient {
                        CostMapData::Unsafe
                    } else {
                        CostMapData::Cost(cost)
                    }
                }
                (None, Some(y)) => {
                    // Check if cost is above max
                    let cost = y.abs() * params.gradient_cost_factor;
                    if cost > params.max_safe_gradient {
                        CostMapData::Unsafe
                    } else {
                        CostMapData::Cost(cost)
                    }
                }
                _ => CostMapData::None,
            }
        });

        Ok(gradient)
    }

    fn calculate_total(&mut self) -> Result<(), GridMapError> {
        for y in 0..self.num_cells.y() {
            for x in 0..self.num_cells.x() {
                // Build current cell
                let cell = Point2::new(x, y);

                // Add costs
                let mut cost = self.get(CostMapLayer::Gradient, &cell)?;
                cost.add(&self.get(CostMapLayer::GroundPlannedPath, &cell)?);

                *self.0.get_mut(CostMapLayer::Total, &cell)? = cost;
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
    pub fn get_path_cost(&self, path: &Path) -> Result<CostMapData, GridMapError> {
        // Array of all cells crossed by the path
        let mut cells_in_path = Vec::new();

        // Accumulate cost for the path
        let mut cost = CostMapData::Cost(0.0);

        // Add all cells by traversing each segment
        for target in 1..(path.points_m.len() - 1) {
            for cell in self.get_cells_on_segment(
                path.points_m[target - 1].into(),
                path.points_m[target].into(),
            )? {
                // Check that the cell hasn't already been visited
                if !cells_in_path.contains(&cell) {
                    cost.add_without_max(&self.get(CostMapLayer::Total, &cell)?);
                    cells_in_path.push(cell);
                }
            }
        }

        Ok(cost)
    }

    fn get_cells_on_segment(
        &self,
        start: Point2<f64>,
        end: Point2<f64>,
    ) -> Result<Vec<Point2<usize>>, GridMapError> {
        // Vec to hold the cells
        let mut cells = Vec::new();

        // Get the start and end cells from the given positions
        let mut cell = self.position_to_cell(&start)?;
        let end_cell = self.position_to_cell(&end)?;

        // Short circuit: if cells are the same exit with just that cell
        if cell == end_cell {
            cells.push(cell);
            return Ok(cells);
        }

        // Get the cell size
        let cell_size_x = self.cell_size.x();
        let cell_size_y = self.cell_size.y();

        // Get the total distance to move, and the signs of those distances
        let dist_x = end.x() - start.x();
        let dist_y = end.y() - start.y();
        let sign_x = -dist_x.signum() as isize;
        let sign_y = -dist_y.signum() as isize;

        // parametric distance to move in each x and y
        let mut param_x = dist_y * ((cell.x() as f64 * cell_size_y) + (sign_x as f64) - start.x());
        let mut param_y = dist_x * ((cell.y() as f64 * cell_size_y) + (sign_y as f64) - start.y());

        println!("Start cell = ({}, {})", cell.x(), cell.y());
        println!("End cell = ({}, {})", end_cell.x(), end_cell.y());
        println!("signs = ({}, {})", sign_x, sign_y);

        // While we aren't at the end cell
        let mut moves = 0;
        while cell != end_cell {
            println!("params = ({}, {})", param_x, param_y);
            let move_x = param_x <= param_y;
            let move_y = param_y <= param_x;

            // Calculate if we will intersect an x axis next
            if move_x {
                // Increment the cell's x position
                cell[0] = ((cell[0] as isize) + sign_x) as usize;

                // Adjust the param_x
                param_x = dist_y * ((cell.x() as f64 * cell_size_x) + (sign_x as f64) - start.x());
            }

            // Calculate if we will intersect an y axis next
            if move_y {
                // Increment the cell's y position
                cell[1] = ((cell[1] as isize) + sign_x) as usize;

                // Adjust the param_y
                param_y = dist_x * ((cell.y() as f64 * cell_size_y) + (sign_y as f64) - start.y());
            }

            println!("Moved to cell ({}, {})", cell.x(), cell.y());
            if moves < 10 {
                moves += 1;
            } else {
                panic!("HELP");
            }
        }

        Ok(cells)
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
        S: Serializer,
    {
        // Convert to a SerializableCostMap
        let ser = SerializableGridMap::from_grid_map(self);

        ser.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for CostMap {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        // Deserialize into a SerializableGridMap
        let ser: SerializableGridMap<CostMapData, CostMapLayer> =
            SerializableGridMap::deserialize(deserializer)?;

        // Convert to cost map
        Ok(CostMap(
            ser.to_grid_map()
                .expect("Couldn't deserialize grid map to CostMap"),
        ))
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
