//! # Terrain Map

// ------------------------------------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------------------------------------

use std::ops::{Deref, DerefMut, Range};

use cell_map::{Bounds, CellMap, CellMapParams, Error as CellMapError, Layer};
use log::trace;
use nalgebra::{Isometry2, Point2};
use noise::{NoiseFn, Perlin};
use serde::{Deserialize, Serialize};

use crate::auto::{loc::Pose, nav::NavPose};

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

    /// Move the map to the new nav pose
    pub fn move_map(&mut self, new_pose: &NavPose) {
        self.0
            .move_map(new_pose.position_m.coords, new_pose.heading_rad);
    }

    /// Merge `other` into `self`, modifying `self`.
    ///
    /// This will average overlapping heights
    pub fn merge(&mut self, other: &Self) {
        self.0.merge(&other.0, |my_cell, other_cells| {
            // Aproach is to average any overlapping heights
            let mut acc = *my_cell;
            let mut num_cells = if acc.is_some() { 1 } else { 0 };

            for cell in other_cells {
                match cell {
                    Some(h) => {
                        num_cells += 1;
                        match acc {
                            Some(ref mut acc) => *acc += *h,
                            None => acc = Some(*h),
                        }
                    }
                    None => (),
                }
            }

            acc.map(|h| h / num_cells as f64)
        });
    }

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

    /// Gets a submap at of self at the given pose, such that the submap's x axis is aligned with
    /// the pose. The submap will be clipped as if it were in the rover field of view.
    ///
    /// None is returned if pose is not in the map. Pose shall be relative to the map's parent.
    pub fn get_submap_at(
        &self,
        pose: &Pose,
        view_range_m: Range<f64>,
        field_of_view_rad: f64,
    ) -> Option<Self> {
        // Check if the pose is in the map
        self.0.index(pose.position2().into())?;

        // Create submap which will contain the view range and field of view, using the same cell
        // size as self.
        let cell_size = self.cell_size();
        let y_abs_bound =
            (view_range_m.end * field_of_view_rad.sin() / cell_size.y).ceil() as isize;
        let mut submap = Self::new(CellMapParams {
            cell_bounds: Bounds::new(
                (0, (view_range_m.end / cell_size.x).ceil() as isize),
                (-y_abs_bound, y_abs_bound),
            )
            .ok()?,
            cell_size,
            cell_boundary_precision: self.params().cell_boundary_precision,
            ..Default::default()
        });

        // Get transform from self to the pose frame
        let position: Point2<f64> = pose.position2().into();
        let heading = pose.get_heading();
        let self_to_submap = Isometry2::new(position.coords, heading).inverse();

        // Loop variables
        let mut in_fov;

        for ((layer, pos), value) in self.iter().positioned() {
            // Skip none
            if value.is_none() {
                continue;
            }

            // Transform the position into the submap frame
            let pos_submap = self_to_submap.transform_point(&pos);

            // Get the index of this position in the submap, if it's not in the map skip it
            if let Some(idx) = submap.index(pos_submap) {
                // Check if position is inside the field of view
                in_fov = true;

                // Polar coordinates between the rover and the pos.
                let radius = pos_submap.coords.norm();
                let theta = pos_submap.y.atan2(pos_submap.x);

                // Check radius is within the view range
                if radius < view_range_m.start || radius > view_range_m.end {
                    in_fov = false;
                }

                // Check the theta is within the field of view
                if theta.abs() > field_of_view_rad / 2.0 {
                    in_fov = false;
                }

                // If the cell is in the FOV we keep it, if it isn't we set it to None
                if in_fov {
                    // trace!("    {} -> {}", pos, pos_submap);
                    submap[(layer, idx)] = *value;
                }
            }
        }

        Some(submap)
    }

    /// Dummy test function that simulates the rover's field of view of the terrain.
    pub fn clip_to_rov_view(&mut self, pose: &Pose, view_range: Range<f64>, field_of_view: f64) {
        let mut in_fov;

        let position: Point2<f64> = pose.position2().into();
        let heading = pose.get_heading();

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
