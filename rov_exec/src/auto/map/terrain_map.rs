//! # Terrain Map

// ------------------------------------------------------------------------------------------------
// INCLUDES
// ------------------------------------------------------------------------------------------------

use std::ops::{Deref, DerefMut, Range};

use cell_map::{Bounds, CellMap, CellMapParams, Error as CellMapError, Layer};
use log::trace;
use nalgebra::{Isometry2, Matrix1x2, Matrix2, Matrix2x1, Point2, Vector2};
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
        // First get the bounds of `other` wrt `self`, which we have to do by accounting for the
        // potential different alignment of `other` wrt `parent`. We do this by getting the corner
        // points, transforming from `other` to `parent`, then from `parent` to `self`. We have to
        // transform all corner points because rotation may lead to the corners being in different
        // positions than when aligned to `other`.
        let other_bounds = other.cell_bounds();
        let corners_in_other = vec![
            Point2::new(other_bounds.x.0, other_bounds.y.0).cast(),
            Point2::new(other_bounds.x.1, other_bounds.y.0).cast() + Vector2::new(1.0, 0.0),
            Point2::new(other_bounds.x.0, other_bounds.y.1).cast() + Vector2::new(0.0, 1.0),
            Point2::new(other_bounds.x.1, other_bounds.y.1).cast() + Vector2::new(1.0, 1.0),
        ];
        let corners_in_parent: Vec<Point2<f64>> = corners_in_other
            .iter()
            .map(|c| other.to_parent().transform_point(c))
            .collect();
        let other_bl_parent = Point2::new(
            corners_in_parent
                .iter()
                .min_by_key(|c| c.x.floor() as isize)
                .unwrap()
                .x
                .floor(),
            corners_in_parent
                .iter()
                .min_by_key(|c| c.y.floor() as isize)
                .unwrap()
                .y
                .floor(),
        );
        let other_ur_parent = Point2::new(
            corners_in_parent
                .iter()
                .max_by_key(|c| c.x.ceil() as isize)
                .unwrap()
                .x
                .ceil(),
            corners_in_parent
                .iter()
                .max_by_key(|c| c.y.ceil() as isize)
                .unwrap()
                .y
                .ceil(),
        );
        let other_in_self = Bounds::from_corner_positions(&self, other_bl_parent, other_ur_parent);

        // Calculate the union of both bounds
        let new_bounds = self.cell_bounds().union(&other_in_self);

        // Resize self
        self.resize(new_bounds);

        // Iterate over the cells in self which overlap with other
        for ((layer, pos), value) in self.iter_mut().positioned() {
            // Interpolate the cost of the point, if it's Some we need to modify self
            // TODO: interp seems to not work very well, just using get instead
            // if let Some(cost) = other.bilinterp(layer, pos) {
            let idx = match other.index(pos) {
                Some(i) => i,
                None => continue,
            };
            if let Some(height) = other.get(layer, idx) {
                let cost = match height {
                    Some(h) => *h,
                    None => continue,
                };

                // If the current value is some replace it with the average of the two, otherwise
                // just asign the interpolated value
                if let Some(current) = value {
                    *current = 0.5 * (cost + *current);
                } else {
                    *value = Some(cost);
                }
            }
        }
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
        let submap_to_self = Isometry2::new(position.coords, heading);

        // Loop variables
        let mut in_fov;

        for ((layer, pos), value) in submap.0.iter_mut().positioned() {
            // Check if position is inside the field of view
            in_fov = true;

            // Polar coordinates between the rover and the pos.
            let radius = pos.coords.norm();
            let theta = pos.y.atan2(pos.x);

            // Check radius is within the view range
            if radius < view_range_m.start || radius > view_range_m.end {
                in_fov = false;
            }

            // Check the theta is within the field of view
            if theta.abs() > field_of_view_rad / 2.0 {
                in_fov = false;
            }

            // If not inside the FOV skip this position
            if !in_fov {
                continue;
            }

            // If inside the FOV transform the position into the self frame
            let pos_self = submap_to_self.transform_point(&pos);

            *value = self.bilinterp(layer, pos_self);
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

    /// Do bilinear interpolation to get the value at the given position
    fn bilinterp(&self, layer: TerrainMapLayer, position: Point2<f64>) -> Option<f64> {
        super::bilinterp(&self.0, layer, position, |height| *height)
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
