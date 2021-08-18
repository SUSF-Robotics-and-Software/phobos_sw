//! Perception module - converts from depth images to heightmaps

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use std::collections::HashMap;

use cell_map::{Bounds, CellMapParams};
use log::{info, warn};
use nalgebra::{Isometry3, Point2, Point3, Translation3, UnitQuaternion, Vector2};
use serde::{Deserialize, Serialize};
use util::{params, session};

use crate::auto::{
    loc::Pose,
    map::{TerrainMap, TerrainMapLayer},
};
use comms_if::eqpt::perloc::DepthImage;

use super::map::{CostMap, CostMapParams};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

/// Manages the perception algorithm, converting between depth images and terrain maps.
///
/// General procedure for terrain map calculation:
///  - Calculate point cloud from the depth image
///  - Bin all points into the grid of the terrain map
///  - Do some inlier detection of the points in each cell - RANSAC?
///  - Calculate estimated height, and assign to the cell
#[derive(Debug, Clone)]
pub struct PerMgr {
    pub params: PerMgrParams,

    dummy_global_terrain: TerrainMap,

    /// Transformation from left camera frame to rover body frame
    depth_img_to_rb: Isometry3<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerMgrParams {
    /// Multiplier to convert from depth units to meters
    pub depth_to_m: f64,

    /// Minimum valid depth in meters from the depth image optical centre
    pub min_depth_m: f64,

    /// Maximum valid depth im meters from the depth image optical centre
    pub max_depth_m: f64,

    /// The principle point (middle) of the calibrated depth image.
    pub principle_point_pixels: Point2<f64>,

    /// The focal length of the x and y axes.
    pub focal_length_pixels: Point2<f64>,

    /// Position of the depth image frame in the rover body frame
    pub depth_img_pos_m_rb: Translation3<f64>,

    /// Attitude of the depth image frame in the rover body frame (quaternion)
    pub depth_img_att_q_rb: UnitQuaternion<f64>,
}

// -----------------------------------------------------------------------------------------------
// ENUMS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone, thiserror::Error)]
pub enum PerError {
    #[error("The provided depth image was empty")]
    DepthImgIsEmpty,

    #[error("The bounds on the point cloud are invalid")]
    PointCloudBoundsInvalid,
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl PerMgr {
    pub fn new(params: PerMgrParams) -> Self {
        let dummy_global_terrain = TerrainMap::generate_random(
            CellMapParams {
                cell_size: Vector2::new(0.1, 0.1),
                cell_bounds: Bounds::new((0, 100), (0, 100)).unwrap(),
                ..Default::default()
            },
            Point2::new(0.1, 0.1),
            Point2::origin(),
        )
        .unwrap();

        session::save("global_terrain.json", dummy_global_terrain.clone());

        Self {
            depth_img_to_rb: Isometry3::from_parts(
                params.depth_img_pos_m_rb,
                params.depth_img_att_q_rb,
            ),
            params,
            dummy_global_terrain,
        }
    }

    /// Calculate the local terrain map from the given depth image and pose.
    ///
    /// The local terrain map is relative to the rover body position at the time the depth image
    /// taken. It must be merged into the global terrain map using the `TerrainMap::move_map` and
    /// `TerrainMap::merge` functions first.
    ///
    /// `depth_img` - the image to process
    /// `cell_size_m` - the size of each terrain map cell in meters
    pub fn calculate(
        &self,
        depth_img: &DepthImage,
        cell_size_m: &Vector2<f64>,
    ) -> Result<TerrainMap, PerError> {
        let mut bins: HashMap<Vector2<isize>, Vec<f64>> = HashMap::new();

        // Calculating the bounds of the point cloud
        let mut min_bound = Point2::new(f64::MAX, f64::MAX);
        let mut max_bound = Point2::new(f64::MIN, f64::MIN);

        // Calculation of each point
        for (u, v, depth) in depth_img.image.enumerate_pixels() {
            let depth_m = (depth.0[0] as f64) * self.params.depth_to_m;

            // Check if depth is valid, if not skip this point
            if depth_m < self.params.min_depth_m || depth_m > self.params.max_depth_m {
                continue;
            }

            let point_m_di = Point3::new(
                (u as f64 - self.params.principle_point_pixels.x) * depth_m
                    / self.params.focal_length_pixels.x,
                (v as f64 - self.params.principle_point_pixels.y) * depth_m
                    / self.params.focal_length_pixels.y,
                depth_m,
            );

            // Transform the point into the RB frame
            let point_m_rb = self.depth_img_to_rb.transform_point(&point_m_di);

            // Get index of this point
            let idx = point_m_rb
                .coords
                .xy()
                .component_div(cell_size_m)
                .map(|v| v as isize);

            // Put the height into the bin
            bins.entry(idx).or_insert_with(Vec::new).push(-point_m_rb.z);

            // TODO: probably a way of doing this using iterators
            if point_m_rb.x < min_bound.x {
                min_bound.x = point_m_rb.x;
            }
            if point_m_rb.y < min_bound.y {
                min_bound.y = point_m_rb.y;
            }
            if point_m_rb.x > max_bound.x {
                max_bound.x = point_m_rb.x;
            }
            if point_m_rb.y > max_bound.y {
                max_bound.y = point_m_rb.y;
            }
        }

        // Get the extents of the point cloud in cells by dividing the min/max bounds by the cell
        // size, ceiling for max and flooring for min
        let min_bound = min_bound
            .coords
            .component_div(cell_size_m)
            .map(|v| v.floor() as isize);
        let max_bound = max_bound
            .coords
            .component_div(cell_size_m)
            .map(|v| v.ceil() as isize);
        let map_bounds = Bounds::new((min_bound.x, max_bound.x), (min_bound.y, max_bound.y))
            .map_err(|_| PerError::PointCloudBoundsInvalid)?;

        info!("Terrain map bounds: {:?}", map_bounds);

        // Create empty terrain map with the extents of the point cloud.
        let mut tm = TerrainMap::new(CellMapParams {
            cell_size: *cell_size_m,
            cell_bounds: map_bounds,
            ..Default::default()
        });

        // Iterate over the bins, calculating the average height and then putting that into the
        // map for that cell
        // TODO: Change from averaging to inlier detection
        for (idx, heights) in bins {
            let avg_height: f64 = heights.iter().sum::<f64>() / heights.len() as f64;

            if let Some(idx) = map_bounds.get_index(Point2 { coords: idx }) {
                tm.set(TerrainMapLayer::Height, idx, Some(avg_height))
                    .unwrap();
            } else {
                warn!("Bin {} is outside the map", idx);
            }
        }

        Ok(tm)
    }

    pub fn get_dummy_terr_map(&self) -> &TerrainMap {
        &self.dummy_global_terrain
    }
}
