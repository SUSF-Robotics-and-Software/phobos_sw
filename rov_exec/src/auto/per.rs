//! Perception module - converts from depth images to heightmaps

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use cell_map::{Bounds, CellMapParams};
use nalgebra::{Point2, Point3, Vector2};
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
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerMgrParams {
    /// Multiplier to convert from depth units to meters
    pub depth_to_m: f64,

    /// The principle point (middle) of the calibrated depth image.
    pub principle_point_pixels: Point2<f64>,

    /// The focal length of the x and y axes.
    pub focal_length_pixels: Point2<f64>
}

// -----------------------------------------------------------------------------------------------
// ENUMS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone, thiserror::Error)]
pub enum PerError {
    #[error("The provided depth image was empty")]
    DepthImgIsEmpty,
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
            params,
            dummy_global_terrain,
        }
    }

    /// Calculate the terrain map from the given depth image and pose.
    ///
    /// `depth_img` - the image to process
    /// `pose` - the pose of the rover when the image was taken, in the global map frame.
    pub fn calculate(&self, depth_img: &DepthImage, pose: &Pose) -> Result<TerrainMap, PerError> {
        let mut point_cloud: Vec<Point3<f64>> = Vec::with_capacity(depth_img.image.len());

        // Calculating the bounds of the point cloud
        let mut min_bound = Point3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max_bound = Point3::new(f64::MIN, f64::MIN, f64::MIN);

        // Calculation of each point
        for (u, v, depth) in depth_img.image.enumerate_pixels() {
            let depth_m = depth * self.params.depth_to_m;
            
            let point_m = Point3::new(
                (u - self.params.principle_point_pixels.x) * depth_m / self.params.focal_length_pixels.x,
                (y - self.params.principle_point_pixels.y) * depth_m / self.params.focal_length_pixels.y,
                depth_m,
            );

            // TODO: probably a way of doing this using iterators but I can't be bothered to find it now
            if point_m.x < min_bound.x {
                min_bound.x = point_m.x;
            }
            if point_m.y < min_bound.y {
                min_bound.y = point_m.y;
            }
            if point_m.z < min_bound.z {
                min_bound.z = point_m.z;
            }
            if point_m.x > max_bound.x {
                min_bound.x = point_m.x;
            }
            if point_m.y > max_bound.y {
                min_bound.y = point_m.y;
            }
            if point_m.z > max_bound.z {
                min_bound.z = point_m.z;
            }

            point_cloud.push(point_m);
        }

        // Calculate point cloud
        //  need min and max bounds of the point cloud.

        // From point cloud, create empty terrain map
        //  point cloud goes from (0.5, 7.0) -> (10.0, 15.0), need map to include that region of
        //  space.

        // Do calculations as described above

        todo!()
    }

    pub fn get_dummy_terr_map(&self) -> &TerrainMap {
        &self.dummy_global_terrain
    }
}
