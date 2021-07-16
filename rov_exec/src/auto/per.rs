//! Perception module - converts from depth images to heightmaps

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use std::{fs::File, time::Duration};

use log::info;
use nalgebra::{Affine3, Isometry3, Point2, Point3, Translation, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use util::session::save_with_timestamp;

use crate::auto::{
    loc::Pose,
    map::{TerrainMap, TerrainMapLayer},
};
use comms_if::eqpt::perloc::DepthImage;

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

    /// Transformation from depth image frame to rover body frame
    pub di_to_rbl: Isometry3<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerMgrParams {
    /// The acceptable range (semi-open, min < depth <= max) of depth to calculate in millimeters.
    pub depth_range_mm: (u16, u16),

    /// Location of the principle point in the depth image
    pub principle_point_pixels: Point2<f64>,

    /// Focal length of the depth image in pixels
    pub focal_length_pixels: Point2<f64>,

    /// The position of the depth image optical centre with respect to the rover body frame
    pub depth_img_pos_m_rb: Vector3<f64>,

    /// The attitude of the depth image optical centre in the rover body frame
    pub depth_img_att_q_rb: UnitQuaternion<f64>,
}

/// Millimeters to meters
const MM_TO_M: f64 = 0.0001;

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
        Self {
            di_to_rbl: Isometry3::from_parts(
                Translation::from(params.depth_img_pos_m_rb).inverse(),
                params.depth_img_att_q_rb,
            ),
            params,
        }
    }

    /// Calculate the terrain map from the given depth image and pose.
    ///
    /// `depth_img` - the image to process
    /// `pose` - the pose of the rover when the image was taken, in the global map frame.
    pub fn calculate(&self, depth_img: &DepthImage, pose: &Pose) -> Result<TerrainMap, PerError> {
        // Calculate point cloud
        //  need min and max bounds of the point cloud.

        let mut point_cloud_rb: Vec<Point3<f64>> = Vec::new();

        // Tracking points for extents of the point cloud, we will set this in the loop so we only
        // have one loop. Note these don't correspond to points themselves but to bounds
        let mut min = Point3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max = Point3::new(f64::MIN, f64::MIN, f64::MIN);

        // Iterate over every pixel in the image, calculate it's position, and insert it into the
        // point cloud
        let start = std::time::Instant::now();
        for (x, y, depth) in depth_img.image.enumerate_pixels() {
            // Get depth as a simple u16
            let depth = depth.0[0];

            // If the depth is outside the range ignore it
            if depth < self.params.depth_range_mm.0 || depth > self.params.depth_range_mm.1 {
                continue;
            }

            // Get the point in the depth image frame
            let point_di = Point3::new(
                (x as f64 - self.params.principle_point_pixels.x)
                    / self.params.focal_length_pixels.x,
                (y as f64 - self.params.principle_point_pixels.y)
                    / self.params.focal_length_pixels.y,
                depth as f64 * MM_TO_M,
            );

            // Transform that point to rbl, then swap axes so we go from the weird left handed rb
            // frame to right handed normal rb
            let point_rbl = self.di_to_rbl.transform_point(&point_di);
            let point_rb = Point3::new(point_rbl.z, point_rbl.y, -point_rbl.x);

            // Check min/max
            if point_rb.x < min.x {
                min.x = point_rb.x
            }
            if point_rb.y < min.y {
                min.y = point_rb.y
            }
            if point_rb.z < min.z {
                min.z = point_rb.z
            }
            if point_rb.x < max.x {
                max.x = point_rb.x
            }
            if point_rb.y < max.y {
                max.y = point_rb.y
            }
            if point_rb.z < max.z {
                max.z = point_rb.z
            }

            // Insert into point cloud
            point_cloud_rb.push(point_rb)
        }
        let end = std::time::Instant::now();
        info!(
            "Point cloud took {} s to calculate",
            (end - start).as_secs_f64()
        );

        serde_json::to_writer(&File::create("point_cloud.json").unwrap(), &point_cloud_rb).unwrap();

        // save_with_timestamp("point_clouds/point_cloud.json", point_cloud_rb);

        // From point cloud, create empty terrain map
        //  point cloud goes from (0.5, 7.0) -> (10.0, 15.0), need map to include that region of
        //  space.

        // Do calculations as described above

        todo!()
    }
}
