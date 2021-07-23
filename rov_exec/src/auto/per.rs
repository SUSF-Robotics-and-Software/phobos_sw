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
pub struct PerMgrParams {}

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
        // TEMP: return portion of global cost map clipped to the current pose
        return self
            .dummy_global_terrain
            .get_submap_at(pose, 0.3..2.0, 1.2)
            .ok_or(PerError::DepthImgIsEmpty);

        // Calculate point cloud
        //  need min and max bounds of the point cloud.

        let mut point_cloud: Vec<Point3<f64>> = Vec::new();

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
