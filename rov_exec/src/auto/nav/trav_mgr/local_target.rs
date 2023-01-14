//! Determines the local target in a check traverse

use crate::auto::{
    map::{CostMap, CostMapLayer},
    nav::NavPose,
    path::Path,
};

use super::TravMgrError;

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Stateful locator of the local target, which will find the furthest point along the path which
/// lies on a populated grid cell.
#[derive(Debug, Clone)]
pub struct LocalTarget {
    /// Index of the previously found furthest segment start index.
    previous_segment_end: usize,

    /// Distance to an unpopulated cell that will cause the target to be invalid.
    exclusion_distance_m: f64,

    /// Maximum distance between the current rover pose and the local target
    max_distance_m: f64,
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl LocalTarget {
    pub fn new(exclusion_distance_m: f64, max_distance_m: f64) -> Self {
        Self {
            previous_segment_end: 1,
            exclusion_distance_m,
            max_distance_m,
        }
    }

    /// Get the next valid target in the given global terrain map.
    pub fn next(
        &mut self,
        current_pose: &NavPose,
        path: &Path,
        global_cost_map: &CostMap,
    ) -> Result<NavPose, TravMgrError> {
        let mut extreme_pose = NavPose::from_path_point(path, self.previous_segment_end)
            .ok_or(TravMgrError::NoValidTarget)?;

        while global_cost_map
            .get_position(CostMapLayer::Total, extreme_pose.position_m)
            .ok_or(TravMgrError::PointOutsideMap)?
            .cost()
            .is_some()
        {
            self.previous_segment_end += 1;
            extreme_pose = NavPose::from_path_point(path, self.previous_segment_end)
                .ok_or(TravMgrError::NoValidTarget)?;

            // Check not beyond max path length
            if (extreme_pose.position_m - current_pose.position_m).norm() > self.max_distance_m {
                break;
            }
        }

        // Get the point at least the threshold way from the previous target
        let mut target_pose_idx = self.previous_segment_end - 1;

        // Return error if rover is outside of map
        let mut target_pose = match NavPose::from_path_point(path, target_pose_idx) {
            None => return Err(TravMgrError::RoverOutsideMap),
            Some(value) => value,
        };

        // TODO: make the exclusion check test all cells in the distance, rather than the distance
        // from the extreme target to the current target.
        while (extreme_pose.position_m - target_pose.position_m).norm() < self.exclusion_distance_m
        {
            target_pose_idx -= 1;
            target_pose = NavPose::from_path_point(path, target_pose_idx)
                .ok_or(TravMgrError::NoEscapeBoundary)?;
        }

        Ok(target_pose)
    }
}
