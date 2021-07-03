//! # Navigation
//!
//! This module provides types for high level navigation of the rover. In particular:
//!
//!  - [`PathPlanner`] plans minimum cost paths through a [`CostMap`], using an A* algorithm.
//!

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use nalgebra::{Point2, Unit, UnitQuaternion, Vector2, Vector3};
use serde::{Deserialize, Serialize};

use super::{
    loc::Pose,
    path::{Path, PathError},
};

// -----------------------------------------------------------------------------------------------
// MODS
// -----------------------------------------------------------------------------------------------

pub mod path_planner;

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

/// Navigation pose, consisting of a 2D position and rotation a [`CellMap`]'s parent frame.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct NavPose {
    /// Position in the a map's parent frame.
    pub position_m: Point2<f64>,

    /// Heading in the map's parent frame (+ve counter clockwise about map Z axis)
    pub heading_rad: f64,

    /// The corresponding full 3D pose in the map's parent frame.
    pub pose_parent: Pose,
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum NavError {
    #[error("The point {0} {1} was outside of the map")]
    PointOutsideMap(String, Point2<f64>),

    #[error("Couldn't build the path fan: {0}")]
    CouldNotBuildFan(PathError),

    #[error("Could not find an optimal path that reaches the target, returning best fit instead")]
    BestPathNotAtTarget(Vec<Path>),

    #[error("Couldn't find a traversable path to the target")]
    NoPathToTarget,
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl NavPose {
    /// Creates a [`NavPose`] from the given [`Pose`] in the map's parent frame.
    pub fn from_parent_pose(pose: &Pose) -> Self {
        // Get the 2D position in the parent
        let pos_parent = Point2::new(pose.position_m.x, pose.position_m.y);

        // Get the forward vector
        let forward_parent = pose.forward2();

        // Get the angle of the forward vector
        let heading_parent = forward_parent.angle(&Vector2::x());

        // Construct self
        Self {
            position_m: pos_parent,
            heading_rad: heading_parent,
            pose_parent: *pose,
        }
    }

    /// Creates a [`NavPose`] from the given components
    pub fn from_parts(position_m: &Point2<f64>, heading_rad: &f64) -> Self {
        // Create the 3D position vector
        let pos_parent_3d = Vector3::new(position_m.x, position_m.y, 0.0);

        // Build the quaternion as a rotation about the z axis.
        let quat =
            UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::z()), *heading_rad);

        // Build pose
        Self {
            position_m: *position_m,
            heading_rad: *heading_rad,
            pose_parent: Pose {
                position_m: pos_parent_3d,
                attitude_q: quat,
            },
        }
    }

    /// Creates a new [`NavPose`] from the given path at the given target point index.
    pub fn from_path_point(path: &Path, point_index: usize) -> Option<Self> {
        let seg = path.get_segment_to_target(point_index)?;

        Some(Self::from_parts(
            &Point2::from(seg.target_m),
            &seg.heading_rad,
        ))
    }

    /// Creates a new [`NavPose`] at the start of the given path
    pub fn from_path_first_point(path: &Path) -> Self {
        let seg = path.get_segment_to_target(1).unwrap();

        Self::from_parts(&Point2::from(seg.start_m), &seg.heading_rad)
    }

    pub fn from_path_last_point(path: &Path) -> Self {
        let seg = path
            .get_segment_to_target(path.get_num_points() - 1)
            .unwrap();

        Self::from_parts(&Point2::from(seg.target_m), &seg.heading_rad)
    }
}
