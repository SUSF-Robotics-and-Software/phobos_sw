//! # Path
//!
//! This module defines the path used by the autonomy system.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use serde::{Serialize, Deserialize};
use nalgebra::Vector2;

use super::loc::Pose;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A path defining the desired trajectory of the rover.
#[derive(Clone, Serialize, Deserialize)]
pub struct Path {
    points_m_lm: Vec<Vector2<f64>>
}

/// A segment between two path points
#[derive(Default, Serialize, Deserialize)]
pub struct PathSegment {

    /// The target of the segment
    pub target_m_lm: Vector2<f64>,

    /// The start point of the segment
    pub start_m_lm: Vector2<f64>,

    /// The length of the segment
    pub length_m: f64,

    /// The slope (dy/dx) of the segment
    pub slope_m: f64,

    /// The intercept (the c in y = mx + c) of the segment
    pub intercept_m: f64,

    /// The heading (angle to the +ve x axis) of the segment
    pub heading_rad: f64
}

/// A sequence of reduced (curv only) Ackermann manouvres which describes a path.
///
/// The first element is the curvature in 1/meters, the second the distance in meters.
#[derive(Clone, Serialize, Deserialize)]
pub struct AckSequence(
    Vec<(f64, f64)>
);

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Path {
    /// Create a new empty path
    pub fn new_empty() -> Self {
        Path {
            points_m_lm: Vec::new()
        }
    }

    /// Returns the path segment connecting the target point and the previous
    /// point.
    ///
    /// If no segment exists (the target is the first point in the sequence or
    /// is beyond the end of the sequence) then `None` will be returned
    pub fn get_segment_to_target(
        &self, 
        target_index: usize
    ) -> Option<PathSegment> {

        // If the path is invalid (not enough points)
        if self.points_m_lm.len() < 2 {
            return None;
        }

        // Catch invalid targets
        if target_index == 0 || target_index > self.points_m_lm.len() {
            return None;
        }

        // Empty segment to start with
        let mut seg = PathSegment::default();

        // Set the target and start
        seg.target_m_lm = self.points_m_lm[target_index];
        seg.start_m_lm = self.points_m_lm[target_index - 1];

        // Set the length of the segment.
        //
        // The unwrap here is safe since we know both start and target have
        // the same dimentions.
        seg.length_m = (seg.target_m_lm - seg.start_m_lm).norm();

        let dx = seg.target_m_lm[0] - seg.start_m_lm[0];
        let dy = seg.target_m_lm[1] - seg.start_m_lm[1];
        // Slope is the change in y over the change in x
        seg.slope_m = dy/dx;

        // The heading is then the arctan of the slope
        seg.heading_rad = dy.atan2(dx);

        // The intercept is then targ_y - slope * targ_x
        seg.intercept_m = seg.target_m_lm[1]  
            - seg.slope_m * seg.target_m_lm[0];

        // Return the segment
        Some(seg)
    }

    /// Return the length of the path in meters.
    ///
    /// If the path is empty (not enough points) then `None` is returned.
    pub fn get_length(&self) -> Option<f64> {
        
        // If the path is invalid (not enough points)
        if self.points_m_lm.len() < 2 {
            return None;
        }

        let mut length_m = 0f64;

        // Length is defined as the sum of the length of all path segments
        for i in 1..self.points_m_lm.len() {
            length_m += self.get_segment_to_target(i)
                .unwrap()
                .length_m;
        }

        Some(length_m)
    }

    /// Get the number of points in the path
    pub fn get_num_points(&self) -> usize {
        self.points_m_lm.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points_m_lm.len() == 0
    }
}

impl AckSequence {
    /// Convert this sequence into a standard [`Path`].
    ///
    /// If the sequence is empty `None` is returned.
    pub fn to_path(self, start_pose: &Pose, point_sep_m: f64) -> Option<Path> {
        // If sequence is empty just return None
        if self.0.len() == 0 {
            return None
        }

        // Create new empty path
        let mut path = Path::new_empty();

        // For each Ack in the sequence calculate the points at the given separation
        for (curv_m, dist_m) in self.0 {
            // If this is the first sequence the start point and heading come from the starting
            // pose, otherwise they come from the current end of the path
            let (x_0, y_0, head_rad) = match path.is_empty() {
                true => {
                    (
                        start_pose.position_m_lm[0], 
                        start_pose.position_m_lm[1], 
                        start_pose.attitude_q_lm.euler_angles().2
                    )
                },
                false => {
                    let last = path.points_m_lm.last().unwrap();
                    (
                        last[0], 
                        last[1], 
                        path
                            .get_segment_to_target(path.get_num_points() - 1)
                            .unwrap()
                            .heading_rad
                    )
                }
            };
            
            // Precalculate the anti-heading (head + pi/2)
            let anti_head_rad = head_rad + std::f64::consts::PI/2.0;

            // Get a linearly spaced vector of s values (dist along ack)
            let num_s = (dist_m / point_sep_m).ceil() as usize;
            let mut s_values_m = Vec::with_capacity(num_s);
            for i in 0..num_s {
                s_values_m.push((i as f64 * point_sep_m).min(dist_m));
            }

            // Iterate over values of s, calculating a new point for the path for each one
            for s_m in s_values_m {
                // Calculate the -sk - head value
                let sk_phi = -s_m * curv_m - head_rad;

                path.points_m_lm.push(Vector2::new(
                    x_0 + 1.0/curv_m*(anti_head_rad.cos() - sk_phi.sin()),
                    y_0 + 1.0/curv_m*(anti_head_rad.sin() - sk_phi.cos())
                ));
            }
        }

        Some(path)
    }
}

#[cfg(test)]
mod test {
    use nalgebra::{UnitQuaternion, Vector3};
    use std::f64::consts::PI;

    use super::*;

    #[test]
    fn test_ack_seq() {
        let ack_seq = AckSequence(vec![
            (1.0, 1.57),
            (0.5, 2.0)
        ]);

        // Convert ack_seq to a path
        let path = ack_seq.to_path(&Pose {
            position_m_lm: Vector3::default(),
            attitude_q_lm: UnitQuaternion::from_euler_angles(0.0, 0.0, PI),
        }, 0.05).unwrap();

        // Serialize the path and write it out so we can plot it to check it's correct
        let path_json = serde_json::to_string_pretty(&path).unwrap();
        std::fs::write("test_path.json", path_json).unwrap();
    }
}