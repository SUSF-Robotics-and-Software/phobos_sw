//! # Path
//!
//! This module defines the path used by the autonomy system.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use std::{
    cmp::Reverse,
    collections::{BTreeSet, BinaryHeap},
};

use color_eyre::owo_colors::OwoColorize;
// External
use nalgebra::{Point2, Vector2};
use ordered_float::NotNan;
use serde::{Deserialize, Serialize};

use super::{loc::Pose, nav::NavPose};
use comms_if::tc::auto::PathSpec;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A path defining the desired trajectory of the rover.
#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct Path {
    pub points_m: Vec<Vector2<f64>>,
}

/// A segment between two path points
#[derive(Default, Serialize, Deserialize)]
pub struct PathSegment {
    /// The target of the segment
    pub target_m: Vector2<f64>,

    /// The start point of the segment
    pub start_m: Vector2<f64>,

    /// The length of the segment
    pub length_m: f64,

    /// The slope (dy/dx) of the segment
    pub slope_m: f64,

    /// The intercept (the c in y = mx + c) of the segment
    pub intercept_m: f64,

    /// The heading (angle to the +ve x axis) of the segment
    pub heading_rad: f64,

    /// Unit vector pointing in the direction of the segment
    pub direction: Vector2<f64>,
}

/// A sequence of reduced (curv only) Ackermann manouvres which describes a path.
///
/// The first element is the curvature in 1/meters, the second the distance in meters.
#[derive(Clone, Serialize, Deserialize)]
pub struct AckSequence {
    seq: Vec<(f64, f64)>,
    point_sep_m: f64,
}

// -----------------------------------------------------------------------------------------------
// ENUMS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum PathError {
    #[error("The PathSpec is invalid")]
    InvalidPathSpec,

    #[error("The PathSpec provided wasn't compatible with the type to be parsed")]
    UnexpectedPathSpecType,

    #[error("Attempted to create a path from an empty sequence")]
    EmptySequence,
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Path {
    /// Create a new empty path
    pub fn new_empty() -> Self {
        Path {
            points_m: Vec::new(),
        }
    }

    /// Produces a direct path between the two position vectors, with each point in the path having
    /// at most the given separation.
    pub fn direct(
        from: Vector2<f64>,
        to: Vector2<f64>,
        point_sep_m: f64,
    ) -> Result<Self, PathError> {
        let diff_vec = to - from;
        let dist = diff_vec.norm();
        // If the points are closer than the separation just produce a new path with the from and
        // to being the only points.
        if dist <= point_sep_m {
            Ok(Path {
                points_m: vec![from, to],
            })
        } else {
            // Get the number of points needed to get regular spacing of the given separation,
            // noting that we should floor this so we don't end up with the last two points being
            // more than the separation apart
            let num_points = (dist / point_sep_m).floor() as usize;

            // Get the delta vector that we can add to the previous step at each new point, i.e.
            // the difference vector but of length point_sep_m.
            let delta = point_sep_m / dist * diff_vec;

            // Create new path with only the first point
            let mut path = Self {
                points_m: vec![from],
            };

            // Add the new points to the path
            for i in 1..num_points {
                path.points_m.push(path.points_m[i - 1] + delta);
            }

            // Return the path
            Ok(path)
        }
    }

    /// Convert from a [`PathSpec`] object into a new path.
    pub fn from_path_spec(spec: PathSpec, pose: &Pose) -> Result<Self, PathError> {
        match spec {
            PathSpec::DirectTo { x, y, separation_m } => {
                Self::direct(pose.position2(), Vector2::new(x, y), separation_m)
            }
            PathSpec::AckSeq { .. } => AckSequence::from_path_spec(spec)?.into_path(pose),
            PathSpec::File { .. } => {
                unimplemented!()
            }
        }
    }

    /// Returns the path segment connecting the target point and the previous
    /// point.
    ///
    /// If no segment exists (the target is the first point in the sequence or
    /// is beyond the end of the sequence) then `None` will be returned
    pub fn get_segment_to_target(&self, target_index: usize) -> Option<PathSegment> {
        // If the path is invalid (not enough points)
        if self.points_m.len() < 2 {
            return None;
        }

        // Catch invalid targets
        if target_index == 0 || target_index > self.points_m.len() {
            return None;
        }

        // Empty segment to start with
        let mut seg = PathSegment::default();

        // Set the target and start
        seg.target_m = self.points_m[target_index];
        seg.start_m = self.points_m[target_index - 1];

        // Set the length of the segment.
        //
        // The unwrap here is safe since we know both start and target have
        // the same dimentions.
        seg.length_m = (seg.target_m - seg.start_m).norm();

        let dx = seg.target_m[0] - seg.start_m[0];
        let dy = seg.target_m[1] - seg.start_m[1];
        // Slope is the change in y over the change in x
        seg.slope_m = dy / dx;

        // The heading is then the arctan of the slope
        seg.heading_rad = dy.atan2(dx);

        // The intercept is then targ_y - slope * targ_x
        seg.intercept_m = seg.target_m[1] - seg.slope_m * seg.target_m[0];

        // Direction vector is [dx, dy] normalized by the length
        seg.direction = Vector2::new(dx / seg.length_m, dy / seg.length_m);

        // Return the segment
        Some(seg)
    }

    /// Return the length of the path in meters.
    ///
    /// If the path is empty (not enough points) then `None` is returned.
    pub fn get_length(&self) -> Option<f64> {
        // If the path is invalid (not enough points)
        if self.points_m.len() < 2 {
            return None;
        }

        let mut length_m = 0f64;

        // Length is defined as the sum of the length of all path segments
        for i in 1..self.points_m.len() {
            length_m += self.get_segment_to_target(i).unwrap().length_m;
        }

        Some(length_m)
    }

    /// Get the number of points in the path
    pub fn get_num_points(&self) -> usize {
        self.points_m.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points_m.len() == 0
    }

    // /// Get the intersection points of self and other as a list of `NavPose`s.
    // pub fn intersect(&self, other: &Self) -> Vec<NavPose> {
    //     // Based on https://www.youtube.com/watch?v=I9EsN2DTnN8 and https://en.wikipedia.org/wiki/Bentley%E2%80%93Ottmann_algorithm

    //     // Create a list of all points in both self and other, inserting new segments by joining
    //     // points in each path
    //     let segments: Vec<Segment> = self
    //         .points_m
    //         .as_slice()
    //         .windows(2)
    //         .filter_map(|points| {
    //             Some(Segment {
    //                 start: Point2::new(
    //                     NotNan::new(points[0].x).ok()?,
    //                     NotNan::new(points[0].y).ok()?,
    //                 ),
    //                 end: Point2::new(
    //                     NotNan::new(points[1].x).ok()?,
    //                     NotNan::new(points[1].y).ok()?,
    //                 ),
    //                 in_self: true,
    //             })
    //         })
    //         .chain(other.points_m.as_slice().windows(2).filter_map(|points| {
    //             Some(Segment {
    //                 start: Point2::new(
    //                     NotNan::new(points[0].x).ok()?,
    //                     NotNan::new(points[0].y).ok()?,
    //                 ),
    //                 end: Point2::new(
    //                     NotNan::new(points[1].x).ok()?,
    //                     NotNan::new(points[1].y).ok()?,
    //                 ),
    //                 in_self: false,
    //             })
    //         }))
    //         .collect();

    //     #[derive(Clone, Copy, PartialEq, Eq)]
    //     struct Segment {
    //         start: Point2<NotNan<f64>>,
    //         end: Point2<NotNan<f64>>,
    //         in_self: bool,
    //     }

    //     #[derive(Clone, Copy, PartialEq, Eq)]
    //     enum Event<'a> {
    //         Start(&'a Segment),
    //         End(&'a Segment),
    //         Intersect(&'a Point2<NotNan<f64>>),
    //     }

    //     impl<'a> PartialOrd for Event<'a> {
    //         fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
    //             let self_x = match self {
    //                 Event::Start(s) => &s.start.x,
    //                 Event::End(e) => &e.end.x,
    //                 Event::Intersect(p) => &p.x,
    //             };
    //             let other_x = match other {
    //                 Event::Start(s) => &s.start.x,
    //                 Event::End(e) => &e.end.x,
    //                 Event::Intersect(p) => &p.x,
    //             };

    //             self_x.partial_cmp(other_x)
    //         }
    //     }
    //     impl<'a> Ord for Event<'a> {
    //         fn cmp(&self, other: &Self) -> std::cmp::Ordering {
    //             let self_x = match self {
    //                 Event::Start(s) => &s.start.x,
    //                 Event::End(e) => &e.end.x,
    //                 Event::Intersect(p) => &p.x,
    //             };
    //             let other_x = match other {
    //                 Event::Start(s) => &s.start.x,
    //                 Event::End(e) => &e.end.x,
    //                 Event::Intersect(p) => &p.x,
    //             };

    //             self_x.cmp(other_x)
    //         }
    //     }

    //     #[derive(Clone, Copy, PartialEq, Eq)]
    //     enum Cross<'a> {
    //         Start(&'a Segment),
    //         End(&'a Segment),
    //         Intersect(&'a Point2<NotNan<f64>>),
    //     }

    //     impl<'a> PartialOrd for Cross<'a> {
    //         fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
    //             let self_y = match self {
    //                 Cross::Start(s) => &s.start.y,
    //                 Cross::End(e) => &e.end.y,
    //                 Cross::Intersect(p) => &p.y,
    //             };
    //             let other_y = match other {
    //                 Cross::Start(s) => &s.start.y,
    //                 Cross::End(e) => &e.end.y,
    //                 Cross::Intersect(p) => &p.y,
    //             };

    //             self_y.partial_cmp(other_y)
    //         }
    //     }
    //     impl<'a> Ord for Cross<'a> {
    //         fn cmp(&self, other: &Self) -> std::cmp::Ordering {
    //             let self_y = match self {
    //                 Cross::Start(s) => &s.start.y,
    //                 Cross::End(e) => &e.end.y,
    //                 Cross::Intersect(p) => &p.y,
    //             };
    //             let other_y = match other {
    //                 Cross::Start(s) => &s.start.y,
    //                 Cross::End(e) => &e.end.y,
    //                 Cross::Intersect(p) => &p.y,
    //             };

    //             self_y.cmp(other_y)
    //         }
    //     }

    //     // Initialise the event queue by inserting all start and end events for the segments into it
    //     let mut events = BTreeSet::new();
    //     for seg in segments.iter() {
    //         // Put start into the set (will not update if it's already there)
    //         let start = Event::Start(seg);
    //         events.insert(start);

    //         // End
    //         let end = Event::End(seg);
    //         events.insert(end);
    //     }

    //     // Create empty status tree
    //     let mut status = BinaryHeap::new();

    //     // Empty intersections list
    //     let mut intersects = Vec::new();

    //     // Main loop
    //     while !events.is_empty() {
    //         if let Some(event) = events.iter().next() {
    //             match event {
    //                 Event::Start(seg) => {
    //                     // Insert a new crossing into the status heap
    //                     status.push(Cross::Start(seg));

    //                     // Get the neighbours of this crossing
    //                     let lower = status.
    //                 }
    //                 Event::End(_) => todo!(),
    //                 Event::Intersect(_) => todo!(),
    //             }
    //         }
    //     }

    //     intersects
    // }
}

impl AckSequence {
    /// Convert this sequence into a standard [`Path`].
    ///
    /// If the sequence is empty `None` is returned.
    pub fn into_path(self, start_pose: &Pose) -> Result<Path, PathError> {
        // If sequence is empty just return None
        if self.seq.is_empty() {
            return Err(PathError::EmptySequence);
        }

        // Create new empty path
        let mut path = Path::new_empty();

        // For each Ack in the sequence calculate the points at the given separation
        for (curv_m, dist_m) in self.seq {
            // If this is the first sequence the start point and heading come from the starting
            // pose, otherwise they come from the current end of the path
            let (x_0, y_0, head_rad) = match path.is_empty() {
                true => (
                    start_pose.position_m[0],
                    start_pose.position_m[1],
                    start_pose.attitude_q.euler_angles().2,
                ),
                false => {
                    let last = path.points_m.last().unwrap();
                    (
                        last[0],
                        last[1],
                        path.get_segment_to_target(path.get_num_points() - 1)
                            .unwrap()
                            .heading_rad,
                    )
                }
            };

            // Precalculate the anti-heading (head + pi/2)
            let anti_head_rad = head_rad + std::f64::consts::PI / 2.0;

            // Get a linearly spaced vector of s values (dist along arc of the ackermann)
            let num_s = (dist_m / self.point_sep_m).ceil() as usize;
            let mut s_values_m = Vec::with_capacity(num_s);
            for i in 0..num_s {
                s_values_m.push((i as f64 * self.point_sep_m).min(dist_m));
            }

            // Iterate over values of s, calculating a new point for the path for each one
            for s_m in s_values_m {
                // If the curvature is approximately zero we should move in a straight line, not in
                // a curv (the 1/curv_m below will mess this up massively).
                if curv_m.abs() <= std::f64::EPSILON {
                    path.points_m.push(Vector2::new(
                        x_0 + s_m * head_rad.cos(),
                        y_0 + s_m * head_rad.sin(),
                    ))
                } else {
                    // Calculate the -sk - head value
                    let sk_phi = -s_m * curv_m - head_rad;

                    // Move in an arc about the centre of rotation (which is 1/curv away along the
                    // anti_heading direction (note -ve curv will put this on the other side)).
                    path.points_m.push(Vector2::new(
                        x_0 + 1.0 / curv_m * (anti_head_rad.cos() - sk_phi.sin()),
                        y_0 + 1.0 / curv_m * (anti_head_rad.sin() - sk_phi.cos()),
                    ));
                }
            }
        }

        // Check for points too close together, i.e ones that may come between the start and end of
        // different segments. Delete those
        let mut points_to_delete = Vec::new();
        for i in 1..path.points_m.len() {
            // Get the segment to this target point
            let seg = match path.get_segment_to_target(i) {
                Some(s) => s,
                None => continue,
            };

            // If the length of the segment is less than half the separation add it to the list of
            // points to delete
            if seg.length_m < 0.5 * self.point_sep_m {
                points_to_delete.push(i);
            }
        }

        // Remove points, making sure to decrement the indices every time we remove a point
        for (points_deleted, point_to_delete) in points_to_delete.iter().enumerate() {
            path.points_m.remove(point_to_delete - points_deleted);
        }

        Ok(path)
    }

    pub fn from_path_spec(path_spec: PathSpec) -> Result<Self, PathError> {
        match path_spec {
            PathSpec::AckSeq { seq, separation_m } => {
                if seq.len() % 2 != 0 {
                    Err(PathError::InvalidPathSpec)
                } else {
                    Ok(Self {
                        seq: seq.chunks(2).map(|p| (p[0], p[1])).collect(),
                        point_sep_m: separation_m,
                    })
                }
            }
            _ => Err(PathError::UnexpectedPathSpecType),
        }
    }
}

#[cfg(test)]
mod test {
    use nalgebra::{UnitQuaternion, Vector3};
    use std::f64::consts::PI;

    use super::*;

    #[test]
    fn test_ack_seq() {
        let ack_seq = AckSequence {
            seq: vec![(1.0, 1.57), (0.5, 2.0), (0.0, 1.0)],
            point_sep_m: 0.05,
        };

        // Convert ack_seq to a path
        let path = ack_seq
            .into_path(&Pose {
                position_m: Vector3::default(),
                attitude_q: UnitQuaternion::from_euler_angles(0.0, 0.0, PI),
            })
            .unwrap();

        // Serialize the path and write it out so we can plot it to check it's correct
        let path_json = serde_json::to_string_pretty(&path).unwrap();
        std::fs::write("test_path.json", path_json).unwrap();
    }
}
