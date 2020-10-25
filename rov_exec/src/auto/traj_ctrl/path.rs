//! # Trajectory control path
//!
//! This module defines the path used by trajectory control.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use serde::{Serialize, Deserialize};

// Internal
use util::maths::norm;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A path defining the desired trajectory of the rover.
#[derive(Clone, Serialize, Deserialize)]
pub struct Path {
    points_m_lm: Vec<[f64; 2]>
}

/// A segment between two path points
#[derive(Default, Serialize, Deserialize)]
pub struct PathSegment {

    /// The target of the segment
    pub target_m_lm: [f64; 2],

    /// The start point of the segment
    pub start_m_lm: [f64; 2],

    /// The length of the segment
    pub length_m: f64,

    /// The slope (dy/dx) of the segment
    pub slope_m: f64,

    /// The intercept (the c in y = mx + c) of the segment
    pub intercept_m: f64
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Path {
    /// Create a new empty path
    pub fn new_empty() -> Self {
        Path {
            points_m_lm: vec![]
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
        seg.length_m = norm(
            &seg.target_m_lm, 
            &seg.start_m_lm)
            .unwrap();

        // Slope is the change in y over the change in x
        seg.slope_m = 
            (seg.target_m_lm[1] - seg.start_m_lm[1])
            / (seg.target_m_lm[0] - seg.start_m_lm[0]);

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
}