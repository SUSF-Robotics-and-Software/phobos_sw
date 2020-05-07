//! # Localisation module
//!
//! This module provides localisation for the rover in the form of visual 
//! odometry. This module is currently a stub.

// ---------------------------------------------------------------------------
// MODULES
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// The current pose (position and attitude in the LM frame) of the rover.
///
/// More specifically this represents the Rover Body (RB) frame in the Local
/// Map (LM) frame.
pub struct Pose {

    /// The position in the LM frame
    pub position_m_lm: [f64; 3],

    /// The attitude of the rover in the LM frame. This is a quaternion that 
    /// will rotate an object from the LM frame into the RB frame.
    pub attitude_q_lm: [f64; 4]
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Pose {

    /// Return the heading (angle to the positive LM_X axis) of the rover in
    /// radians.
    pub fn get_heading(&self) -> f64 {
        2f64 * self.attitude_q_lm[3].acos()
    }
}