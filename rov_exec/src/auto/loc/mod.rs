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

use serde::Deserialize;
use nalgebra::{Vector3, UnitQuaternion};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// The current pose (position and attitude in the LM frame) of the rover.
///
/// More specifically this represents the Rover Body (RB) frame in the Local
/// Map (LM) frame.
#[derive(Debug, Copy, Clone, Deserialize)]
pub struct Pose {

    /// The position in the LM frame
    pub position_m_lm: Vector3<f64>,

    /// The attitude of the rover in the LM frame. This is a quaternion that 
    /// will rotate an object from the LM frame into the RB frame.
    pub attitude_q_lm: UnitQuaternion<f64>
}

/// Provides an interface for the Localisation system of the rover.
#[derive(Clone)]
pub struct LocMgr;

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Pose {

    /// Return the heading (angle to the positive LM_X axis) of the rover in radians.
    ///
    /// Heading is given in the range [0, 2*pi], with 0 being in the LM_X direction.
    pub fn get_heading(&self) -> f64 {
        // TODO: verify this
        self.attitude_q_lm.euler_angles().2
    }
}

impl LocMgr {
    pub fn new() -> Self {
        Self {}
    }

    pub fn get_pose(&mut self) -> Option<Pose> {
        crate::sim_client::rov_pose_lm()
    }

    // TODO: make instanced
    pub fn _get_pose() -> Option<Pose> {
        crate::sim_client::rov_pose_lm()
    }
}