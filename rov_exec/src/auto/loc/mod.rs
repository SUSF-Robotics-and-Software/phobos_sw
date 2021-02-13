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
#[derive(Debug, Copy, Clone)]
pub struct Pose {

    /// The position in the LM frame
    pub position_m_lm: [f64; 3],

    /// The attitude of the rover in the LM frame. This is a quaternion that 
    /// will rotate an object from the LM frame into the RB frame.
    pub attitude_q_lm: [f64; 4]
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
        // // Normalize the quaternion so it only has value in the y axis.
        // let mut q = self.attitude_q_lm;
        // q[0] = 0.0;
        // q[2] = 0.0;
        // let mag = util::maths::norm(&q, &[0.0; 4]).unwrap();
        // q[1] /= mag;
        // q[3] /= mag;

        // 2f64 * self.attitude_q_lm[3].acos()
        util::maths::map_pi_to_2pi(self.attitude_q_lm[1].atan2(self.attitude_q_lm[3])*2.0)
    }
}

impl LocMgr {
    pub fn get_pose() -> Option<Pose> {
        crate::sim_client::rov_pose_lm()
    }
}