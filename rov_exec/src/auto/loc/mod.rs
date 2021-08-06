//! # Localisation module
//!
//! This module provides localisation for the rover in the form of visual
//! odometry. This module is currently a stub.

// ---------------------------------------------------------------------------
// MODULES
// ---------------------------------------------------------------------------

mod params;
pub use params::LocMgrParams;

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use nalgebra::{Affine3, Isometry3, Translation3, UnitQuaternion, Vector2, Vector3};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// The current pose (position and attitude) of the rover in some parent frame.
///
/// More specifically this represents the Rover Body (RB) frame in the parent frame, for example
/// the Global Map (GM) frame.
#[derive(Debug, Copy, Clone)]
pub struct Pose {
    /// The position in the parent frame
    ///
    /// Note: While this should strictly be a Point3, this wasn't used when it was first created,
    /// so for backwards compatibility this is staying as a Vector3.
    pub position_m: Vector3<f64>,

    /// The attitude of the rover in the parent frame. This is a quaternion that
    /// will rotate an object from the parent frame into the RB frame.
    pub attitude_q: UnitQuaternion<f64>,

    /// Affine transform that will transform a point relative to this pose to the pose's parent.
    pub pose_to_parent: Affine3<f64>,
}

/// A shallow copy of Pose without the affine, so that existing serde serialize/deserialize doesn't
/// break
#[derive(Serialize, Deserialize)]
struct SerdePose {
    position_m: Vector3<f64>,
    attitude_q: UnitQuaternion<f64>,
}

/// Provides an interface for the Localisation system of the rover.
#[derive(Clone)]
pub struct LocMgr {
    source: LocSource,

    pose: Option<Pose>,
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, Copy, Clone, Deserialize)]
pub enum LocSource {
    OnSet,
    SimClient,
    PerlocClient,
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Pose {
    pub fn new(position_m: Vector3<f64>, attitude_q: UnitQuaternion<f64>) -> Self {
        Self {
            pose_to_parent: Affine3::from_matrix_unchecked(
                Isometry3::from_parts(Translation3::from(position_m), attitude_q).to_matrix(),
            ),
            position_m,
            attitude_q,
        }
    }

    /// Return the heading (angle to the positive LM_X axis) of the rover in radians.
    ///
    /// Heading is given in the range [0, 2*pi], with 0 being in the LM_X direction.
    pub fn get_heading(&self) -> f64 {
        // TODO: verify this
        self.attitude_q.euler_angles().2
    }

    /// Get the 3D unit vector in the forward direction of the rover.
    pub fn forward3(&self) -> Vector3<f64> {
        self.attitude_q * Vector3::new(1.0, 0.0, 0.0)
    }

    /// Get the 2D unit vector in the forward direction of the rover.
    pub fn forward2(&self) -> Vector2<f64> {
        let f3 = self.forward3();
        let f2 = Vector2::new(f3[0], f3[1]);
        f2 / f2.norm()
    }

    /// Get the 2D position vector of the rover
    pub fn position2(&self) -> Vector2<f64> {
        Vector2::new(self.position_m.x, self.position_m.y)
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            position_m: Vector3::zeros(),
            attitude_q: UnitQuaternion::identity(),
            pose_to_parent: Affine3::identity(),
        }
    }
}

impl Serialize for Pose {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let p = SerdePose {
            position_m: self.position_m,
            attitude_q: self.attitude_q,
        };

        p.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Pose {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let p = SerdePose::deserialize(deserializer)?;

        Ok(Pose::new(p.position_m, p.attitude_q))
    }
}

impl LocMgr {
    pub fn new(source: LocSource) -> Self {
        Self { source, pose: None }
    }

    pub fn set_pose(&mut self, pose: Pose) {
        self.pose = Some(pose);
    }

    pub fn get_pose(&mut self) -> Option<Pose> {
        match self.source {
            LocSource::OnSet => self.pose,
            LocSource::SimClient => crate::sim_client::rov_pose_lm(),
            _ => unimplemented!(),
        }
    }

    // TODO: make instanced
    pub fn _get_pose() -> Option<Pose> {
        crate::sim_client::rov_pose_lm()
    }
}
