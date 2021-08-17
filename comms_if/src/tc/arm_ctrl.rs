//! # Arm control telecommands

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use crate::eqpt::mech::MechDems;
use serde::{Deserialize, Serialize};
use structopt::StructOpt;

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// A rotational command that can be completed by arm control.
#[derive(Debug, Clone, Serialize, Deserialize, StructOpt)]
pub enum ArmCmd {
    /// A generic rotational command.
    ///
    /// A rotational command will rotate each joint of
    /// the arm to the desired angular position.
    #[structopt(name = "rot")]
    BasicRotation {
        #[structopt(skip)]
        dems: MechDems,
    },

    /// A simplified control where the user defines the location of the head
    /// and the positions of the motors are calculated to achieve this position.
    #[structopt(name = "ik")]
    InverseKinematics {
        /// Horizontal distance from the base of the arm.
        ///
        /// Positive is away from the rover body which is in the
        /// -y axis in the frame of the rover.
        horizontal_distance_m: f64,

        /// Vertical distance from the base of the arm.
        ///
        /// Positive is away from the rover body which is in the
        /// z+ axis in the frame of the rover.
        vertical_distance_m: f64,

        /// Speed of the rover's head to the target position.
        ///
        /// Can only be positive.
        speed_ms: f64,
    },

    /// Stop the arm, maintaining the current axis angles but setting
    /// all angular velocities to zero.
    #[structopt(name = "stop")]
    Stop,
}
