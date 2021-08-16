//! # Locomotion control telecommands

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::{Serialize, Deserialize};
use structopt::StructOpt;

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// A manouvre that can be completed by locomotion control.
#[derive(Debug, Copy, Clone, Serialize, Deserialize, StructOpt)]
pub enum MnvrCmd {
    /// A generic ackerman command.
    ///
    /// An ackerman manouvre will drive the rover in a coordinated maner in a circle around a centre
    /// of rotation. The location of the CoR is defined by the curvature of the turn and the crab
    /// angle.
    #[structopt(name = "ack")]
    Ackerman {
        /// The speed of the manouvre in meters/second.
        ///
        /// Positive speeds are "forwards", negative speeds are "backwards"
        speed_ms: f64,

        /// The curvature of the manouvre in 1/meters.
        ///
        /// Follows the right hand rule about the rover's Z+ (upwards) axis, so that positive
        /// curvature is a turn to the left, and negative curvature a turn to the right.
        curv_m: f64,

        /// The crab angle of the manouvre in radians.
        ///
        /// Follows the right hand grip rule about the rover's Z+ (upwards) axis, so that positive
        /// crab angles will move to the left, and negative crab angle to the right.
        crab_rad: f64
    },

    /// A turn-on-the-spot manouvre about the centre of the rover's wheelbase.
    #[structopt(name = "pt")]
    PointTurn {
        /// The turn rate of the manouvre in radians/second.
        ///
        /// Follows the right hand rule about the rover's Z+ (upwrads) axis, so that a positive turn
        /// rate will rotate the rover to the left, and a negative turn rate will rotate the rover
        /// to the right.
        rate_rads: f64
    },

    /// A tank-like steering manouvre in which all wheels point forwards and the rover is steered
    /// using differential speeds on the left and right wheels.
    #[structopt(name = "skid")]
    SkidSteer {
        /// The speed of the manouvre in meters/second.
        ///
        /// Positive speeds are "forwards", negative speeds are "backwards"
        speed_ms: f64,

        /// The curvature of the manouvre in 1/meters.
        ///
        /// Follows the right hand rule about the rover's Z+ (upwards) axis, so that positive
        /// curvature is a turn to the left, and negative curvature a turn to the right.
        curv_m: f64,
    },

    /// Stop the rover, maintaining the current steer axis angles but setting all drive axes to zero
    /// speed.
    #[structopt(name = "stop")]
    Stop
}