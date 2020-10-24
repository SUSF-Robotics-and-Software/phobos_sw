//! # Autonomy Telecommands

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::path::PathBuf;
use serde::{Serialize, Deserialize};
use structopt::StructOpt;

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// A command that can be performed by the Autonomy system.
#[derive(Debug, Clone, Serialize, Deserialize, StructOpt)]
pub enum AutoCmd {
    /// Follow a LocoCtrl style manouvre for a given distance.
    #[structopt(name = "mnvr")]
    Manouvre(AutoMnvrCmd),

    /// Follow the path stored in the path file at the given system path.
    #[structopt(name = "follow")]
    Follow {
        /// The path to the path file.
        path: PathBuf
    },

    /// Autonomously navigate to the given coordinates in the LocalMap frame.
    #[structopt(name = "goto")]
    Goto {
        /// The x-coordinate of the point to navigate to.
        x_m_lm: f64,

        /// The y-coordinate of the point to navigate to.
        y_m_lm: f64
    }
}

/// A command to perform an autonomous Locomotion Control manouvre.
#[derive(Debug, Copy, Clone, Serialize, Deserialize, StructOpt)]
pub enum AutoMnvrCmd {
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
        crab_rad: f64,

        /// The total distance to traverse in this manouvre.
        ///
        /// This is the complete distance the rover should traverse, along the arc of the manouvre.
        /// For a non-straight Ackerman this is equivalent to the length of the sector traced by the
        /// manouvre.
        dist_m: f64,
    },

    /// A turn-on-the-spot manouvre about the centre of the rover's wheelbase.
    #[structopt(name = "pt")]
    PointTurn {
        /// The turn rate of the manouvre in radians/second.
        ///
        /// Follows the right hand rule about the rover's Z+ (upwrads) axis, so that a positive turn
        /// rate will rotate the rover to the left, and a negative turn rate will rotate the rover
        /// to the right.
        rate_rads: f64,

        /// The absolute angular distance to traverse in this manouvre.
        dist_rad: f64
    },
}