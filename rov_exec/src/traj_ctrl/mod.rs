//! # Trajectory control module
//!
//! Trajectory control is responsible for keeping the rover on the target path.
//! It does this using a pair of PID controllers operating on the lateral error
//! and heading error respectively. These concepts are explained later.
//!
//! The path itself is made up of a number of points, separated by about 5 cm.
//! These points exist on the XY plane of the Local Map coordinate frame, and
//! each point is joined in sequence. A path segment is defined as the line 
//! connecting two neighbouring points.
//! 
//! The lateral error is the distance between the rover's current location and
//! the path segment, i.e. how far off the segment we are. The heading error
//! is the difference between the rover's heading and the heading of the 
//! segment. The controllers attempt to minimise these errors by outputing 
//! curvature demands which are then summed and saturated. Speed demands are 
//! calculated based off of the curvature demand, the tighter the turn, the 
//! slower the desired speed.

// ---------------------------------------------------------------------------
// MODULES
// ---------------------------------------------------------------------------

pub mod controllers;
pub mod params;
pub mod path;
pub mod state;

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// Internal
pub use path::*;
pub use controllers::*;
pub use params::Params;
pub use state::*;
