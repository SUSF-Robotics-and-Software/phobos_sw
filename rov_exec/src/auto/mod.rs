//! # Autonomy Module
//!
//! This module provides high level autonomy for the rover, allowing it to perform actions such as
//! traverse by itself, under instruction from ground control.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

// use comms_if::tc::{
//     auto::AutoCmd,
//     loco_ctrl::MnvrCmd,
// };
// use log::{info, warn};
// use nav::NavCtrl;

// use self::{loc::LocMgr, nav::{NavCtrlType, NavError}};

pub use auto_mgr::AutoMgr;

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

/// Automation Manager module
pub mod auto_mgr;

/// Localisation module - provides the rover with an idea of where it is in the world
pub mod loc;

/// Trajectory control module - keeps the rover on the given path
pub mod traj_ctrl;

/// Defines path types
pub mod path;

/// Navigation module - provides path planning using cost maps
pub mod nav;

/// Map module - provides implementations for terrain and cost maps
pub mod map;

/// Perception module - converts from depth images to terrain maps
pub mod per;
