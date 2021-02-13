//! # Navigation
//!
//! This module provides high level navigation control of the rover.
//!
//! Three different Navigation controllers are implemented for the three main autonomous modes:
//! - [`FollowNavCtrl`] - Implements the `Follow` mode, where a ground-plan path is followed by the
//!   rover without any analysis being performed.
//! - [`CheckNavCtrl`] - Implements the `Check` mode, where a ground-planned path is loosely 
//!   followed by the rover, with the rover attempting to avoid obstacles on the path.
//! - [`GotoNavCtrl`] - Implements the `Goto` mode, in which the rover attempts to navigate itself
//!   towards the target position with no prior ground-planned path.
//!

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

pub mod check_nav_ctrl;
pub mod follow_nav_ctrl;
pub mod goto_nav_ctrl;

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use crate::auto::{
    map::TerrainMap,
    loc::Pose,
    AutoMgrMode
};

pub use check_nav_ctrl::CheckNavCtrl;

use super::{AutoMgrError, map::GridMapError};

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum NavError {
    #[error("Attempted to update a cost map when there is no valid terrain map")]
    NoTerrainMap,

    #[error("An error occured while calculating cost map from terrain map")]
    CostMapCalcFailed(GridMapError)
}

#[derive(Clone)]
pub enum NavCtrlType {
    Check(CheckNavCtrl),
}

// ------------------------------------------------------------------------------------------------
// TRAITS
// ------------------------------------------------------------------------------------------------

pub trait NavCtrl {
    /// Step the controller's logic, and return an optional error
    fn step(&mut self) -> Result<(), NavError>;

    /// Set the controller's terrain map for use in the next [`step`].
    fn set_terrain_map(&mut self, terrain_map: &TerrainMap);

    /// Set the pose the controller will use for it's next [`step`].
    fn set_pose(&mut self, pose: &Pose);

    /// Get the AutoMgrMode requested by the controller.
    /// 
    /// If None no mode change is requested. Otherwise AutoMgr should change into the requested
    /// mode. 
    fn requested_auto_mgr_mode(&self) -> Option<AutoMgrMode>;
}

impl From<NavError> for AutoMgrError {
    fn from(nav_error: NavError) -> Self {
        Self::NavError(nav_error)
    }
}

impl NavCtrl for NavCtrlType {
    fn step(&mut self) -> Result<(), NavError> {
        match self {
            NavCtrlType::Check(ref mut c) => c.step()
        }
    }

    fn set_terrain_map(&mut self, terrain_map: &TerrainMap) {
        match self {
            NavCtrlType::Check(ref mut c) => c.set_terrain_map(terrain_map)
        }
    }

    fn set_pose(&mut self, pose: &Pose) {
        match self {
            NavCtrlType::Check(ref mut c) => c.set_pose(pose)
        }
    }

    fn requested_auto_mgr_mode(&self) -> Option<AutoMgrMode> {
        match self {
            NavCtrlType::Check(ref c) => c.requested_auto_mgr_mode()
        }
    }
}