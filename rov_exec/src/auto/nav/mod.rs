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
// IMPORTS
// ------------------------------------------------------------------------------------------------

use super::map::GridMapError;

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