//! # [`AutoMgrState::WaitNewPose`] implementation



// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use crate::auto::AutoMgrError;
use super::{StepOutput, params::AutoMgrParams};
use serde::Deserialize;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// WaitNewPose state.
///
/// This state is designed to wait for the LocMgr to be able to provide a new pose. It should be
/// used if the [`LocMgr::get_pose()`] function returns `None`. It will wait up to the provided time
/// duration before aborting.
#[derive(Debug)]
pub struct WaitNewPose {
    /// Maximum duration to wait before aborting
    max_wait_time_s: f64
}

#[derive(Debug, Clone, Deserialize)]
pub struct WaitNewPoseParams {
    /// Maximum duration to wait before aborting
    pub max_wait_time_s: f64
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl WaitNewPose {

    pub fn new(params: &AutoMgrParams) -> Self {
        Self {
            max_wait_time_s: params.wait_new_pose.max_wait_time_s
        }
    }

    pub fn step(

    ) -> Result<StepOutput, AutoMgrError> {
        todo!()
    }
}