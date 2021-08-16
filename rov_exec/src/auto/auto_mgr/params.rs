//! # AutoMgr Parameters

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use crate::auto::{
    loc::LocMgrParams, map::CostMapParams, nav::path_planner::PathPlannerParams, per::PerMgrParams,
};
use cell_map::CellMapParams;
use serde::Deserialize;

use super::{auto_mnvr::AutoMnvrParams, stop::StopParams, wait_new_pose::WaitNewPoseParams};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Deserialize)]
pub struct AutoMgrParams {
    pub loc_mgr: LocMgrParams,

    pub wait_new_pose: WaitNewPoseParams,

    pub stop: StopParams,

    pub auto_mnvr: AutoMnvrParams,
}
