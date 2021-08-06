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
    pub wait_new_pose: WaitNewPoseParams,

    pub terrain_map_params: CellMapParams,

    pub cost_map_params: CostMapParams,

    pub stop: StopParams,

    pub loc_mgr: LocMgrParams,

    pub auto_mnvr: AutoMnvrParams,

    pub path_planner: PathPlannerParams,

    pub per_mgr: PerMgrParams,
}
