//! # AutoMgr Parameters

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::Deserialize;
use crate::auto::{loc::LocMgrParams, map::{CostMapParams, TerrainMapParams}};

use super::{stop::StopParams, wait_new_pose::WaitNewPoseParams, auto_mnvr::AutoMnvrParams};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Deserialize)]
pub struct AutoMgrParams {
    pub wait_new_pose: WaitNewPoseParams,

    pub terrain_map_params: TerrainMapParams,

    pub cost_map_params: CostMapParams,

    pub stop: StopParams,

    pub loc_mgr: LocMgrParams,

    pub auto_mnvr: AutoMnvrParams
}