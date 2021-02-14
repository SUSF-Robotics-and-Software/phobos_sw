//! # AutoMgr Parameters

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::Deserialize;
use crate::auto::map::TerrainMapParams;

use super::{stop::StopParams, wait_new_pose::WaitNewPoseParams};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Deserialize)]
pub struct AutoMgrParams {
    pub wait_new_pose: WaitNewPoseParams,

    pub terrain_map_params: TerrainMapParams,

    pub stop: StopParams
}