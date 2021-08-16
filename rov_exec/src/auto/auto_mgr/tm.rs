//! Defines Telemetry Pack for Autonomy

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use serde::{Deserialize, Serialize};

use crate::auto::{loc::Pose, map::CostMap, nav::NavPose, path::Path, traj_ctrl::StatusReport};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AutoTm {
    pub path: Option<Path>,
    pub secondary_path: Option<Path>,
    pub pose: Option<Pose>,
    pub traj_ctrl_status: Option<StatusReport>,
    pub global_cost_map: Option<CostMap>,
    pub local_target: Option<NavPose>,
}
