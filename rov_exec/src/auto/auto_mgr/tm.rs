//! Defines Telemetry Pack for Autonomy

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use serde::{Deserialize, Serialize};

use crate::auto::{loc::Pose, map::CostMap, path::Path, traj_ctrl::StatusReport};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AutoTm {
    pub path: Option<Path>,
    pub pose: Option<Pose>,
    pub traj_ctrl_status: Option<StatusReport>,
    pub local_cost_map: Option<CostMap>,
}
