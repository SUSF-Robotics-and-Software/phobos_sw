//! # Defines Telemetry Pack for Autonomy

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use serde::{Serialize, Deserialize};

use crate::auto::{loc::Pose, path::Path, traj_ctrl::StatusReport};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AutoTm {
    pub path: Option<Path>,
    pub pose: Option<Pose>,
    pub traj_ctrl_status: Option<StatusReport>
}