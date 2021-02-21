//! # Defines Telemetry Pack for Autonomy

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use serde::{Serialize, Deserialize};

use crate::auto::{loc::Pose, path::Path};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AutoTm {
    pub path: Option<Path>,
    pub pose: Option<Pose>,
}