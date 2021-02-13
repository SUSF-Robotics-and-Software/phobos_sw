//! # Check Mode Navigation Controller

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use crate::auto::{AutoMgrMode, loc::Pose, map::{CostMap, TerrainMap}};

use super::{NavCtrl, NavError};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Clone)]
pub struct CheckNavCtrl {
    /// Terrain map to convert to cost map
    terrain_map: Option<TerrainMap>,

    /// True if the terrain map has been updated between the current and previous step.
    terrain_map_update: bool,

    /// Cost map to plan path through
    cost_map: Option<CostMap>,

    /// Current estimated pose of the rover
    pose: Option<Pose>,

    /// Requested mode for AutoMgr
    requested_auto_mgr_mode: Option<AutoMgrMode>
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl CheckNavCtrl {
    pub fn new() -> Result<Self, NavError> {
        todo!()
    }
}

impl NavCtrl for CheckNavCtrl {
    fn step(&mut self) -> Result<(), NavError> {
        // If there's an update to the terrain map recalculate the cost map
        if self.terrain_map_update {
            match self.terrain_map {
                Some(ref t) => 
                    self.cost_map = Some(CostMap::calculate(t)
                        .map_err(|e| NavError::CostMapCalcFailed(e))?
                    ),
                None => return Err(NavError::NoTerrainMap)
            }
        }

        Ok(())
    }

    fn set_terrain_map(&mut self, terrain_map: &TerrainMap) {
        self.terrain_map = Some(terrain_map.clone());
        self.terrain_map_update = true;
    }

    fn set_pose(&mut self, pose: &Pose) {
        self.pose = Some(pose.clone())
    }

    fn requested_auto_mgr_mode(&self) -> Option<AutoMgrMode> {
        self.requested_auto_mgr_mode
    }
}