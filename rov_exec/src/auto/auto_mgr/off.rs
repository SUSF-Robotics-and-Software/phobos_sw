
// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use crate::auto::{AutoMgrError, map::TerrainMapParams};

use super::{AutoMgrState, AutoMgrPersistantData};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Off state of the AutoMgr.
///
/// Possible transitions, all triggered by TC:
/// - AutoMnvr,
/// - Follow,
/// - Check,
/// - Goto,
pub struct Off;

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl AutoMgrState<Off> {
    /// Create a new manager, which is initialised to [`Off`].
    pub fn new(terr_map_params: TerrainMapParams) -> Result<Self, AutoMgrError> {
        Ok(Self {
            persistant: AutoMgrPersistantData::new(terr_map_params)?,
            state: Off
        })
    }
}

// ------------------------------------------------------------------------------------------------
// TRANSITIONS FROM OFF
// ------------------------------------------------------------------------------------------------