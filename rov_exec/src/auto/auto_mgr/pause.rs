//! # [`AutoMgr<Pause>`] implementation

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use comms_if::tc::auto::AutoCmd;

use super::{AutoMgrState, states::*};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

pub struct Pause<S> {
    from_state: S
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// TRANSITIONS FROM PAUSE
// ------------------------------------------------------------------------------------------------

impl<S> From<AutoMgrState<Pause<S>>> for AutoMgrState<Off> {
    /// Pause may alwasys move to Off via an Abort TC.
    fn from(previous: AutoMgrState<Pause<S>>) -> Self {
        Self {
            persistant: previous.persistant,
            state: Off
        }
    }
}

impl From<AutoMgrState<Pause<AutoMnvr>>> for AutoMgrState<AutoMnvr> {
    fn from(previous: AutoMgrState<Pause<AutoMnvr>>) -> Self {
        Self {
            persistant: previous.persistant,
            state: previous.state.from_state
        }
    }
}
