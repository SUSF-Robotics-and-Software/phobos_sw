//! # Check AutoMgr Mode

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use super::{AutoCmd, AutoMgr, AutoMgrError, nav::{CheckNavCtrl, NavCtrlType}};

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl AutoMgr {
    pub(super) fn mode_check(&mut self) -> Result<(), AutoMgrError> {
        unimplemented!();
        
        // // If the check controller isn't set initialise it
        // if self.nav_ctrl.is_none() {
        //     self.nav_ctrl = Some(NavCtrlType::Check(CheckNavCtrl::new()?));
        // }

        // // Check the controller is of the right type
        // match self.nav_ctrl {
        //     Some(NavCtrlType::Check(_)) => (),
        //     _ => return Err(AutoMgrError::UnexpectedNavCtrlType)
        // }

        // Ok(())
    }
}