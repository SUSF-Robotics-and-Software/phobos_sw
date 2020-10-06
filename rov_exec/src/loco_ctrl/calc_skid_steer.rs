//! Skid steer calculations

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// Internal imports
use super::LocoCtrl;

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl LocoCtrl {

    /// Perform the skid steer command calculations
    pub(crate) fn calc_skid_steer(
        &mut self,
        _speed_ms: f64,
        _curv_m: f64
    ) -> Result<(), super::LocoCtrlError> {
        Err(super::LocoCtrlError::NotYetSupported(String::from(
            "Manouvre command 'Skid Steer' is not yet supported")))
    }
}