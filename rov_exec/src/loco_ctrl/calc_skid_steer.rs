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
    pub(crate) fn calc_skid_steer(&mut self) -> Result<(), super::Error> {
        Err(super::Error::NotYetSupported(String::from(
            "Manouvre command 'Skid Steer' is not yet supported")))
    }
}