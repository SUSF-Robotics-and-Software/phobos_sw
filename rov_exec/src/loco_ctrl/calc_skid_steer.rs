//! Skid steer calculations

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External imports
use eyre::eyre;
use color_eyre::Report;

// Internal imports
use super::LocoCtrl;

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl LocoCtrl {

    /// Perform the skid steer command calculations
    pub(crate) fn calc_skid_steer(&mut self) -> Result<(), Report> {
        return Err(eyre!("Manouvre command 'Skid Steer' is not yet supported"))
    }
}