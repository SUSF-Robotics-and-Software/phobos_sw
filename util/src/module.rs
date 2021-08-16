//! Module interfaces
//!
//! Each module in `rov_exec` and `gnd_exec` shall implement all the items
//! in this module.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// Internal imports
use crate::session::Session;

// ---------------------------------------------------------------------------
// MODULE STATE
// ---------------------------------------------------------------------------

/// The module's internal state.
pub trait State {
    /// Data required during initialisation
    type InitData;
    /// An error which can occur during initialisation.
    type InitError;

    /// Data required for cyclic processing.
    type InputData;
    /// Data produced by cyclic processing.
    type OutputData;
    /// A report on the status of the cyclic processing.
    type StatusReport;
    /// An error which can occur during cyclic processing.
    type ProcError;

    /// Initialise the module.
    ///
    /// # Inputs
    /// - `init_data`: The input data required by the
    ///
    /// # Outputs
    /// - On success `Ok(())`.
    /// - On error an `InitError` instance.
    fn init(&mut self, init_data: Self::InitData, session: &Session)
        -> Result<(), Self::InitError>;

    /// Main module processing function.
    ///
    /// # Inputs
    /// - `input_data`: The data required for processing by the module.
    ///
    /// # Outputs
    /// - On success a tuple of the output data and status report.
    /// - On error a `ProcError` instance.
    fn proc(&mut self, input_data: &Self::InputData)
        -> Result<(Self::OutputData, Self::StatusReport), Self::ProcError>;
}