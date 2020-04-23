//! Locomotion control module

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use util::module::*;

// ---------------------------------------------------------------------------
// MODULES
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// MODULE IMPLEMENTATIONS
// ---------------------------------------------------------------------------

/// Locomotion control module state.
pub struct LocoCtrl {

}

impl State for LocoCtrl {
    type InitData = ();
    type InitError = ();
    
    type InputData = ();
    type OutputData = ();
    type StatusReport = ();
    type ProcError = ();

    /// Initialise the LocoCtrl module.
    fn init(init_data: Self::InitData) -> Result<(), Self::InitError> {
        // TODO
        Err(())
    }

    /// Perform cyclic processing of Locomotion Control.
    fn proc(input_data: Self::InputData)
        -> Result<(Self::OutputData, Self::StatusReport), Self::ProcError> 
    {
        // TODO
        Err(())
    }
    
}
