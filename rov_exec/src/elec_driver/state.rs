//! # Electronics driver module state

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use pyo3::Python;

// Internal
use util::{
    module::State,
    session::Session,
    params
};
use crate::loco_ctrl;
use super::Params;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

pub struct ElecDriver<'p> {
    params: Params,

    py: Option<Python<'p>>
}

pub struct InputData {
    pub safe_mode: bool,

    pub loco_ctrl_output: loco_ctrl::OutputData
}

pub struct StatusReport {

}

// ---------------------------------------------------------------------------
// ENUEMRATIONS
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub enum InitError {
    ParamLoadError(params::LoadError)
}

pub enum ProcError {
    NotImplemented
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl<'p> State for ElecDriver<'p> {
    type InitData = &'static str;
    type InitError = InitError;

    type InputData = InputData;
    type OutputData = ();
    type StatusReport = StatusReport;
    type ProcError = ProcError;

    /// Initialise the electronics driver.
    ///
    /// Expected init data is the path to the module parameters file.
    fn init(
        &mut self, 
        init_data: Self::InitData, 
        _session: &Session
    ) -> Result<(), Self::InitError> {
        // Load parameters
        self.params = match params::load(init_data) {
            Ok(p) => p,
            Err(e) => return Err(InitError::ParamLoadError(e))
        };

        Ok(())
    }

    /// Cyclic processing for electronics driver.
    ///
    /// Takes the output data from LocoCtrl and sends the demands to the motors.
    ///
    /// # Notes
    /// - If `safe_mode` is true the outputs will be halted so that no driving
    ///   is performed.
    fn proc(
        &mut self, 
        input_data: &Self::InputData
    ) -> Result<(Self::OutputData, Self::StatusReport), Self::ProcError> {


        Err(ProcError::NotImplemented)
    }
}