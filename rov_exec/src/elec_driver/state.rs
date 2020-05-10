//! # Electronics driver module state

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use log::{trace, error};
use pyo3::prelude::*;
#[allow(unused_imports)]
use pyo3::types::PyDict;
use thiserror::Error;

// Internal
use util::{
    module::State,
    session::Session,
    params,
    maths::poly_val
};
use crate::loco_ctrl::{self, AxisRate};
use super::{Params, ParamsError};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

#[allow(dead_code)]
#[derive(Default)]
pub struct ElecDriver {
    params: Params,
    report: StatusReport,

    gil: Option<GILGuard>,
    boards: Option<PyObject>,

    current_str_sk: [f64; loco_ctrl::NUM_STR_AXES],

    // We can't directly convert [[u64; _]; _] into a PyObject so we need to 
    // reset them as vectors.
    drv_idx_map: Vec<Vec<u64>>,
    str_idx_map: Vec<Vec<u64>>
}

#[derive(Default)]
pub struct InputData {
    pub safe_mode: bool,

    pub loco_ctrl_output: loco_ctrl::OutputData
}

#[derive(Default, Copy, Clone)]
pub struct StatusReport {

}

// ---------------------------------------------------------------------------
// ENUEMRATIONS
// ---------------------------------------------------------------------------

#[derive(Debug, Error)]
pub enum InitError {
    #[error("Failed to load parameters: {0}")]
    ParamLoadError(params::LoadError),

    #[error("Loaded parameters are invalid: {0}")]
    ParamsInvalid(ParamsError),

    #[error("Failed acquiring reference to the python GIL")]
    GilRefFailed,

    #[error("An error occured within the python interpreter")]
    PythonError,

    #[error("ServoKit not found")]
    ServoKitNotFound
}

#[derive(Debug, Error)]
pub enum ProcError {
    #[error("Absolute drive rates not yet supported")]
    AbsDrvRateUnsupported,

    #[error("Failed acquiring reference to the python GIL")]
    GilRefFailed,

    #[error("An error occured within the python interpreter")]
    PythonError,

    #[error("ServoKit not found")]
    ServoKitNotFound
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl State for ElecDriver {
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

        // Check parameters are valid
        match self.params.are_valid() {
            Ok(_) => (),
            Err(e) => return Err(InitError::ParamsInvalid(e))
        }

        // Python stuff should be excluded from non-rpi builds since the servo
        // kit won't actually work without it
        #[cfg(target_arch = "arm")]
        {

            // Get the GIL lock
            self.gil = Some(Python::acquire_gil());

            // Get a python instance
            // let py = match self.gil {
            //     Some(ref g) => g.python(),
            //     None => return Err(InitError::GilRefFailed)
            // };
            let py = gil.python();
            
            // Create the servokit instance
            let locals = PyDict::new(py);

            unwrap_py_init(py,
                locals.set_item("num_boards", self.params.num_boards))?;
            unwrap_py_init(py, 
                locals.set_item("num_channels", &self.params.num_channels))?;
            unwrap_py_init(py, 
                locals.set_item(
                    "board_addresses", &self.params.board_addresses))?;

            unwrap_py_init(py, py.run(
                r#"
from adafruit_servokit import ServoKit
boards = []
for i in range(num_boards):
    boards.append(ServoKit(channels=num_channels[i], address=board_addresses[i]))
"#,
                None, //Some(&globals),
                Some(&locals)
            ))?;

            // Get the kit instance from the locals dict
            self.boards = match locals.get_item("boards") {
                Some(k) => Some(k.to_object(py)),
                None => return Err(InitError::ServoKitNotFound)
            };
        }

        for i in 0..loco_ctrl::NUM_DRV_AXES {
            self.drv_idx_map.push(self.params.drv_idx_map[i].to_vec());
            self.str_idx_map.push(self.params.str_idx_map[i].to_vec());
        }

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
    
        let mut str_sk = [0f64; loco_ctrl::NUM_STR_AXES];
        let mut drv_sk = [0f64; loco_ctrl::NUM_DRV_AXES];

        // If make safe is true hold STR at current angle and set drv to 0
        if input_data.safe_mode {
            str_sk = self.current_str_sk;
        }
        else {
            // Otherwise convert the angles to PWM values
            //
            // Assume NUM_STR = NUM_DRV
            for i in 0..loco_ctrl::NUM_STR_AXES {
                str_sk[i] = poly_val(
                    &input_data.loco_ctrl_output.str_abs_pos_rad[i], 
                    &self.params.str_ang_rad_to_sk_coeffs[i]);
                
                drv_sk[i] = match input_data.loco_ctrl_output.drv_rate[i] {
                    AxisRate::Normalised(n) => poly_val(
                        &n, 
                        &self.params.drv_rate_norm_to_sk_coeffs[i]
                    ),
                    AxisRate::Absolute(_) => 
                        return Err(ProcError::AbsDrvRateUnsupported)
                };
            }
        }

        // Python manipulation, only on the rpi
        #[cfg(target_arch = "arm")]
        {
            // Get the python instance
            let py = match &self.gil {
                Some(g) => g.python(),
                None => return Err(ProcError::GilRefFailed)
            };

            // Set the locals up
            let locals = PyDict::new(py);
            unwrap_py_proc(py, locals.set_item("boards", match &self.boards {
                Some(k) => k,
                None => return Err(ProcError::ServoKitNotFound)
            }))?;
            unwrap_py_proc(py, 
                locals.set_item("num_boards", self.params.num_boards))?;
            unwrap_py_proc(py, 
                locals.set_item("drv_idx_map", &self.drv_idx_map))?;
            unwrap_py_proc(py, 
                locals.set_item("str_idx_map", &self.str_idx_map))?;
            unwrap_py_proc(py, locals.set_item("drv_sk", &drv_sk.to_vec()))?;
            unwrap_py_proc(py, locals.set_item("str_sk", &str_sk.to_vec()))?;

            // Run the python code
            unwrap_py_proc(py, py.run(
                r#"
for i in range(0, 6):
    print(f"DRV axis {i} is at address [{drv_idx_map[i][0]}, {drv_idx_map[i][1]}]")
    boards[drv_idx_map[i][0]].continuous_servo[drv_idx_map[i][1]].throttle = drv_sk[i]
    boards[str_idx_map[i][0]].servo[str_idx_map[i][1]].angle = str_sk[i]
                "#,
                None,
                Some(&locals)
            ))?;
        }

        // Update current steer position
        self.current_str_sk = str_sk;

        trace!("commands out:\n    drv: {:?}\n    str: {:?}", drv_sk, str_sk);

        Ok(((), self.report))
    }
}

impl ProcError {
    fn from_py_err(py: Python, e: PyErr) -> Self {
        error!("Python error occured:");
        e.print_and_set_sys_last_vars(py);
        ProcError::PythonError
    }
}

impl InitError {
    fn from_py_err(py: Python, e: PyErr) -> Self {
        error!("Python error occured:");
        e.print_and_set_sys_last_vars(py);
        InitError::PythonError
    }
}

// ---------------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ---------------------------------------------------------------------------

#[allow(dead_code)]
fn unwrap_py_proc<T>(py: Python, r: PyResult<T>) -> Result<T, ProcError> {
    match r {
        Ok(t) => Ok(t),
        Err(e) => Err(ProcError::from_py_err(py, e))
    }
}

#[allow(dead_code)]
fn unwrap_py_init<T>(py: Python, r: PyResult<T>) -> Result<T, InitError> {
    match r {
        Ok(t) => Ok(t),
        Err(e) => Err(InitError::from_py_err(py, e))
    }
}