//! Implementations for the LocoCtrl state structure

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// Internal
use super::{
    Params, 
    MnvrCommand, MnvrType,
    LocoConfig, Axis, AxisRate, 
    NUM_DRV_AXES, NUM_STR_AXES};
use util::{params, module::State};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Locomotion control module state
#[derive(Default)]
pub struct LocoCtrl {

    params: Params,

    report: StatusReport,

    current_cmd: Option<MnvrCommand>,

    target_loco_config: Option<LocoConfig>,

    previous_output: Option<OutputData>

}

/// Data required for initialising Locomotion Control.
pub struct InitData {

    /// Path to the `loco_ctrl.toml` parameters file.
    pub params_path: &'static str
}

/// Input data to Locomotion Control.
pub struct InputData {
    /// The manouvre command to be executed, or `None` if there is no new
    /// command on this cycle.
    pub cmd: Option<MnvrCommand>
}

/// Output command from LocoCtrl that the electronics driver must execute.
#[derive(Clone, Copy)]
pub struct OutputData {
    /// Steer axis absolute position demand in radians.
    /// 
    /// Units: radians
    pub str_abs_pos_rad: [Axis<f64>; NUM_STR_AXES],

    /// Drive axis rate demand.
    /// 
    /// Units: radians/second for `AxisRate::Absolute` or between -1 and +1 for
    ///        `AxisRate::Normalised`.
    pub drv_rate: [Axis<AxisRate>; NUM_DRV_AXES]
}

impl Default for OutputData {
    fn default() -> Self {
        OutputData {
            str_abs_pos_rad: Axis::<f64>::default_array(),
            drv_rate: Axis::<AxisRate>::array(AxisRate::Normalised(0.0))
        }
    }
}

/// Status report for LocoCtrl processing.
#[derive(Clone, Copy, Default)]
pub struct StatusReport {

}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl State for LocoCtrl {
    type InitData = InitData;
    type InitError = params::LoadError;
    
    type InputData = InputData;
    type OutputData = OutputData;
    type StatusReport = StatusReport;
    type ProcError = super::Error;

    /// Initialise the LocoCtrl module.
    fn init(&mut self, init_data: Self::InitData) 
        -> Result<(), Self::InitError> 
    {
        
        // Load the parameters
        self.params = match params::load(init_data.params_path) {
            Ok(p) => p,
            Err(e) => return Err(e)
        };

        // Thoese items wrapped in an `Option` will be defaulted to `None`, and
        // since there's no way we can get information on the current command
        // or configuration yet there's no need to change them.

        Ok(())
    }

    /// Perform cyclic processing of Locomotion Control.
    fn proc(&mut self, input_data: Self::InputData)
        -> Result<(Self::OutputData, Self::StatusReport), Self::ProcError> 
    {
        // Clear the status report
        self.report = StatusReport::default();

        // Check to see if there's a new command
        if let Some(cmd) = input_data.cmd {
            // Update the interal copy of the command
            self.current_cmd = Some(cmd);

            // Calculate the target configuration based on this new command.
            self.calc_target_config()?;
        }

        let output: OutputData;

        // If there's a target config to move to
        if let Some(cfg) = self.target_loco_config {
            let mut str_abs_pos =  [0f64; NUM_STR_AXES];
            let mut drv_rate = [AxisRate::Normalised(0.0); NUM_DRV_AXES];

            // Iterate over each leg and extract the str and drv demands from
            // the configuration
            for i in 0..NUM_DRV_AXES {
                str_abs_pos[i] = cfg.str_axes[i].unwrap().abs_pos_rad;
                drv_rate[i] = cfg.drv_axes[i].unwrap().rate;
            }

            output = OutputData {
                str_abs_pos_rad: Axis::<f64>::into_axis_array(&str_abs_pos),
                drv_rate: Axis::<AxisRate>::into_axis_array(&drv_rate)
            }
        }
        else {
            // If no target keep the previous output with the drive rates
            // zeroed. If there is no previous output use the default (zero)
            // position and rate.
            output = match self.previous_output {
                Some(po) => {
                    let mut o = po.clone();
                    o.drv_rate = Axis::<AxisRate>::array(AxisRate::Normalised(0.0));
                    o
                },
                None => OutputData::default()
            }
        }

        Ok((output, self.report))
    }
}

impl LocoCtrl {
    
    /// Based on the current command calculate a target configuration for
    /// the rover to achieve.
    /// 
    /// A valid command should be set in `self.current_cmd` before calling
    /// this function.
    fn calc_target_config(&mut self) -> Result<(), super::Error> {

        // Check we have a valid command
        match self.current_cmd {
            Some(c) => match c.is_valid() {
                true => (),
                false => return Err(super::Error::InvalidMnvrCmd(c))
            },
            None => return Err(super::Error::NoMnvrCmd)
        }

        // Perform calculations for each command type
        match self.current_cmd.unwrap().mnvr_type {
            MnvrType::Stop => self.calc_stop(),
            MnvrType::None => self.calc_none(),
            MnvrType::Ackerman => self.calc_ackerman(),
            MnvrType::PointTurn => self.calc_point_turn(),
            MnvrType::SkidSteer => self.calc_skid_steer()
        }
    }
    
    /// Perform the stop command calculations
    fn calc_stop(&mut self) -> Result<(), super::Error> {
        Err(super::Error::NotYetSupported(String::from(
            "Manouvre command 'Stop' is not yet supported")))
    }

    /// Perform the none command calculations
    fn calc_none(&mut self) -> Result<(), super::Error> {
        Err(super::Error::NotYetSupported(String::from(
            "Manouvre command 'None' is not yet supported")))
    }

    /// Perform the ackerman command calculations
    fn calc_ackerman(&mut self) -> Result<(), super::Error> {
        Err(super::Error::NotYetSupported(String::from(
            "Manouvre command 'Ackerman' is not yet supported")))
    }

    /// Perform the point turn command calculations
    fn calc_point_turn(&mut self) -> Result<(), super::Error> {
        Err(super::Error::NotYetSupported(String::from(
            "Manouvre command 'Point Turn' is not yet supported")))
    }

    /// Perform the skid steer command calculations
    fn calc_skid_steer(&mut self) -> Result<(), super::Error> {
        Err(super::Error::NotYetSupported(String::from(
            "Manouvre command 'Skid Steer' is not yet supported")))
    }
}