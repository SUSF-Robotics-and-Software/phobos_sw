//! Implementations for the LocoCtrl state structure

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use log::trace;
use serde::Serialize;

// Internal
use super::{
    Params, 
    MnvrCommand, MnvrType,
    LocoConfig, AxisData, AxisRate, 
    NUM_DRV_AXES, NUM_STR_AXES};
use util::{
    params, 
    module::State,
    archive::{Archived, Archiver},
    session::Session};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Locomotion control module state
#[derive(Default)]
pub struct LocoCtrl {

    pub(crate) params: Params,

    pub(crate) report: StatusReport,
    arch_report: Archiver,

    pub(crate) current_cmd: Option<MnvrCommand>,
    arch_current_cmd: Archiver,

    pub(crate) target_loco_config: Option<LocoConfig>,
    arch_target_loco_config: Archiver,

    pub(crate) output: Option<OutputData>,
    arch_output: Archiver
}

/// Input data to Locomotion Control.
#[derive(Default)]
pub struct InputData {
    /// The manouvre command to be executed, or `None` if there is no new
    /// command on this cycle.
    pub cmd: Option<MnvrCommand>
}

/// Output command from LocoCtrl that the electronics driver must execute.
#[derive(Clone, Copy, Serialize, Debug)]
pub struct OutputData {
    /// Steer axis absolute position demand in radians.
    /// 
    /// Units: radians
    pub str_abs_pos_rad: [f64; NUM_STR_AXES],

    /// Drive axis rate demand.
    /// 
    /// Units: radians/second for `AxisRate::Absolute` or between -1 and +1 for
    ///        `AxisRate::Normalised`.
    pub drv_rate: [AxisRate; NUM_DRV_AXES]
}

impl Default for OutputData {
    fn default() -> Self {
        OutputData {
            str_abs_pos_rad: [0.0; NUM_STR_AXES],
            drv_rate: [AxisRate::Normalised(0.0); NUM_DRV_AXES]
        }
    }
}

/// Status report for LocoCtrl processing.
#[derive(Clone, Copy, Default, Serialize, Debug)]
pub struct StatusReport {
    str_abs_pos_limited: [bool; NUM_STR_AXES],
    drv_rate_limited: [bool; NUM_STR_AXES],
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl State for LocoCtrl {
    type InitData = &'static str;
    type InitError = params::LoadError;
    
    type InputData = InputData;
    type OutputData = OutputData;
    type StatusReport = StatusReport;
    type ProcError = super::LocoCtrlError;

    /// Initialise the LocoCtrl module.
    ///
    /// Expected init data is the path to the parameter file
    fn init(&mut self, init_data: Self::InitData, session: &Session) 
        -> Result<(), Self::InitError> 
    {
        
        // Load the parameters
        self.params = match params::load(init_data) {
            Ok(p) => p,
            Err(e) => return Err(e)
        };

        // Create the arch folder for loco_ctrl
        let mut arch_path = session.arch_root.clone();
        arch_path.push("loco_ctrl");
        std::fs::create_dir_all(arch_path).unwrap();

        // Initialise the archivers
        self.arch_report = Archiver::from_path(
            session, "loco_ctrl/status_report.csv"
        ).unwrap();
        self.arch_current_cmd = Archiver::from_path(
            session, "loco_ctrl/current_cmd.csv"
        ).unwrap();
        self.arch_target_loco_config = Archiver::from_path(
            session, "loco_ctrl/target_loco_config.csv"
        ).unwrap();
        self.arch_output = Archiver::from_path(
            session, "loco_ctrl/output.csv"
        ).unwrap();

        // Thoese items wrapped in an `Option` will be defaulted to `None`, and
        // since there's no way we can get information on the current command
        // or configuration yet there's no need to change them.

        Ok(())
    }

    /// Perform cyclic processing of Locomotion Control.
    fn proc(&mut self, input_data: &Self::InputData)
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
            let mut drv_rate = 
                [AxisRate::Normalised(0.0); NUM_DRV_AXES];

            // Iterate over each leg and extract the str and drv demands from
            // the configuration
            for i in 0..NUM_DRV_AXES {
                str_abs_pos[i] = cfg.str_axes[i].abs_pos_rad;
                drv_rate[i] = cfg.drv_axes[i].rate;
            }

            output = OutputData {
                str_abs_pos_rad: str_abs_pos,
                drv_rate: drv_rate
            }
        }
        else {
            // If no target keep the previous output with the drive rates
            // zeroed. If there is no previous output use the default (zero)
            // position and rate.
            output = match self.output {
                Some(po) => {
                    let mut o = po.clone();
                    o.drv_rate = [AxisRate::Normalised(0.0); NUM_DRV_AXES];
                    o
                },
                None => OutputData::default()
            }
        }

        trace!("LocoCtrl output:\n    drv: {:?}\n    strL {:?}", 
            output.drv_rate, 
            output.str_abs_pos_rad);

        // Update the output in self
        self.output = Some(output);

        Ok((output, self.report))
    }
}

impl Archived for LocoCtrl {
    fn write(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Write each one individually
        self.arch_report.serialise(self.report)?;
        self.arch_current_cmd.serialise(self.current_cmd)?;
        self.arch_target_loco_config.serialise(self.target_loco_config)?;
        self.arch_output.serialise(self.output)?;

        Ok(())
    }
}

impl LocoCtrl {
    
    /// Based on the current command calculate a target configuration for
    /// the rover to achieve.
    /// 
    /// A valid command should be set in `self.current_cmd` before calling
    /// this function.
    fn calc_target_config(&mut self) -> Result<(), super::LocoCtrlError> {

        // Check we have a valid command
        match self.current_cmd {
            Some(c) => match c.is_valid() {
                true => (),
                false => return Err(super::LocoCtrlError::InvalidMnvrCmd(c))
            },
            None => return Err(super::LocoCtrlError::NoMnvrCmd)
        }

        // Perform calculations for each command type. These calculation
        // functions shall update `self.target_loco_config`.
        match self.current_cmd.unwrap().mnvr_type {
            MnvrType::Stop => self.calc_stop()?,
            MnvrType::None => self.calc_none()?,
            MnvrType::Ackerman => self.calc_ackerman()?,
            MnvrType::PointTurn => self.calc_point_turn()?,
            MnvrType::SkidSteer => self.calc_skid_steer()?
        };

        // Limit target to rover capabilities
        self.enforce_limits()
    }

    /// Enforce the limits in the Rover's hardware capabilities.
    ///
    /// This function shall modify the current target configuration to ensure
    /// that no capability of the rover is exceeded.
    ///
    /// If a limit is reached the corresponding flag in the status report will 
    /// be raised.
    fn enforce_limits(&mut self) -> Result<(), super::LocoCtrlError> {

        // Get a copy of the config, or return if there isn't one
        let mut target_config = match self.target_loco_config {
            Some(t) => t,
            None => return Ok(())
        };

        // Check steer axis abs pos limits
        for i in 0..NUM_STR_AXES {
            if target_config.str_axes[i].abs_pos_rad 
                > 
                self.params.str_max_abs_pos_rad[i] 
            {
                target_config.str_axes[i].abs_pos_rad = 
                    self.params.str_max_abs_pos_rad[i];
                self.report.str_abs_pos_limited[i] = true;
            }
            if target_config.str_axes[i].abs_pos_rad 
                <
                self.params.str_min_abs_pos_rad[i] 
            {
                target_config.str_axes[i].abs_pos_rad = 
                    self.params.str_max_abs_pos_rad[i];
                self.report.str_abs_pos_limited[i] = true;
            }
        }

        // Check drive axis abs pos limits
        for i in 0..NUM_DRV_AXES {
            let rate_norm = match target_config.drv_axes[i].rate {
                AxisRate::Normalised(n) => n,
                AxisRate::Absolute(r) => util::maths::lin_map(
                    (
                        self.params.drv_min_abs_rate_rads[i],
                        self.params.drv_max_abs_rate_rads[i]
                    ), (-1f64, 1f64), r)
            };

            if rate_norm > 1.0 {
                target_config.drv_axes[i].rate = AxisRate::Normalised(1.0);
                self.report.drv_rate_limited[i] = true;
            }
            if rate_norm < -1.0 {
                target_config.drv_axes[i].rate = AxisRate::Normalised(-1.0);
                self.report.drv_rate_limited[i] = true;
            }
        }

        // Update the target
        self.target_loco_config = Some(target_config);

        Ok(())

    }
    
    /// Perform the stop command calculations.
    /// 
    /// The stop command shall:
    ///     1. Maintain the current steer axis positions
    ///     2. Set all drive axes to stopping.
    /// 
    /// Stop shall never error and must always succeed in bringing the rover to
    /// a full and complete stop.
    fn calc_stop(&mut self) -> Result<(), super::LocoCtrlError> {

        // Get the current target or an empty (all zero) target if no target is
        // currently set.
        //
        // Modify the current target to have all drive axes set at zero.
        let target = match self.target_loco_config {
            Some(t) => {
                let mut t = t.clone();
                
                // Modify the target's rates to be zero, demanding that the 
                // rover stop.
                for i in 0..NUM_DRV_AXES {
                    t.str_axes[i].rate = AxisRate::Normalised(0.0);
                    t.drv_axes[i].rate = AxisRate::Normalised(0.0);
                }

                t
            },
            None => {
                let default = AxisData {
                    abs_pos_rad: 0.0,
                    rate: AxisRate::Normalised(0.0)
                };

                LocoConfig {
                    str_axes: [default; NUM_STR_AXES],
                    drv_axes: [default; NUM_DRV_AXES]
                }
            }
        };

        // Update the target
        self.target_loco_config = Some(target);
        
        Ok(())
    }

    /// Perform the none command calculations.
    /// 
    /// The None command shall not change the current target.
    fn calc_none(&mut self) -> Result<(), super::LocoCtrlError> {
        
        // Simply exit as there's nothing to do.
        Ok(())
    }

}