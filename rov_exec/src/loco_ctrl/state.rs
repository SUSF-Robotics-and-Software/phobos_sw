//! Implementations for the LocoCtrl state structure

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
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
    maths::lin_map, 
    archive::{Archived, Archiver, Writer},
    session::Session};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Locomotion control module state
#[derive(Default)]
pub struct LocoCtrl {

    params: Params,

    report: StatusReport,
    arch_report: Archiver,

    current_cmd: Option<MnvrCommand>,
    arch_current_cmd: Archiver,

    target_loco_config: Option<LocoConfig>,
    arch_target_loco_config: Archiver,

    output: Option<OutputData>,
    arch_output: Archiver
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
#[derive(Clone, Copy, Serialize)]
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
#[derive(Clone, Copy, Default, Serialize)]
pub struct StatusReport {
    dummy: i32,
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
    fn init(&mut self, init_data: Self::InitData, session: &Session) 
        -> Result<(), Self::InitError> 
    {
        
        // Load the parameters
        self.params = match params::load(init_data.params_path) {
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
    fn calc_target_config(&mut self) -> Result<(), super::Error> {

        // Check we have a valid command
        match self.current_cmd {
            Some(c) => match c.is_valid() {
                true => (),
                false => return Err(super::Error::InvalidMnvrCmd(c))
            },
            None => return Err(super::Error::NoMnvrCmd)
        }

        // Perform calculations for each command type. These calculation
        // functions shall update `self.target_loco_config`.
        match self.current_cmd.unwrap().mnvr_type {
            MnvrType::Stop => self.calc_stop(),
            MnvrType::None => self.calc_none(),
            MnvrType::Ackerman => self.calc_ackerman(),
            MnvrType::PointTurn => self.calc_point_turn(),
            MnvrType::SkidSteer => self.calc_skid_steer()
        }
    }
    
    /// Perform the stop command calculations.
    /// 
    /// The stop command shall:
    ///     1. Maintain the current steer axis positions
    ///     2. Set all drive axes to stopping.
    /// 
    /// Stop shall never error and must always succeed in bringing the rover to
    /// a full and complete stop.
    fn calc_stop(&mut self) -> Result<(), super::Error> {

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
    fn calc_none(&mut self) -> Result<(), super::Error> {
        
        // Simply exit as there's nothing to do.
        Ok(())
    }

    /// Perform the ackerman command calculations.
    /// 
    /// The Ackerman manouvre is described in 
    /// https://en.wikipedia.org/wiki/Ackermann_steering_geometry, and involves
    /// the rover pivoting about a point known as the centre of rotation. All
    /// wheel tangents (those passing through the central axis of the wheel
    /// and perpendicular to the current wheel "forward" direction) intersect
    /// at the centre of rotation. The centre of rotation must be outside the
    /// wheelbase.
    /// 
    /// The manouvre is parameterised by the curvature of the turn (1/radius
    /// of the turn) and the desired speed of the rover. Curvature is used so
    /// that infinity can be avoided for "straight" manouvres.
    fn calc_ackerman(&mut self) -> Result<(), super::Error> {

        // Command has previously been verified so we can just extract the
        // curvature and speed for future use.
        let curvature_m = self.current_cmd.unwrap().curvature_m.unwrap();
        let speed_ms = self.current_cmd.unwrap().speed_ms.unwrap();

        // If the demanded curvature is close to zero set the target to point
        // straight ahead.
        if curvature_m.abs() < self.params.ackerman_min_curvature_m {
            // Convert the desired speed into normalised speed
            let mut drv_axes = [AxisData::default(); NUM_DRV_AXES];

            for i in 0..NUM_DRV_AXES {
                drv_axes[i].rate = AxisRate::Normalised(lin_map(
                    (
                        self.params.drv_min_abs_rate_rads[i], 
                        self.params.drv_max_abs_rate_rads[i]
                    ), (-1f64, 1f64), speed_ms));
            }

            // Build the new target
            self.target_loco_config = Some(LocoConfig {
                str_axes: [AxisData::default(); NUM_STR_AXES],
                drv_axes
            });
        }
        else {
            // TODO Perform the normal ackerman calculation
            return Err(super::Error::NotYetSupported(String::from(
                "Non-straight Ackerman manouvres not yet supported"
            )))
        }

        Ok(())
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