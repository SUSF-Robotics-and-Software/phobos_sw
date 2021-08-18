//! Implementations for the LocoCtrl state structure

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use log::debug;
use serde::Serialize;

// Internal
use super::{AxisData, LocoConfig, Params, NUM_DRV_AXES, NUM_STR_AXES};
use comms_if::{
    eqpt::mech::{ActId, MechDems},
    tc::loco_ctrl::MnvrCmd,
};
use std::collections::HashMap;
use util::{
    archive::{Archived, Archiver},
    module::State,
    params,
    session::Session,
};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Locomotion control module state
#[derive(Default)]
pub struct LocoCtrl {
    pub(crate) params: Params,

    pub(crate) report: StatusReport,
    arch_report: Archiver,

    pub(crate) current_cmd: Option<MnvrCmd>,
    arch_current_cmd: Archiver,

    pub(crate) target_loco_config: Option<LocoConfig>,
    arch_target_loco_config: Archiver,

    pub(crate) output: Option<MechDems>,
    arch_output: Archiver,
}

/// Input data to Locomotion Control.
#[derive(Default)]
pub struct InputData {
    /// The manouvre command to be executed, or `None` if there is no new
    /// command on this cycle.
    pub cmd: Option<MnvrCmd>,
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
    type OutputData = MechDems;
    type StatusReport = StatusReport;
    type ProcError = super::LocoCtrlError;

    /// Initialise the LocoCtrl module.
    ///
    /// Expected init data is the path to the parameter file
    fn init(
        &mut self,
        init_data: Self::InitData,
        session: &Session,
    ) -> Result<(), Self::InitError> {
        // Load the parameters
        self.params = match params::load(init_data) {
            Ok(p) => p,
            Err(e) => return Err(e),
        };

        // Create the arch folder for loco_ctrl
        let mut arch_path = session.arch_root.clone();
        arch_path.push("loco_ctrl");
        std::fs::create_dir_all(arch_path).unwrap();

        // Initialise the archivers
        self.arch_report = Archiver::from_path(session, "loco_ctrl/status_report.csv").unwrap();
        self.arch_current_cmd = Archiver::from_path(session, "loco_ctrl/current_cmd.csv").unwrap();
        self.arch_target_loco_config =
            Archiver::from_path(session, "loco_ctrl/target_loco_config.csv").unwrap();
        self.arch_output = Archiver::from_path(session, "loco_ctrl/output.csv").unwrap();

        // Thoese items wrapped in an `Option` will be defaulted to `None`, and
        // since there's no way we can get information on the current command
        // or configuration yet there's no need to change them.

        Ok(())
    }

    /// Perform cyclic processing of Locomotion Control.
    fn proc(
        &mut self,
        input_data: &Self::InputData,
    ) -> Result<(Self::OutputData, Self::StatusReport), Self::ProcError> {
        // Clear the status report
        self.report = StatusReport::default();

        // Check to see if there's a new command
        if let Some(cmd) = input_data.cmd {
            // Update the interal copy of the command
            self.current_cmd = Some(cmd);

            // Ouptut the command in debug mode
            // debug!("New LocoCtrl MnvrCmd::{:#?}", cmd);

            // Calculate the target configuration based on this new command.
            self.calc_target_config()?;
        }

        // Calculate the output
        self.set_output();

        Ok((
            match self.output {
                Some(ref o) => o.clone(),
                None => MechDems::default(),
            },
            self.report,
        ))
    }
}

impl Archived for LocoCtrl {
    fn write(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Write each one individually
        self.arch_report.serialise(self.report)?;
        self.arch_current_cmd.serialise(self.current_cmd)?;
        self.arch_target_loco_config
            .serialise(self.target_loco_config)?;
        self.arch_output.serialise(self.output.clone())?;

        Ok(())
    }
}

impl LocoCtrl {
    /// Function called when entering safe mode.
    ///
    /// Must result in no motion of the vehicle
    pub fn make_safe(&mut self) {
        self.current_cmd = Some(MnvrCmd::Stop);

        self.calc_target_config().unwrap();

        self.set_output();
    }

    /// Set the output based on the target loco config.
    fn set_output(&mut self) {
        let output: MechDems;

        // If there's a target config to move to
        if let Some(cfg) = self.target_loco_config {
            let mut pos_rad = HashMap::new();
            let mut speed_rads = HashMap::new();

            // Steer axis positions
            pos_rad.insert(ActId::StrFL, cfg.str_axes[0].abs_pos_rad);
            pos_rad.insert(ActId::StrML, cfg.str_axes[1].abs_pos_rad);
            pos_rad.insert(ActId::StrRL, cfg.str_axes[2].abs_pos_rad);
            pos_rad.insert(ActId::StrFR, cfg.str_axes[3].abs_pos_rad);
            pos_rad.insert(ActId::StrMR, cfg.str_axes[4].abs_pos_rad);
            pos_rad.insert(ActId::StrRR, cfg.str_axes[5].abs_pos_rad);

            // Drive axis rates
            speed_rads.insert(ActId::DrvFL, cfg.drv_axes[0].rate_rads);
            speed_rads.insert(ActId::DrvML, cfg.drv_axes[1].rate_rads);
            speed_rads.insert(ActId::DrvRL, cfg.drv_axes[2].rate_rads);
            speed_rads.insert(ActId::DrvFR, cfg.drv_axes[3].rate_rads);
            speed_rads.insert(ActId::DrvMR, cfg.drv_axes[4].rate_rads);
            speed_rads.insert(ActId::DrvRR, cfg.drv_axes[5].rate_rads);

            output = MechDems {
                pos_rad,
                speed_rads,
            }
        } else {
            // If no target keep the previous output with the drive rates
            // zeroed. If there is no previous output use the default (zero)
            // position and rate.
            output = match self.output.take() {
                Some(po) => {
                    let mut o = po.clone();
                    for (_, speed_rads) in o.speed_rads.iter_mut() {
                        *speed_rads = 0.0;
                    }
                    o
                }
                None => MechDems::default(),
            }
        }

        // Update the output in self
        self.output = Some(output.clone());
    }

    /// Based on the current command calculate a target configuration for
    /// the rover to achieve.
    ///
    /// A valid command should be set in `self.current_cmd` before calling
    /// this function.
    fn calc_target_config(&mut self) -> Result<(), super::LocoCtrlError> {
        // Check we have a valid command
        match self.is_current_cmd_valid() {
            true => (),
            false => return Err(super::LocoCtrlError::InvalidMnvrCmd),
        }

        // Perform calculations for each command type. These calculation
        // functions shall update `self.target_loco_config`.
        match self.current_cmd.unwrap() {
            MnvrCmd::Stop => self.calc_stop()?,
            MnvrCmd::Ackerman {
                speed_ms,
                curv_m,
                crab_rad,
            } => self.calc_ackerman(speed_ms, curv_m, crab_rad)?,
            MnvrCmd::PointTurn { rate_rads } => self.calc_point_turn(rate_rads)?,
            MnvrCmd::SkidSteer { speed_ms, curv_m } => self.calc_skid_steer(speed_ms, curv_m)?,
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
            None => return Ok(()),
        };

        // Check steer axis abs pos limits
        for i in 0..NUM_STR_AXES {
            if target_config.str_axes[i].abs_pos_rad > self.params.str_max_abs_pos_rad[i] {
                target_config.str_axes[i].abs_pos_rad = self.params.str_max_abs_pos_rad[i];
                self.report.str_abs_pos_limited[i] = true;
            }
            if target_config.str_axes[i].abs_pos_rad < self.params.str_min_abs_pos_rad[i] {
                target_config.str_axes[i].abs_pos_rad = self.params.str_max_abs_pos_rad[i];
                self.report.str_abs_pos_limited[i] = true;
            }
        }

        // Check drive axis abs pos limits
        for i in 0..NUM_DRV_AXES {
            if target_config.drv_axes[i].rate_rads > self.params.drv_max_abs_rate_rads[i] {
                target_config.drv_axes[i].rate_rads = self.params.drv_max_abs_rate_rads[i];
                self.report.drv_rate_limited[i] = true;
            }
            if target_config.drv_axes[i].rate_rads < self.params.drv_min_abs_rate_rads[i] {
                target_config.drv_axes[i].rate_rads = self.params.drv_min_abs_rate_rads[i];
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
                    t.str_axes[i].rate_rads = 0.0;
                    t.drv_axes[i].rate_rads = 0.0;
                }

                t
            }
            None => {
                let default = AxisData {
                    abs_pos_rad: 0.0,
                    rate_rads: 0.0,
                };

                LocoConfig {
                    str_axes: [default; NUM_STR_AXES],
                    drv_axes: [default; NUM_DRV_AXES],
                }
            }
        };

        // Update the target
        self.target_loco_config = Some(target);

        Ok(())
    }

    /// Validate that the current manouvre command is achievable
    /// TODO
    fn is_current_cmd_valid(&self) -> bool {
        true
    }
}
