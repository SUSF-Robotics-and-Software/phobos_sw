//! Implementations for the ArmCtrl state structure

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use log::debug;
use serde::{Deserialize, Serialize};

// Internal
use super::{Params, NUM_ROT_AXES};
use comms_if::{
    eqpt::mech::{ActId, MechDems},
    tc::arm_ctrl::ArmCmd,
};
use std::collections::HashMap;
use util::{module::State, params, session::Session};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Arm control module state
#[derive(Default)]
pub struct ArmCtrl {
    pub(crate) params: Params,

    pub(crate) report: StatusReport,

    pub(crate) current_cmd: Option<ArmCmd>,

    pub(crate) current_arm_config: Option<MechDems>,

    pub(crate) target_arm_config: Option<MechDems>,

    pub(crate) output: Option<MechDems>,
}

/// Input data to Arm Control.
#[derive(Default)]
pub struct InputData {
    /// The rotation command to be executed, or `None` if there is no new
    /// command on this cycle.
    pub cmd: Option<ArmCmd>,
}

/// Status report for ArmCtrl processing.
#[derive(Clone, Copy, Default, Serialize, Deserialize, Debug)]
pub struct StatusReport {
    pub abs_pos_limited: [bool; NUM_ROT_AXES],
    pub rate_limited: [bool; NUM_ROT_AXES],
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl State for ArmCtrl {
    type InitData = &'static str;
    type InitError = params::LoadError;

    type InputData = InputData;
    type OutputData = MechDems;
    type StatusReport = StatusReport;
    type ProcError = super::ArmCtrlError;

    /// Initialise the ArmCtrl module.
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

        self.current_arm_config = Some(MechDems::default());
        self.target_arm_config = self.current_arm_config.clone();

        Ok(())
    }

    /// Perform cyclic processing of Arm Control.
    fn proc(
        &mut self,
        input_data: &Self::InputData,
    ) -> Result<(Self::OutputData, Self::StatusReport), Self::ProcError> {
        // Clear the status report
        self.report = StatusReport::default();

        // Check to see if there's a new command
        if let Some(cmd) = &input_data.cmd {
            // Update the interal copy of the command
            self.current_cmd = Some(cmd.clone());

            // Ouptut the command in debug mode
            debug!("New ArmCtrl ArmCmd::{:#?}", cmd);

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

impl ArmCtrl {
    /// Function called when entering safe mode.
    ///
    /// Must result in no motion of the vehicle
    pub fn make_safe(&mut self) {
        self.current_cmd = Some(ArmCmd::Stop);

        self.calc_target_config().unwrap();

        self.set_output();
    }

    /// Set the output based on the target arm config.
    /// TODO Lerp
    fn set_output(&mut self) {
        let output: MechDems;

        // If there's a target config to move to
        if let Some(target_cfg) = &self.target_arm_config {
            // TODO Lerp here, mutate current
            if let Some(ref mut current_cfg) = self.current_arm_config {
                let mut pos_rad = HashMap::new();

                for (i, act_id) in ActId::arm_ids().iter().enumerate() {
                    *current_cfg.pos_rad.get_mut(act_id).unwrap() += ((target_cfg.pos_rad[act_id]
                        - current_cfg.pos_rad[act_id])
                        * crate::CYCLE_FREQUENCY_HZ)
                        .clamp(
                            self.params.min_abs_rate_rads[i],
                            self.params.max_abs_rate_rads[i],
                        )
                        / crate::CYCLE_FREQUENCY_HZ;

                    pos_rad.insert(*act_id, current_cfg.pos_rad[act_id]);
                }
                // Rotational axis positions
                // TODO Insert current

                // TODO (Better comment) Merged so empty speed required
                output = MechDems {
                    pos_rad,
                    speed_rads: HashMap::new(),
                }
            } else {
                // If no target keep the previous output with the rotation rates
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
        } else {
            // If no target keep the previous output with the rotation rates
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
    /// the arm to achieve.
    ///
    /// A valid command should be set in `self.current_cmd` before calling
    /// this function.
    fn calc_target_config(&mut self) -> Result<(), super::ArmCtrlError> {
        // Check we have a valid command
        match self.is_current_cmd_valid() {
            true => (),
            false => return Err(super::ArmCtrlError::InvalidArmCmd),
        }

        // Perform calculations for each command type. These calculation
        // functions shall update `self.target_arm_config`.
        if let Some(cmd) = &self.current_cmd {
            match cmd {
                ArmCmd::Stop => self.calc_stop()?,
                ArmCmd::BasicRotation { dems } => {
                    self.target_arm_config = Some(dems.clone());
                }
                ArmCmd::InverseKinematics {
                    horizontal_distance_m,
                    vertical_distance_m,
                    speed_ms,
                } => self.calc_inverse_kinematics(
                    *horizontal_distance_m,
                    *vertical_distance_m,
                    *speed_ms,
                )?,
            }
        }

        // Limit target to rover capabilities
        self.enforce_limits()
    }

    /// Enforce the limits in the arm's hardware capabilities.
    ///
    /// This function shall modify the current target configuration to ensure
    /// that no capability of the arm is exceeded.
    ///
    /// If a limit is reached the corresponding flag in the status report will
    /// be raised.
    fn enforce_limits(&mut self) -> Result<(), super::ArmCtrlError> {
        // Get a copy of the config, or return if there isn't one
        if let Some(ref mut target_config) = self.target_arm_config {
            // Check rotation axis abs pos limits
            for (i, act_id) in ActId::arm_ids().iter().enumerate() {
                if target_config.pos_rad[act_id] > self.params.max_abs_pos_rad[i] {
                    *target_config.pos_rad.get_mut(act_id).unwrap() =
                        self.params.max_abs_pos_rad[i];
                    self.report.abs_pos_limited[i] = true;
                }
                if target_config.pos_rad[act_id] < self.params.min_abs_pos_rad[i] {
                    *target_config.pos_rad.get_mut(act_id).unwrap() =
                        self.params.min_abs_pos_rad[i];
                    self.report.abs_pos_limited[i] = true;
                }
            }
        }

        Ok(())
    }

    /// Perform the stop command calculations.
    ///
    /// The stop command shall:
    ///     1. Maintain the current rotation axis positions
    ///     2. Set all rotation axes to stopping.
    ///
    /// Stop shall never error and must always succeed in bringing the arm to
    /// a full and complete stop.
    fn calc_stop(&mut self) -> Result<(), super::ArmCtrlError> {
        // Get the current target or an empty (all zero) target if no target is
        // currently set.
        if let Some(target) = &mut self.target_arm_config {
            // If there is a current we discard the target and replace it with current.
            if let Some(current) = &self.current_arm_config {
                target.pos_rad = current.pos_rad.clone();
            }
            // If no current don't mutate target
        }
        // If no target no need to mutate it

        Ok(())
    }

    /// Validate that the current arm command is achievable
    /// TODO
    fn is_current_cmd_valid(&self) -> bool {
        true
    }
}
