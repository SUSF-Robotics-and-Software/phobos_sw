//! Arm inverse kinematics calculations

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use ndarray::{prelude::*, stack};
use std::{array, collections::HashMap, fmt::DebugList};

use comms_if::eqpt::mech::{ActId, MechDems};
use log::debug;

// Internal imports
use super::*;

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl ArmCtrl {
    /// Perform the inverse kinematics calculations.
    ///
    /// Inverse kinematics is described in
    /// https://en.wikipedia.org/wiki/Inverse_kinematics and the
    /// derivation for the equations used can be seen in
    /// https://sotonac.sharepoint.com/:w:/t/SUSFORT2021/EZy22V-ApHtPqR5HTDhpOtQBieNeUIsqRxCZKjLvyYYcEQ?e=SMhspf.
    /// It involves taking an input coordinate for the head of the arm relative
    /// to the base of the arm and using trigonometry to find the angle required
    /// for the different joints.
    ///
    /// The command is parameterised by the angle and speed of rotation of the base of the arm,
    /// the x & y coordinate of the head of the arm, the speed of movement,
    /// the angle and speed of rotation of the wrist and the angel and speed of rotation for the claw.

    pub(crate) fn calc_inverse_kinematics(
        &mut self,
        base_pos_rad: f64,
        horizontal_distance_m: f64,
        vertical_distance_m: f64,
        wrist_pos_rad: f64,
        grabber_pos_rad: f64,
    ) -> Result<(), super::ArmCtrlError> {
        // Axis array
        let mut pos_rad = HashMap::new();
        let mut horizontal_distance_m = horizontal_distance_m;
        let mut vertical_distance_m = vertical_distance_m;

        let max_distance_m = self.params.shoulder_length_m + self.params.elbow_length_m;
        let min_distance_m = self.params.shoulder_length_m - self.params.elbow_length_m;
        let mut head_target_distance_m =
            (horizontal_distance_m.powi(2) + vertical_distance_m.powi(2)).sqrt();
        let delta_arm_square_m2 =
            self.params.shoulder_length_m.powi(2) - self.params.elbow_length_m.powi(2);

        // Limit target distance to be within range of arm
        if head_target_distance_m > max_distance_m {
            horizontal_distance_m *= max_distance_m / head_target_distance_m;
            vertical_distance_m *= max_distance_m / head_target_distance_m;
            head_target_distance_m = max_distance_m;
        } else if head_target_distance_m < min_distance_m {
            horizontal_distance_m *= min_distance_m / head_target_distance_m;
            vertical_distance_m *= min_distance_m / head_target_distance_m;
            head_target_distance_m = min_distance_m;
        }

        // Calculate elbow y solutions
        //
        // Calculate angle for shoulder joint
        let y_weighted_mid_point_m =
            0.5 * vertical_distance_m * (1. + delta_arm_square_m2 / head_target_distance_m.powi(2));

        let y_mid_point_offset_m = (-head_target_distance_m.powi(6)
            + head_target_distance_m.powi(4)
                * (vertical_distance_m.powi(2) - 2. * delta_arm_square_m2)
            + head_target_distance_m.powi(2)
                * (2. * vertical_distance_m.powi(2) * delta_arm_square_m2
                    - delta_arm_square_m2.powi(2)
                    + 4. * horizontal_distance_m.powi(2) * self.params.shoulder_length_m.powi(2))
            + vertical_distance_m.powi(2) * delta_arm_square_m2.powi(2))
        .abs()
        .sqrt()
            / (2. * head_target_distance_m.powi(2));

        let mut y_elbow_m = array![
            y_weighted_mid_point_m + y_mid_point_offset_m,
            y_weighted_mid_point_m - y_mid_point_offset_m
        ];

        let shoulder_length_solutions = y_elbow_m.mapv(|i| {
            self.params.shoulder_length_m - i < 0.
                && (self.params.shoulder_length_m.powi(2) - i.powi(2)).abs() < 1e-3
        });

        for (i, condition) in shoulder_length_solutions.indexed_iter() {
            if *condition {
                y_elbow_m[i] = self.params.shoulder_length_m;
            }
        }

        let x_elbow_pos_m =
            (self.params.shoulder_length_m.powi(2) - y_elbow_m.mapv(|i| i.powi(2))).mapv(f64::sqrt);
        let x_elbow_m = stack![Axis(0), x_elbow_pos_m, -&x_elbow_pos_m];
        let elbow_lengths_m = ((horizontal_distance_m - &x_elbow_m).mapv(|i| i.powi(2))
            + (vertical_distance_m - &y_elbow_m).mapv(|i| i.powi(2)))
        .mapv(f64::sqrt);
        let solution_idx = (&elbow_lengths_m - self.params.elbow_length_m).mapv(|i| i.abs() < 1e-4);


        if let Some(ref mut current_cfg) = self.current_arm_config {
            let mut shoulder_angle_rad = 0.;
            let mut elbow_angle_rad = 0.;
            for ((i, j), condition) in solution_idx.indexed_iter() {
                debug!("\nNew: {:?}\nOld: {:?}\n", (current_cfg.pos_rad[&ActId::ArmShoulder]
                    - (y_elbow_m[j] / x_elbow_m[[i, j]]).atan())
                .abs(), (current_cfg.pos_rad[&ActId::ArmShoulder] - shoulder_angle_rad).abs());
                if *condition
                    && (current_cfg.pos_rad[&ActId::ArmShoulder]
                        - (y_elbow_m[j] / x_elbow_m[[i, j]]).atan())
                    .abs()
                        < (current_cfg.pos_rad[&ActId::ArmShoulder] - shoulder_angle_rad).abs()
                {
                    shoulder_angle_rad = (y_elbow_m[j] / x_elbow_m[[i, j]]).atan();
                    elbow_angle_rad = ((vertical_distance_m - y_elbow_m[j])
                        / (horizontal_distance_m - x_elbow_m[[i, j]]))
                    .atan()
                        - shoulder_angle_rad;
                }
            }

            debug!("\nArm positions:\nShoulder: {:?}\nElbow: {:?}\n", shoulder_angle_rad, elbow_angle_rad);
            pos_rad.insert(ActId::ArmShoulder, shoulder_angle_rad);
            pos_rad.insert(ActId::ArmElbow, elbow_angle_rad);
            pos_rad.insert(ActId::ArmBase, base_pos_rad);
            pos_rad.insert(ActId::ArmWrist, wrist_pos_rad);
            pos_rad.insert(ActId::ArmGrabber, grabber_pos_rad);
        }
        debug!("\nArm positions:\n{:?}", pos_rad);

        self.target_arm_config = Some(MechDems {
            pos_rad,
            speed_rads: HashMap::new(),
        });

        Ok(())
    }
}
