//! Arm inverse kinematics calculations

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

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
        horizontal_distance_m: f64,
        vertical_distance_m: f64,
        speed_ms: f64,
    ) -> Result<(), super::ArmCtrlError> {
        // Axis array
        let mut rot_axes = [AxisData::default(); NUM_ROT_AXES];

        let max_distance_m = self.params.shoulder_length_m + self.params.elbow_length_m;
        let head_target_distance_m =
            (horizontal_distance_m.powi(2) + vertical_distance_m.powi(2)).sqrt();
        let delta_arm_square_m2 =
            self.params.shoulder_length_m.powi(2) - self.params.elbow_length_m.powi(2);

        if head_target_distance_m > max_distance_m {
            horizontal_distance_m *= max_distance_m / head_target_distance_m;
            vertical_distance_m *= max_distance_m / head_target_distance_m;
            head_target_distance_m = max_distance_m
        }

        let weighted_mid_point_m =
            0.5 * vertical_distance_m * (1. + delta_arm_square_m2 / head_target_distance_m.powi(2));

        let mid_point_offset_m = (-head_target_distance_m.powi(6)
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

        self.target_arm_config = Some(ArmConfig { rot_axes });

        Ok(())
    }
}
