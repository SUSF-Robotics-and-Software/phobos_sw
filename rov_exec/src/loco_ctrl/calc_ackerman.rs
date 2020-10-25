//! Ackerman manouvre calculations

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// Internal imports
use super::*;
use util::maths::clamp;

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl LocoCtrl {

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
    ///
    /// TODO: Add crab
    pub(crate) fn calc_ackerman(
        &mut self, 
        speed_ms: f64,
        curv_m: f64, 
        _crab_ms: f64
    ) -> Result<(), super::LocoCtrlError> {

        // If the demanded curvature is close to zero set the target to point
        // straight ahead.
        if curv_m.abs() < self.params.ackerman_min_curvature_m {
            self.calc_ackerman_straight(speed_ms)?;
        }
        // Otherwise perform the generic ackerman calculation
        else {
            self.calc_ackerman_generic(speed_ms, curv_m)?;
        }

        Ok(())
    }

    /// Calculate the ackerman outputs for a straight drive
    fn calc_ackerman_straight(
        &mut self, speed_ms: f64
    ) -> Result<(), super::LocoCtrlError> {
        // Convert the desired speed into normalised speed
        let mut drv_axes = [AxisData::default(); NUM_DRV_AXES];

        // Calculate the required speed in wheel radians/second
        let wheel_rate_rads = speed_ms/self.params.wheel_radius_m;

        for i in 0..NUM_DRV_AXES {
            drv_axes[i].rate_rads = wheel_rate_rads;
        }

        // Build the new target
        self.target_loco_config = Some(LocoConfig {
            str_axes: [AxisData::default(); NUM_STR_AXES],
            drv_axes
        });

        Ok(())
    }

    /// Calculate generic ackerman outputs.
    ///
    /// The calling function must ensure that the curvature demand is not less
    /// than `loco_ctrl::Params::ackerman_min_curvature_m`. Failure to do so
    /// may result in a division by zero.
    fn calc_ackerman_generic(
        &mut self,
        speed_ms: f64,
        curv_m: f64
    ) -> Result<(), super::LocoCtrlError> {
        
        // Axis arrays
        let mut str_axes = [AxisData::default(); NUM_STR_AXES];
        let mut drv_axes = [AxisData::default(); NUM_DRV_AXES];

        // Compute the radius of curvature in meters.
        //
        // Clamp the curvature to the maximum curv value.
        //
        //  Note: No check is required for a division by zero as this check
        //  is performed by the calling function.
        let curv_radius_m = 
            1.0 
            / 
            clamp(
                &curv_m, 
                &(-self.params.ackerman_max_curvature_m),
                &self.params.ackerman_max_curvature_m);

        // Steer axis angles
        //
        // These angles are based on the wheel positions and curvature. To find
        // the correct angle (with the right sign, i.e. right hand grip rule 
        // about RB_Z), take the arctan of the RB_X position of the wheel and 
        // the RB_Y component of the line joining the centre of rotation and 
        // the centre of the steer axis motor.
        //
        // We use atan2 here to respect signs.
        for i in 0..NUM_STR_AXES {

            str_axes[i].abs_pos_rad = 
            (
                self.params.str_axis_pos_m_rb[i][0]
                /
                (curv_radius_m - self.params.str_axis_pos_m_rb[i][1])
            ).atan();

        }

        // Drive rate
        //
        // Drive rate is based on the idea that all wheels will rotate about
        // concentric circles centred on the centre of rotation. If the desired
        // speed of the rover is known we can work out the angular rate about
        // the CoR, and use `v = r\omega` to work out the speed of each 
        // individual wheel.
        for i in 0..NUM_DRV_AXES {

            // Calculate the desired speed at this wheel's radius from the
            // centre of rotation
            let wheel_speed_ms = (speed_ms / curv_radius_m.abs())
                * (
                    (curv_radius_m - self.params.str_axis_pos_m_rb[i][1]).powi(2)
                    + self.params.str_axis_pos_m_rb[i][0].powi(2)
                ).sqrt();
            
            // Calculate the wheel rate by converting the speed into rads/s
            drv_axes[i].rate_rads = 
                wheel_speed_ms / self.params.wheel_radius_m;
        }

        // Build the target configuration
        self.target_loco_config = Some(LocoConfig {
            drv_axes,
            str_axes
        });

        Ok(())
    }

}