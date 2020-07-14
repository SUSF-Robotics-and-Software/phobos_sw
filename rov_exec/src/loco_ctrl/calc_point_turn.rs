//! Point turn calculations

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// Internal imports
use super::*;
use util::maths::lin_map;

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl LocoCtrl {

    /// Perform the point turn command calculations
    pub(crate) fn calc_point_turn(&mut self) -> Result<(), super::LocoCtrlError> {

        // Command has previously been verified so we can just extract the
        // turn rate.
        let turn_rate_rads = self.current_cmd.unwrap()
            .turn_rate_rads.unwrap();
        
        // Axis arrays
        let mut str_axes = [AxisData::default(); NUM_STR_AXES];
        let mut drv_axes = [AxisData::default(); NUM_DRV_AXES];

        // Calculate steer axis angles
        for i in 0..NUM_STR_AXES {
            str_axes[i].abs_pos_rad = 
                -1f64 * (
                    self.params.str_axis_pos_m_rb[i][0]
                    /
                    self.params.str_axis_pos_m_rb[i][1]
                ).atan();
        }

        // Calculate drive axis rates
        for i in 0..NUM_DRV_AXES {
            let wheel_rate_rads = 
                turn_rate_rads
                * (
                    self.params.str_axis_pos_m_rb[i][0].powi(2)
                    + self.params.str_axis_pos_m_rb[i][1].powi(2)
                ).sqrt()
                / self.params.wheel_radius_m;
            
            drv_axes[i].rate = AxisRate::Normalised(lin_map(
                (
                    self.params.drv_min_abs_rate_rads[i],
                    self.params.drv_max_abs_rate_rads[i]
                ), (1f64, -1f64), wheel_rate_rads)
            );
            
            // If on the right side reverse direction
            if i > 3 {
                drv_axes[i].rate = drv_axes[i].rate.invert();
            }
        }

        // Build the target configuration
        self.target_loco_config = Some(LocoConfig {
            drv_axes,
            str_axes
        });

        Ok(())
    }
}