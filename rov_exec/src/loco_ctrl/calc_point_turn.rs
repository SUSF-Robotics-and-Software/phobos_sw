//! Point turn calculations

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// Internal imports
use super::*;

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl LocoCtrl {

    /// Perform the point turn command calculations
    pub(crate) fn calc_point_turn(&mut self, rate_rads: f64) -> Result<(), super::LocoCtrlError> {

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
            drv_axes[i].rate_rads = 
                rate_rads
                * (
                    self.params.str_axis_pos_m_rb[i][0].powi(2)
                    + self.params.str_axis_pos_m_rb[i][1].powi(2)
                ).sqrt()
                / self.params.wheel_radius_m;
            
            // If on the right side reverse direction
            if i < 3 {
                drv_axes[i].rate_rads *= -1.0;
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