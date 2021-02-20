//! # Trajectory controllers module
//!
//! This module provides the PID controllers used for TrajCtrl, including their
//! error calculations.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use std::time::Instant;

use nalgebra::Vector2;
// Internal
use crate::auto::{
    loc::Pose,
    path::*
};
use comms_if::tc::loco_ctrl::MnvrCmd;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A PID controller
#[derive(Debug)]
pub struct PidController {
    /// Previous instant that the error was passed in 
    prev_time: Option<Instant>,

    /// Proportional gain
    k_p: f64,

    /// Integral gain
    k_i: f64,

    /// Dervative gain
    k_d: f64,

    /// Previous error
    prev_error: Option<f64>,

    /// The integral accumulation
    integral: f64
}

/// The trajectory controllers
#[derive(Debug)]
pub struct TrajControllers {
    /// Lateral error controller
    lat_ctrl: PidController,

    /// Heading error controller
    head_ctrl: PidController
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl PidController {

    /// Create a new controller with the given gains.
    pub fn new(k_p: f64, k_i: f64, k_d: f64) -> Self {
        Self {
            k_p, k_i, k_d,
            integral: 0f64,
            prev_time: None,
            prev_error: None
        }
    }

    /// Get the value of the controller for the given error.
    ///
    /// This function is time-aware so there is no need to pass in a delta-time
    /// value.
    pub fn get(&mut self, error: f64) -> f64 {
        // Get current time
        let curr_time = Instant::now();

        // Calculate dt
        let dt = match self.prev_time {
            Some(t0) => Some((curr_time - t0).as_secs_f64()),
            None => None
        };

        // Accumulate the integral term.
        //
        // If there's no time difference then we don't accumulate the integral
        // The other option is to add on the error and that will produce a 
        // large spike in integral compared to normal operation, so we don't do
        // this.
        self.integral += match dt {
            Some(t) => error * t,
            None => 0f64
        };

        // Calculate the derivative.
        //
        // If there's no time difference again we assume no derivative, for the
        // same reasons as for integral.
        let deriv = match self.prev_error {
            Some(e) => match dt {
                Some(t) => (error - e) / t,
                None => 0f64
            },
            None => match dt {
                Some(t) => error / t,
                None => 0f64
            }
        };

        // Calculate the output
        let out = 
            self.k_p * error 
            + self.k_i * self.integral 
            + self.k_d * deriv;
        
        // Remember the previous error and time
        self.prev_error = Some(error);
        self.prev_time = Some(curr_time);

        // Return
        out
    }
}

impl TrajControllers {

    /// Create a new instance of the controllers from the parameters
    pub fn new(params: &super::Params) -> Self {
        Self {
            lat_ctrl: PidController::new(
                params.lat_k_p, params.lat_k_i, params.lat_k_d
            ),
            head_ctrl: PidController::new(
                params.head_k_p, params.head_k_i, params.head_k_d
            )
        }
    }

    /// Get the ackerman demand for the current path segment and pose.
    ///
    /// TODO: Add crab support
    pub fn get_ackerman_cmd(
        &mut self, 
        segment: &PathSegment, 
        pose: &Pose,
        report: &mut super::StatusReport,
        params: &super::Params
    ) -> MnvrCmd {

        // Calculate lateral error
        let lat_err_m = self.calc_lat_error(segment, pose);
        report.lat_error_m = lat_err_m;

        // Calcualte heading error
        let head_err_rad = self.calc_head_error(segment, pose);
        report.head_error_rad = head_err_rad;

        // Enforce limits on heading and lateral errors
        if lat_err_m > params.lat_error_limit_m {
            report.lat_error_limit_exceeded = true;
        }
        if head_err_rad > params.head_error_limit_rad {
            report.head_error_limit_exceeded = true;
        }

        // Pass the errors through the controllers
        let lat_curv_dem_m = self.lat_ctrl.get(lat_err_m);
        let head_curv_dem_m = self.head_ctrl.get(head_err_rad);

        // Sum the curvatures and apply limits
        let mut curv_dem_m = lat_curv_dem_m + head_curv_dem_m;

        if curv_dem_m > params.max_curv_dem_m {
            curv_dem_m = params.max_curv_dem_m;
        }
        if curv_dem_m < params.min_curv_dem_m {
            curv_dem_m = params.min_curv_dem_m;
        }

        // Calculate speed demand
        let mut speed_dem_ms = 0f64;
        for (i, c) in params.curv_speed_map_coeffs
            .iter()
            .enumerate() 
        {
            speed_dem_ms += 
                curv_dem_m.powi(
                    (params.curv_speed_map_coeffs.len() - 1 - i) 
                    as i32)
                * c;
        }

        // Apply speed limits
        if speed_dem_ms > params.max_speed_dem_ms {
            speed_dem_ms = params.max_speed_dem_ms
        }
        if speed_dem_ms < params.min_speed_dem_ms {
            speed_dem_ms = params.min_speed_dem_ms
        }

        MnvrCmd::Ackerman {
            speed_ms: speed_dem_ms,
            curv_m: curv_dem_m,
            crab_rad: 0.0
        }
    }

    /// Calculate the lateral error to the segment.
    fn calc_lat_error(
        &self,
        segment: &PathSegment,
        pose: &Pose
    ) -> f64 {
        // Get the slope and intercept of the line that passes through the 
        // rover's position and is perpendicular to the segment.
        let lat_slope_m = - 1f64 / segment.slope_m;
        let lat_intercept_m = pose.position_m_lm[1] 
            - lat_slope_m * pose.position_m_lm[0];

        // Find the point of intersection by equating the lines for the segment
        // and the lateral.
        // let mut isect_m_lm = [0f64; 2];
        // isect_m_lm[0] = (lat_intercept_m - segment.intercept_m)
        //     / (lat_slope_m - segment.slope_m);
        // isect_m_lm[1] = segment.slope_m * isect_m_lm[0] + segment.intercept_m;
        let isect_x = (lat_intercept_m - segment.intercept_m) / (lat_slope_m - segment.slope_m);
        let isect_m_lm = Vector2::new(
            isect_x,
            segment.slope_m * isect_x + segment.intercept_m
        );

        // Get 2D position vector of the rover
        let pos_m_lm = Vector2::new(
            pose.position_m_lm[0],
            pose.position_m_lm[1]
        );

        // Get the distance between them
        (isect_m_lm - pos_m_lm).norm()

        // The lateral error is then the distance between the intersection
        // and the rover.
        //
        // The unwrap here is safe as we are enforcing dimentions by taking
        // a slice of the position.
        
        // norm(&isect_m_lm, &pose.position_m_lm[0..1]).unwrap()
    }

    /// Calculate the heading error to the segment
    fn calc_head_error(
        &self,
        segment: &PathSegment,
        pose: &Pose
    ) -> f64 {
        
        // Get the heading of the segment.
        //
        // To do this we simply get the arctan of the segment slope.
        let seg_head_rad = segment.slope_m.atan();

        // Return the rover's heading - the segment heading
        pose.get_heading() - seg_head_rad
    }
}
