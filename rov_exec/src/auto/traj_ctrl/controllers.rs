//! # Trajectory controllers module
//!
//! This module provides the PID controllers used for TrajCtrl, including their
//! error calculations.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use std::time::Instant;
use log::debug;
use nalgebra::{Vector2, Vector3};
use serde::Serialize;

// Internal
use crate::auto::{
    loc::Pose,
    path::*
};
use comms_if::tc::loco_ctrl::MnvrCmd;

use super::TrajCtrlTuningOutput;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A PID controller
#[derive(Debug, Serialize, Clone)]
pub struct PidController {
    /// Previous instant that the error was passed in 
    #[serde(skip)]
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
#[derive(Debug, Serialize, Clone)]
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
    pub fn get_ackerman_cmd(
        &mut self, 
        segment: &PathSegment, 
        pose: &Pose,
        report: &mut super::StatusReport,
        tuning_output: &mut super::TrajCtrlTuningOutput,
        params: &super::Params
    ) -> MnvrCmd {

        // Calculate lateral error
        let lat_err_m = self.calc_lat_error(segment, pose);
        report.lat_error_m = lat_err_m;
        tuning_output.lat_error_m = lat_err_m;

        // Calcualte heading error
        let head_err_rad = self.calc_head_error(segment, pose);
        report.head_error_rad = head_err_rad;
        tuning_output.head_error_rad = head_err_rad;

        // Enforce limits on heading and lateral errors
        if lat_err_m.abs() > params.lat_error_limit_m {
            report.lat_error_limit_exceeded = true;
        }
        if head_err_rad.abs() > params.head_error_limit_rad {
            report.head_error_limit_exceeded = true;
        }

        // Pass the errors through the controllers. The lateral error controlls the crab demand and
        // the heading controlls the curvature
        let mut crab_dem_rad = self.lat_ctrl.get(lat_err_m);
        let mut curv_dem_m = self.head_ctrl.get(head_err_rad);

        // Apply limits to curv and crab demands
        crab_dem_rad = crab_dem_rad.clamp(params.min_crab_dem_rad, params.max_crab_dem_rad);
        curv_dem_m = curv_dem_m.clamp(params.min_curv_dem_m, params.max_curv_dem_m);

        tuning_output.lat_ctrl = crab_dem_rad;
        tuning_output.head_ctrl = curv_dem_m;

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
            crab_rad: crab_dem_rad
        }
    }

    /// Calculate the lateral error to the segment.
    ///
    /// Lateral error will be positive if the rover is to the "left" of the segment, and negative
    /// if it's to the right (following right hand rule).
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
        let isect_x = (lat_intercept_m - segment.intercept_m) / (segment.slope_m - lat_slope_m);
        let isect_m_lm = Vector2::new(
            isect_x,
            segment.slope_m * isect_x + segment.intercept_m
        );

        // Get 2D position vector of the rover
        let pos_m_lm = Vector2::new(
            pose.position_m_lm[0],
            pose.position_m_lm[1]
        );

        // We can get which side of the segment the rover is on by using the cross product of the
        // start->end and rov->end vectors. +ve cross is left, -ve is right
        let cross = Vector3::new(
            segment.direction[0],
            segment.direction[1],
            0.0
        ).cross(&Vector3::new(
            pos_m_lm[0] - segment.start_m[0],
            pos_m_lm[1] - segment.start_m[1],
            0.0
        ));

        // If the signs of left and rov side match the rover is on the left, otherwise it is on the
        // right, so multiplying by both signnums of signs will do this.
        //
        // Get the distance between them
        (isect_m_lm - pos_m_lm).norm() * cross[2].signum()
    }

    /// Calculate the heading error to the segment
    ///
    /// The heading error is +ve if the rover is pointing to the right of the segment, and negative
    /// if it's pointing to the left (right hand rule about Z)
    fn calc_head_error(
        &self,
        segment: &PathSegment,
        pose: &Pose
    ) -> f64 {
        
        // We can find the heading error by calculating the angle from the dot product and the sign
        // by computing the cross between the vector defining the segment and the vector defining
        // the rover heading. We have to extend this into 3D because there's no cross in 3D, but
        // the vectors are just zero on the z.
        // Get the vector pointing in the pose direction (in 2D)
        let pose_dir = pose.forward2();

        // Heading error is then the angle between the two direction vectors
        let head_err_rad = segment.direction.angle(&pose_dir);

        // Compute the extended cross product
        let cross = Vector3::new(
            pose_dir[0],
            pose_dir[1],
            0.0
        ).cross(&Vector3::new(
            segment.direction[0],
            segment.direction[1],
            0.0
        ));

        // Multiply the heading error by the sign of the cross product
        head_err_rad * cross[2].signum()
    }
}

// -----------------------------------------------------------------------------------------------
// FUNCTIONS
// -----------------------------------------------------------------------------------------------

fn side(start: Vector2<f64>, end: Vector2<f64>, point: Vector2<f64>) -> f64 {
    // Using
    // https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
    // we can determine which side of the segment the point is on.
    (point[0] - start[0])*(end[1] - start[1])
    -
    (point[1] - start[0])*(end[0] - start[0])
}