//! # Trajectory controllers module
//!
//! This module provides the PID controllers used for TrajCtrl, including their
//! error calculations.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use log::trace;
// External
use nalgebra::{Vector2, Vector3};
use serde::Serialize;
use std::time::Instant;

// Internal
use crate::auto::{loc::Pose, path::*};
use comms_if::tc::loco_ctrl::MnvrCmd;

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
    integral: f64,
}

/// The trajectory controllers
#[derive(Debug, Serialize, Clone)]
pub struct TrajControllers {
    /// Lateral error controller
    lat_ctrl: PidController,

    /// Heading error controller
    head_ctrl: PidController,
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl PidController {
    /// Create a new controller with the given gains.
    pub fn new(k_p: f64, k_i: f64, k_d: f64) -> Self {
        Self {
            k_p,
            k_i,
            k_d,
            integral: 0f64,
            prev_time: None,
            prev_error: None,
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
        let dt = self.prev_time.map(|t0| (curr_time - t0).as_secs_f64());

        // Accumulate the integral term.
        //
        // If there's no time difference then we don't accumulate the integral
        // The other option is to add on the error and that will produce a
        // large spike in integral compared to normal operation, so we don't do
        // this.
        self.integral += match dt {
            Some(t) => error * t,
            None => 0f64,
        };

        // Calculate the derivative.
        //
        // If there's no time difference again we assume no derivative, for the
        // same reasons as for integral.
        let deriv = match self.prev_error {
            Some(e) => match dt {
                Some(t) => (error - e) / t,
                None => 0f64,
            },
            None => match dt {
                Some(t) => error / t,
                None => 0f64,
            },
        };

        // Calculate the output
        let out = self.k_p * error + self.k_i * self.integral + self.k_d * deriv;

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
            lat_ctrl: PidController::new(params.lat_k_p, params.lat_k_i, params.lat_k_d),
            head_ctrl: PidController::new(params.head_k_p, params.head_k_i, params.head_k_d),
        }
    }

    /// Get the ackerman demand for the current path segment and pose.
    pub fn get_ackerman_cmd(
        &mut self,
        segment: &PathSegment,
        pose: &Pose,
        report: &mut super::StatusReport,
        tuning_output: &mut super::TrajCtrlTuningOutput,
        params: &super::Params,
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
        for (i, c) in params.curv_speed_map_coeffs.iter().enumerate() {
            speed_dem_ms +=
                curv_dem_m.powi((params.curv_speed_map_coeffs.len() - 1 - i) as i32) * c;
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
            crab_rad: crab_dem_rad,
        }
    }

    /// Calculate the lateral error to the segment.
    ///
    /// Lateral error will be positive if the rover is to the "left" of the segment, and negative
    /// if it's to the right (following right hand rule).
    fn calc_lat_error(&self, segment: &PathSegment, pose: &Pose) -> f64 {
        // Use the triangular formula from
        // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        let err = (((segment.target_m.x - segment.start_m.x)
            * (segment.start_m.y - pose.position_m.y))
            - ((segment.start_m.x - pose.position_m.x) * (segment.target_m.y - segment.start_m.y)))
            .abs()
            / segment.length_m;

        // We can get which side of the segment the rover is on by using the cross product of the
        // start->end and rov->end vectors. +ve cross is left, -ve is right
        let cross =
            Vector3::new(segment.direction.x, segment.direction.y, 0.0).cross(&Vector3::new(
                pose.position_m.x - segment.start_m.x,
                pose.position_m.y - segment.start_m.y,
                0.0,
            ));

        // If the signs of left and rov side match the rover is on the left, otherwise it is on the
        // right, so multiplying by both signnums of signs will do this.
        //
        // Get the distance between them
        -err * cross[2].signum()
    }

    /// Calculate the heading error to the segment
    ///
    /// The heading error is +ve if the rover is pointing to the right of the segment, and negative
    /// if it's pointing to the left (right hand rule about Z)
    fn calc_head_error(&self, segment: &PathSegment, pose: &Pose) -> f64 {
        // We can find the heading error by calculating the angle from the dot product and the sign
        // by computing the cross between the vector defining the segment and the vector defining
        // the rover heading. We have to extend this into 3D because there's no cross in 3D, but
        // the vectors are just zero on the z.
        // Get the vector pointing in the pose direction (in 2D)
        let pose_dir = pose.forward2();

        // Heading error is then the angle between the two direction vectors
        let head_err_rad = segment.direction.angle(&pose_dir);

        // Compute the extended cross product
        let cross = Vector3::new(pose_dir[0], pose_dir[1], 0.0).cross(&Vector3::new(
            segment.direction[0],
            segment.direction[1],
            0.0,
        ));

        // Multiply the heading error by the sign of the cross product
        head_err_rad * cross[2].signum()
    }
}
