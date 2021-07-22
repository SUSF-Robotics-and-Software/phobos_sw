//! Defines the escape boundary for a map.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use log::{error, trace};
use nalgebra::{Isometry2, Point2, Translation2, Unit, UnitComplex, Vector2};
use serde::{Deserialize, Serialize};
use util::session;

use crate::auto::{
    loc::Pose,
    map::{CostMap, CostMapData, CostMapLayer},
    nav::NavPose,
    path::Path,
};

use super::TravMgrError;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// An escape boundary, which defines the limit of where the rover has knowledge of the terrain
/// ahead of it.
///
/// This is modeled as an arc, centred on the pose where the rover took the depth image associated
/// with this boundary, extending a uniform radius out from the centre, and bounded by a minimum
/// and maximum heading angle.
///
/// While the escape boundary is calculated from the local cost map, all it's data is in the **global
/// map frame**.
#[derive(Debug, Clone)]
pub struct EscapeBoundary {
    /// The centre of the boundary (rover pose when depth image for this boundary was acquired)
    pub centre_m: NavPose,

    /// The radius of the boundary
    pub radius_m: f64,

    /// The minimum heading angle of the boundary
    pub min_head_rad: f64,

    /// The minimum heading angle of the boundary
    pub max_head_rad: f64,

    /// The path that describes the boundary
    pub path: Path,

    /// Target point along the escape boundary that has the lowest cost
    pub min_cost_target: NavPose,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EscapeBoundaryParams {
    /// Maximum radius of the escape boundary
    pub max_radius_m: f64,

    /// Minimum radius of the escape boundary
    pub min_radius_m: f64,

    /// The threshold used when performing the binary search for the largest possible radius of the
    /// escape boundary
    pub radius_threshold_m: f64,

    /// The amount to step the radius inwards when testing a new potential escape boundary/
    pub radius_step_m: f64,

    /// The safety factor to apply to the final radius, i.e. r = r_final / SF
    pub radius_safety_factor: f64,

    /// Maximum heading of the edges of the escape boundary
    pub max_heading_rad: f64,

    /// Minimum heading of the edges of the escape boundary
    pub min_heading_rad: f64,

    /// The threshold used when calculating the heading of the escape boundary
    pub heading_threshold_rad: f64,

    /// The safety factor to apply to the final headings, i.e. h = h_final / SF
    pub heading_safety_factor: f64,
}

struct TestBoundary {
    radius_m: f64,
    left_heading_rad: f64,
    right_heading_rad: f64,
    area_m2: f64,
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl EscapeBoundary {
    pub fn calculate(
        params: &EscapeBoundaryParams,
        local_cost_map: &CostMap,
        pose: &Pose,
    ) -> Result<Self, TravMgrError> {
        // General approach:
        //  - initialise an empty test boundary of radii, (min, max) heading pairs, and boundary
        //    areas
        //  - binary search for the starting radius, begining from the maximum radius and looking
        //    for the first cell which has a cost (including unsafe). if no valid start radius error
        //  - once start radius found iterate towards centre along x, at each radius perform binary
        //    search to find min/max angles:
        //      - shortcut: test (-max, max) angles first, if both are valid the boundary is at the
        //        maximum
        //      - if only one was valid do binary search for the other
        //      - if both were invalid do binary search starting from -max angle
        //      - if valid pair of endpoints is found calculate area of the boundary they create:
        //          - if the new area is larger than the current max area update current max
        //          - if not ignore this area and move to next radius
        //  - transform the boundary into cost map parent frame points
        //  - generate the path describing the boundary
        //  - iterate over the boundary path and find the minimum cost cell
        //  - set this cell's position as the target position, and calculate the normal to the
        //    boundary at this point to use that as the target heading.

        let mut current: Option<TestBoundary> = None;

        // Initialise the radius for the search to max_radius
        let mut radius_m = params.max_radius_m;
        let mut last_valid_radius_m = None;

        // start radius binary search
        let mut loops = 1;
        loop {
            let valid = is_point_valid(local_cost_map, radius_m, 0.0);
            trace!("centreline {} m valid?: {}", radius_m, valid);

            // See if we're at the end of the search
            let dist_to_max = params.max_radius_m - radius_m;
            let dist_to_min = radius_m - params.min_radius_m;
            let at_end = (dist_to_max < params.radius_threshold_m)
                || (dist_to_min < params.radius_threshold_m);

            // If valid and at end of search
            if valid && at_end {
                // Start radius found, break out of search
                break;
            }
            // Or if not valid and at end (but not on first loop)
            else if !valid && at_end && loops != 1 {
                // Start radius not found, error
                return Err(TravMgrError::EscBoundaryInvalidCentreline);
            }
            // If still searching
            else {
                // Check if the distance to move is less than the threshold
                let dist_to_move =
                    0.5 * (params.max_radius_m - params.min_radius_m) / (loops as f64);
                let keep_moving = dist_to_move > params.radius_threshold_m;

                // If the point is valid and we should move, move radius out
                if valid && keep_moving {
                    last_valid_radius_m = Some(radius_m);
                    radius_m += dist_to_move;
                }
                // If it wasn't valid and we should keep moving, move radius in
                else if !valid && keep_moving {
                    radius_m -= dist_to_move;
                }
                // If the point is valid and we shouldn't keep moving we have found the start
                // radius
                else if valid {
                    break;
                }
                // If we previously found a valid radius return that
                else if let Some(r) = last_valid_radius_m {
                    radius_m = r;
                    break;
                // Otherwise the centreline wasn't valid
                } else {
                    return Err(TravMgrError::EscBoundaryInvalidCentreline);
                }
            }

            loops += 1;
        }

        // iterate backwards from the start radius
        loop {
            // Initialise headings
            let mut left_heading_rad = params.max_heading_rad;
            let mut right_heading_rad = -params.max_heading_rad;

            // First shortcut and test validity of both endpoints
            let endpoints_valid = (
                is_point_valid(local_cost_map, radius_m, left_heading_rad),
                is_point_valid(local_cost_map, radius_m, right_heading_rad),
            );

            trace!("Endpoints valid: {:?}", endpoints_valid);

            // If a binary search is needed to find the endpoints do it
            let both_endpoints_valid = match endpoints_valid {
                // No search needed
                (true, true) => true,
                // Search starting from left being valid
                (true, false) => {
                    if let Some(head_rad) = search_for_endpoint(
                        local_cost_map,
                        radius_m,
                        right_heading_rad,
                        left_heading_rad,
                        params.heading_threshold_rad,
                    ) {
                        right_heading_rad = head_rad;
                        true
                    } else {
                        // Otherwise continue to the next radius
                        false
                    }
                }
                // Search starting from right being valid
                (false, true) => {
                    if let Some(head_rad) = search_for_endpoint(
                        local_cost_map,
                        radius_m,
                        left_heading_rad,
                        right_heading_rad,
                        params.heading_threshold_rad,
                    ) {
                        left_heading_rad = head_rad;
                        true
                    } else {
                        // Otherwise continue to the next radius
                        false
                    }
                }
                // Search for both left and right, starting with left
                (false, false) => {
                    // First the left
                    if let Some(head_rad) = search_for_endpoint(
                        local_cost_map,
                        radius_m,
                        left_heading_rad,
                        right_heading_rad,
                        params.heading_threshold_rad,
                    ) {
                        trace!("Got valid left heading");
                        left_heading_rad = head_rad;

                        // Then the right
                        if let Some(head_rad) = search_for_endpoint(
                            local_cost_map,
                            radius_m,
                            right_heading_rad,
                            left_heading_rad,
                            params.heading_threshold_rad,
                        ) {
                            trace!("Got valid right heading");
                            right_heading_rad = head_rad;
                            true
                        } else {
                            // Otherwise continue to the next radius
                            false
                        }
                    } else {
                        trace!("Couldn't find left heading");
                        // Otherwise continue to the next radius
                        false
                    }
                }
            };

            // If both endpoints could be found
            if both_endpoints_valid {
                // calculate the area of the boundary
                let test = get_test_boundary(radius_m, left_heading_rad, right_heading_rad);

                trace!(
                    "Valid EB: {}, ({}, {}), {}",
                    radius_m,
                    left_heading_rad,
                    right_heading_rad,
                    test.area_m2
                );

                // If the area is larger than the current max update it, if it's smaller than the
                // current boundary shortuct out of the iteration
                if let Some(ref c) = current {
                    if c.area_m2 < test.area_m2 {
                        trace!("New largest area");
                        current = Some(test);
                    } else {
                        trace!("New area smaller, exiting");
                        break;
                    }
                } else {
                    current = Some(test)
                }
            }

            // Change to next radius
            radius_m -= params.radius_step_m;

            // Check if we've reached the end of the iteration
            if radius_m < params.min_radius_m {
                break;
            }
        }

        // Return the maximum area boundary
        if let Some(boundary) = current {
            // Generate the points in the boundary (local map frame), separated by the threshold
            // angular distance, and add them to the path, accounting for safety factors
            let radius_m = boundary.radius_m / params.radius_safety_factor;
            let min_heading_rad_lm = boundary.right_heading_rad / params.heading_safety_factor;
            let max_heading_rad_lm = boundary.left_heading_rad / params.heading_safety_factor;
            let mut points_m_lm = Vec::new();
            let mut angle_rad = min_heading_rad_lm;

            while angle_rad < max_heading_rad_lm {
                points_m_lm.push(get_point_on_arc(radius_m, angle_rad));

                angle_rad += params.heading_threshold_rad;
            }

            // Save escape boundary path for debugging
            session::save(
                "eb_path.json",
                Path {
                    points_m: points_m_lm.iter().map(|p| p.coords).collect(),
                },
            );

            // Find min cost cell along that path
            let mut min_cost_cell = (Point2::new(0.0, 0.0), f64::MAX);
            trace!("{} points to test", points_m_lm.len());
            for i in 1..points_m_lm.len() {
                trace!("Testing {}", i);
                for ((_, pos), cost) in (*local_cost_map)
                    .line_iter(points_m_lm[i], points_m_lm[i - 1])
                    .map_err(|_| {
                        error!(
                            "One of {} or {} was outside the map",
                            points_m_lm[i],
                            points_m_lm[i - 1]
                        );
                        TravMgrError::EscapeBoundaryPointOutsideMap
                    })?
                    .layer(CostMapLayer::Total)
                    .positioned()
                {
                    // Ignore none or unsafe points
                    if let CostMapData::Cost(c) = cost {
                        if *c < min_cost_cell.1 {
                            trace!("New min cost cell at {}", pos);
                            min_cost_cell = (pos, *c);
                        }
                    }
                }
            }

            // Normalise the position, which will be the direction vector for the target
            let dir_vector_lm = Unit::new_normalize(min_cost_cell.0.coords);

            // Get the affine transform to the global map frame
            let lm_to_gm = Isometry2::from_parts(
                Translation2::from(pose.position2()),
                UnitComplex::from_angle(pose.get_heading()),
            );

            // Transform all data into global map frame
            let x_vec = Vector2::x();
            let centre_m_gm = lm_to_gm.transform_point(&Point2::origin());
            let heading_rad_gm = lm_to_gm.transform_vector(&x_vec).angle(&x_vec);
            let target_m_gm = lm_to_gm.transform_point(&min_cost_cell.0);
            let dir_vector_gm = lm_to_gm.transform_vector(&dir_vector_lm);
            let points_m_gm: Vec<_> = points_m_lm
                .iter()
                .map(|p_lm| lm_to_gm.transform_point(p_lm).coords)
                .collect();
            let min_heading_rad_gm = lm_to_gm
                .transform_vector(&Vector2::new(
                    min_heading_rad_lm.cos(),
                    min_heading_rad_lm.sin(),
                ))
                .angle(&x_vec);
            let max_heading_rad_gm = lm_to_gm
                .transform_vector(&Vector2::new(
                    max_heading_rad_lm.cos(),
                    max_heading_rad_lm.sin(),
                ))
                .angle(&x_vec);

            // Build the escape boundary
            Ok(EscapeBoundary {
                centre_m: NavPose::from_parts(&centre_m_gm, &heading_rad_gm),
                radius_m,
                min_head_rad: min_heading_rad_gm,
                max_head_rad: max_heading_rad_gm,
                path: Path {
                    points_m: points_m_gm,
                },
                min_cost_target: NavPose::from_parts(&target_m_gm, &dir_vector_gm.angle(&x_vec)),
            })
        } else {
            Err(TravMgrError::NoEscapeBoundary)
        }
    }
}

// ------------------------------------------------------------------------------------------------
// FUNCTIONS
// ------------------------------------------------------------------------------------------------

#[inline]
fn get_point_on_arc(radius_m: f64, angle_rad: f64) -> Point2<f64> {
    Point2::new(radius_m * angle_rad.cos(), radius_m * angle_rad.sin())
}

#[inline]
fn is_point_valid(local_cost_map: &CostMap, radius_m: f64, angle_rad: f64) -> bool {
    if let Some(idx) = local_cost_map.index(get_point_on_arc(radius_m, angle_rad)) {
        matches!(
            local_cost_map.get(CostMapLayer::Total, idx).unwrap(),
            CostMapData::Cost(_)
        )
    } else {
        false
    }
}

#[inline]
fn get_test_boundary(radius_m: f64, left_heading_rad: f64, right_heading_rad: f64) -> TestBoundary {
    TestBoundary {
        radius_m,
        left_heading_rad,
        right_heading_rad,
        area_m2: 0.5 * radius_m * radius_m * (left_heading_rad - right_heading_rad),
    }
}

/// Returns Some(heading) of first valid point starting from min_bound_rad, and going up to
/// max_bound_rad. Will attempt to get the point closest to min_bound_rad.
fn search_for_endpoint(
    local_cost_map: &CostMap,
    radius_m: f64,
    min_bound_rad: f64,
    max_bound_rad: f64,
    angle_threshold_rad: f64,
) -> Option<f64> {
    // Start at min
    let mut angle_rad = min_bound_rad;
    let mut last_valid_angle_rad = None;

    trace!(
        "Endpoint search, in range {}..{}",
        min_bound_rad,
        max_bound_rad
    );

    // Main search loop
    let mut loops = 1;
    loop {
        // Get if the current position is valid
        let valid = is_point_valid(local_cost_map, radius_m, angle_rad);

        // Calculate if we're at the end of the search
        let dist_to_max = max_bound_rad - angle_rad;
        let dist_to_min = angle_rad - min_bound_rad;
        let at_end =
            (dist_to_max.abs() < angle_threshold_rad) || (dist_to_min.abs() < angle_threshold_rad);

        trace!(
            "    {}: {}, {}, {}, {}",
            angle_rad,
            dist_to_min,
            dist_to_max,
            valid,
            at_end
        );

        // if valid and at the end we've found a point, return it
        if valid && at_end {
            return Some(angle_rad);
        }
        // if we're at the end but the point isn't valid we couldn't find one
        else if !valid && at_end && loops > 1 {
            return None;
        }
        // Otherwise calculate the distance we need to move
        else {
            // Check if the distance to move is less than the threshold
            let dist_to_move = 0.5 * (max_bound_rad - min_bound_rad) / (loops as f64);
            let keep_moving = dist_to_move.abs() > angle_threshold_rad;

            // If we should keep moving
            if keep_moving {
                // If valid step towards min
                if valid {
                    last_valid_angle_rad = Some(angle_rad);
                    angle_rad -= dist_to_move;
                }
                // Otherwise step towards max
                else {
                    angle_rad += dist_to_move;
                }
            }
            // If we could find the valid point and we shouldn't be moving return it
            else if valid {
                return Some(angle_rad);
            }
            // Or if we found a valid angle in the past
            else if let Some(angle_rad) = last_valid_angle_rad {
                return Some(angle_rad);
            }
            // If we shouldn't move (i.e. the step is less than the threshold) we couldn't find a
            // valid point
            else {
                return None;
            }
        }

        loops += 1;
    }
}
