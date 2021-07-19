//! Defines the escape boundary for a map.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use nalgebra::Point2;
use serde::{Deserialize, Serialize};

use crate::auto::{
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

    /// The threshold used when calculating the radius of the escape boundary
    pub radius_threshold_m: f64,

    /// Maximum heading of the edges of the escape boundary
    pub max_heading_rad: f64,

    /// Minimum heading of the edges of the escape boundary
    pub min_heading_rad: f64,

    /// The threshold used when calculating the heading of the escape boundary
    pub heading_threshold_m: f64,
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

        // start radius binary search
        let mut loops = 1;
        loop {
            let valid = is_point_valid(local_cost_map, radius_m, 0.0);

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
                return Err(TravMgrError::NoEscapeBoundary);
            }
            // If still searching
            else {
                // Check if the distance to move is less than the threshold
                let dist_to_move =
                    0.5 * (params.max_radius_m - params.min_radius_m) / (loops as f64);
                let keep_moving = dist_to_move > params.radius_threshold_m;

                // If the point is valid and we should move, move radius out
                if valid && keep_moving {
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
                // Otherwise
                else {
                    return Err(TravMgrError::EscBoundaryInvalidCentreline);
                }
            }

            loops += 1;
        }

        // iterate backwards from the start radius
        loop {
            // Initialise headings
            let mut left_heading_rad = -params.max_heading_rad;
            let mut right_heading_rad = params.max_heading_rad;

            // First shortcut and test validity of both endpoints
            let endpoints_valid = (
                is_point_valid(local_cost_map, radius_m, left_heading_rad),
                is_point_valid(local_cost_map, radius_m, right_heading_rad),
            );

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
                        params.heading_threshold_m,
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
                        params.heading_threshold_m,
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
                        params.heading_threshold_m,
                    ) {
                        left_heading_rad = head_rad;

                        // Then the right
                        if let Some(head_rad) = search_for_endpoint(
                            local_cost_map,
                            radius_m,
                            right_heading_rad,
                            left_heading_rad,
                            params.heading_threshold_m,
                        ) {
                            right_heading_rad = head_rad;
                            true
                        } else {
                            // Otherwise continue to the next radius
                            false
                        }
                    } else {
                        // Otherwise continue to the next radius
                        false
                    }
                }
            };

            // If both endpoints could be found
            if both_endpoints_valid {
                // calculate the area of the boundary
                let test = get_test_boundary(radius_m, left_heading_rad, right_heading_rad);

                // If the area is larger than the current max update it, if it's smaller than the
                // current boundary shortuct out of the iteration
                if let Some(ref c) = current {
                    if c.area_m2 < test.area_m2 {
                        current = Some(test);
                    } else {
                        break;
                    }
                } else {
                    current = Some(test)
                }
            }

            // Increment the radius
            radius_m -= params.radius_threshold_m;

            // Check if we've reached the end of the iteration
            if radius_m < params.min_radius_m {
                break;
            }
        }

        // Return the maximum area boundary
        if let Some(boundary) = current {
            // Generate the points in the boundary (local map frame), separated by the threshold
            // angular distance, and add them to the path
            let mut points_m = Vec::new();
            let mut angle_rad = boundary.right_heading_rad;

            while angle_rad < boundary.left_heading_rad {
                points_m.push(get_point_on_arc(boundary.radius_m, angle_rad));
            }

            // Find min cost cell along that path
            let mut min_cost_cell = (Point2::new(0.0, 0.0), f64::MAX);
            for i in 1..points_m.len() {
                for ((_, pos), cost) in (*local_cost_map)
                    .line_iter(points_m[i], points_m[i - 1])?
                    .layer(CostMapLayer::Total)
                    .positioned()
                {
                    match cost {
                        CostMapData::Cost(c) => {
                            if c < min_cost_cell.1 {
                                min_cost_cell = (pos, c);
                            }
                        }
                        _ => (),
                    }
                }
            }

            // Normalise the position, which will be the direction vector for the target
            let dir_vector_lm = min_cost_cell.0.norm
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
            CostMapData::Cost(_) | CostMapData::Unsafe
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
        area_m2: 0.5 * radius_m * radius_m * (left_heading_rad + right_heading_rad),
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
    // Start at the midpoint
    let mut angle_rad = 0.5 * (min_bound_rad + max_bound_rad);

    // Main search loop
    let mut loops = 1;
    loop {
        // Get if the current position is valid
        let valid = is_point_valid(local_cost_map, radius_m, angle_rad);

        // Calculate if we're at the end of the search
        let dist_to_max = max_bound_rad - angle_rad;
        let dist_to_min = angle_rad - min_bound_rad;
        let at_end = (dist_to_max < angle_threshold_rad) || (dist_to_min < angle_threshold_rad);

        // if valid and at the end we've found a point, return it
        if valid && at_end {
            return Some(angle_rad);
        }
        // if we're at the end but the point isn't valid we couldn't find one
        else if !valid && at_end {
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
                    angle_rad -= dist_to_move;
                }
                // Otherwise step towards max
                else {
                    angle_rad += dist_to_move;
                }
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
