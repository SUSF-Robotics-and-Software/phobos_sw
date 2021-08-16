//! Plans minimum cost paths through a [`CostMap`], using an A* algorithm.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::collections::{BinaryHeap, HashMap};

use comms_if::tc::auto::PathSpec;
use log::{info, trace, warn};
use nalgebra::{Point2, Unit, UnitQuaternion, Vector2, Vector3};
use serde::{Deserialize, Serialize};

use crate::auto::{
    loc::Pose,
    map::{CostMap, CostMapData},
    path::{Path, PathError},
};

use super::{NavError, NavPose};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct PathPlanner {
    params: PathPlannerParams,
}

#[derive(Debug, Clone, Deserialize)]
pub struct PathPlannerParams {
    /// Possible curvatures of path to assess, in 1/meters.
    pub test_curvs_m: Vec<f64>,

    /// Possible heading changes (i.e. delta from current heading) to assess, in radians.
    pub test_heads_rad: Vec<f64>,

    /// The separation between points within a path.
    pub path_point_separation_m: f64,

    /// Weight of remaining straight line cost to the target when calculating the heuristic.
    pub heuristic_remaining_cost_weight: f64,

    /// Weight of the alignment cost to the target pose when calculating the heuristic.
    pub heuristic_alignment_cost_weight: f64,

    /// Tolerance for a path being at the target position
    pub target_tolerance_m: f64,

    /// The maximum possible individual path length
    pub max_path_length_m: f64,

    /// The minimum possible individual path length
    pub min_path_length_m: f64,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PathPlannerReport {
    pub num_tested_paths: usize,

    pub target: NavPose,

    pub tree: NodeTree,

    pub result: Option<Vec<Path>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeTree {
    /// None if parent
    pub node: Option<Node>,
    pub children: Vec<NodeTree>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PathCost {
    pub raw_cost: f64,
    pub heuristic: f64,
}

/// An A* node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    pub id: usize,
    pub parent_id: usize,

    /// The number of parent paths this node has
    pub depth: usize,

    /// Node could be start in which case it has no path
    pub path: Path,

    /// Nodes always have a cost, the start node's cost is based on the heuristic to the target.
    pub cost: PathCost,
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl PathPlanner {
    pub fn new(params: PathPlannerParams) -> Self {
        Self { params }
    }

    /// Plans a minimum cost path towards the target position, attempting to finish the path facing
    /// in the target heading.
    ///
    /// This function will attempt to plan a direct path between the start and target pose,
    /// producing the parameterised number of paths, and automatically determining the length of
    /// each path. Use [`PathPlanner::plan_indirect`] to avoid this automatic limitation of the
    /// number and length of paths.
    pub fn plan_direct(
        &self,
        cost_map: &CostMap,
        start_pose: &NavPose,
        target_pose: &NavPose,
        num_paths: usize,
    ) -> Result<Vec<Path>, NavError> {
        // Get the distance between the start and end poses
        let total_dist_m = (target_pose.position_m - start_pose.position_m).norm();

        // Get the length of each path
        let path_length_m = total_dist_m / (num_paths as f64);

        self.plan(
            cost_map,
            start_pose,
            target_pose,
            path_length_m,
            Some(num_paths),
        )
    }

    /// Plans a minimum cost path towards the target position, attempting to finish the path facing
    /// in the target heading.
    ///
    /// This function will attempt to plan an indirect path between the start and target pose,
    /// producing a variable number of paths of the given length. Use [`PathPlanner::plan_direct`]
    /// to automatically produce a predetermined number of paths.
    pub fn plan_indirect(
        &self,
        cost_map: &CostMap,
        start_pose: &NavPose,
        target_pose: &NavPose,
        path_length_m: f64,
    ) -> Result<Vec<Path>, NavError> {
        self.plan(cost_map, start_pose, target_pose, path_length_m, None)
    }

    /// Internal planning function, independent of direct/indirect route.
    fn plan(
        &self,
        cost_map: &CostMap,
        start_pose: &NavPose,
        target_pose: &NavPose,
        path_length_m: f64,
        num_paths: Option<usize>,
    ) -> Result<Vec<Path>, NavError> {
        // Check both start and end poses are in the map
        if cost_map.index(start_pose.position_m).is_none() {
            return Err(NavError::PointOutsideMap(
                "PathPlanner::plan::start_pose".into(),
                start_pose.position_m,
            ));
        }
        if cost_map.index(target_pose.position_m).is_none() {
            return Err(NavError::PointOutsideMap(
                "PathPlanner::plan::target_pose".into(),
                target_pose.position_m,
            ));
        }

        // Report which we will build as we process, then save
        let mut report = PathPlannerReport {
            num_tested_paths: 0,
            target: *target_pose,
            tree: NodeTree {
                node: None,
                children: vec![],
            },
            result: None,
        };

        // Create the priority queue (binary heap) for tracking nodes
        let mut heap = BinaryHeap::new();

        // Create the hashmap for tracking visited nodes
        let mut visited: HashMap<usize, Node> = HashMap::new();

        // Counter to create node ids, with start node being 0, which is actually not in the heap.
        let mut num_nodes = 1;

        // Set to true if the target is reached
        let mut target_reached = false;

        // Limit the path length to the max path length
        let path_length_m =
            path_length_m.clamp(self.params.min_path_length_m, self.params.max_path_length_m);

        // Start by placing into the heap the paths fanning out from the start pose
        for (fan_id, path) in self
            .get_path_fan(start_pose, path_length_m)
            .map_err(NavError::CouldNotBuildFan)?
            .iter()
            .enumerate()
        {
            // Compute the cost for this path
            let cost = match self.get_path_cost(cost_map, path, target_pose, None) {
                Some(c) => c,
                None => {
                    // warn!("Fan path {} of node {} is untraversable", fan_id, 0);
                    continue;
                }
            };

            // Build the node, all parents here are node 0, the start
            let node = Node {
                id: num_nodes,
                parent_id: 0,
                depth: 0,
                path: path.clone(),
                cost,
            };

            // Add the node to the heap
            heap.push(node.clone());

            // Add the node to the report tree
            report.tree.children.push(NodeTree {
                node: Some(node),
                children: vec![],
            });
            report.num_tested_paths += 1;

            num_nodes += 1;
        }

        // Main loop
        while !target_reached {
            // Get the minimum cost node, if there's nothing in the heap break out
            let min_node = match heap.pop() {
                Some(n) => n,
                None => break,
            };

            // Calculate the distance to the target
            let dist_to_target =
                Point2::from(min_node.path.points_m[min_node.path.get_num_points() - 1])
                    - target_pose.position_m;

            // trace!("Dist to target = {}", dist_to_target.norm());

            // If we're within the threshold to the target we're there, so we can exit.
            if dist_to_target.norm() <= self.params.target_tolerance_m {
                target_reached = true;
                break;
            }

            // If we're not within the target threshold, extend this path with a new fan.
            //
            // If we are limited on the number of paths and this path is at the maximum depth don't
            // extend this one.
            let extend_path = match num_paths {
                Some(n) => min_node.depth < n - 1,
                None => true,
            };

            if extend_path {
                let path_end_pose =
                    NavPose::from_path_last_point(&min_node.path).ok_or(NavError::EmptyPath)?;
                for path in self
                    .get_path_fan(&path_end_pose, path_length_m)
                    .map_err(NavError::CouldNotBuildFan)?
                {
                    // Compute the cost for this path in the fan
                    let cost = match self.get_path_cost(cost_map, &path, target_pose, None) {
                        Some(c) => c,
                        None => {
                            continue;
                        }
                    };

                    // Build the fan's node
                    let node = Node {
                        id: num_nodes,
                        parent_id: min_node.id,
                        depth: min_node.depth + 1,
                        path: path.clone(),
                        cost,
                    };

                    // Add the node to the heap
                    heap.push(node.clone());

                    // Add the node to the report
                    let parent = report
                        .tree
                        .get_by_id(node.parent_id)
                        .expect("Couldn't find parent node in report tree");
                    parent.children.push(NodeTree {
                        node: Some(node),
                        children: vec![],
                    });
                    report.num_tested_paths += 1;

                    num_nodes += 1;
                }
            }

            // Put the old min cost node in the visited map
            visited.insert(min_node.id, min_node);
        }

        // Work backwards from the current min node to get the paths, if the target was reached we
        // need to find the min cost from the heap. If it wasn't we have to go through the visited
        // map to find the one with the lowest cost.
        let mut min_node = if target_reached {
            match heap.peek() {
                Some(n) => n,
                None => {
                    unreachable!("Expected there to be a minimum node in the heap but there wasnt!")
                }
            }
        } else {
            match visited.iter().max_by(|a, b| a.1.cmp(b.1)).map(|(_, v)| v) {
                Some(n) => n,
                None => return Err(NavError::NoPathToTarget),
            }
        };

        let mut paths = vec![min_node.path.clone()];

        // Record the index of the lowest cost path
        let mut lowest_cost_idx = 0;
        let mut lowest_cost = f64::MAX;

        while min_node.parent_id != 0 {
            min_node = match visited.get(&min_node.parent_id) {
                Some(n) => n,
                None => {
                    unreachable!("Couldn't find parent node in visited list!")
                }
            };

            // Check for lowest cost
            if min_node.cost.total() < lowest_cost {
                lowest_cost = min_node.cost.total();
                lowest_cost_idx = paths.len();
            }

            paths.push(min_node.path.clone());
        }

        // Remove all paths before the lowest cost index, since the planner may actually
        // overestimate some times, don't do this if the lowest cost index is 0
        if lowest_cost_idx != 0 {
            paths = paths
                .iter()
                .enumerate()
                .filter_map(|(i, p)| {
                    if i < lowest_cost_idx - 1 {
                        None
                    } else {
                        Some(p.clone())
                    }
                })
                .collect();
        }

        // Reverse the path list to get one that goes from the start to the target.
        paths.reverse();

        report.result = Some(paths.clone());

        // Write the report out
        util::session::save_with_timestamp("path_planner/report.json", report);

        // if the target wasn't reached we'll warn that we're choosing the best fit, and return it
        // in the error
        if !target_reached {
            warn!("Couldn't get within tolerance of target, choosing best fit instead");
            Err(NavError::BestPathNotAtTarget(paths))
        } else {
            info!("Target reached");
            Ok(paths)
        }
    }

    /// Get the fan of potential paths from the given start pose as a vector of paths.
    fn get_path_fan(
        &self,
        start_pose: &NavPose,
        path_length_m: f64,
    ) -> Result<Vec<Path>, PathError> {
        let mut fan = Vec::new();
        for head_rad in self.params.test_heads_rad.iter() {
            let pose = Pose::new(
                start_pose.pose_parent.position_m,
                start_pose.pose_parent.attitude_q
                    * UnitQuaternion::from_axis_angle(
                        &Unit::new_normalize(Vector3::z()),
                        *head_rad,
                    ),
            );
            for curv_m in self.params.test_curvs_m.iter() {
                // Build path spec
                let spec = PathSpec::AckSeq {
                    separation_m: self.params.path_point_separation_m,
                    seq: vec![*curv_m, path_length_m],
                };

                // Build the path
                fan.push(Path::from_path_spec(spec, &pose)?);
            }
        }

        Ok(fan)
    }

    /// Returns the cost for the given path towards the target pose.
    ///
    /// Returns `None` if the path isn't traversable.
    fn get_path_cost(
        &self,
        cost_map: &CostMap,
        path: &Path,
        target_pose: &NavPose,
        parent_cost: Option<&PathCost>,
    ) -> Option<PathCost> {
        // Get the raw cost of the path
        let raw_cost = match cost_map.get_path_cost(path) {
            Ok(CostMapData::Cost(c)) => c,
            Ok(_) => {
                // warn!("Untraversable cost {:?}", c);
                return None;
            }
            Err(e) => {
                warn!("Couldn't calculate path cost: {}", e);
                return None;
            }
        };

        // Get the NavPose at the end of the path, by getting the heading in the last segment.
        let last_pose = NavPose::from_path_last_point(path)?;

        // Check the pose is in the map
        if cost_map.index(last_pose.position_m).is_none() {
            warn!("Path endpoint is outside the map");
            return None;
        }

        // Calculate the heuristic
        let heuristic = self.get_heuristic(
            cost_map,
            &Point2::from(path.points_m[0]),
            &last_pose,
            target_pose,
            raw_cost / path.get_length().unwrap(),
        );

        // Compute the path cost based on the parent cost, if there is one
        if let Some(parent_cost) = parent_cost {
            Some(PathCost {
                raw_cost: parent_cost.raw_cost + raw_cost,
                heuristic,
            })
        } else {
            Some(PathCost {
                raw_cost,
                heuristic,
            })
        }
    }

    /// Calculates the heuristic for the end_pose, aiming to move between the start_pose and
    /// target_pose.
    /// Calculate heuristic, which needs to acount for:
    ///  - Estimated cost between the end of the path and the target, assuing straight line
    ///    path.
    ///  - The heading error to the target pose.
    ///
    /// For the first part we try to calculate the cost of a straight line path between the end of
    /// the path and the target. If this straight line isn't traversable (i.e. it's unsafe or
    /// unpopulated) we estimate the cost as the provided average multiplied by the distance to the
    /// target.
    ///
    /// The second part requires calculating the forward vector of target_pose, and the forward
    /// vector of last_pose, taking the dot product and normalising by their lengths to get a
    /// value from -1 to 1 of how aligned the two vectors are. If we then take 1 - that we get a
    /// value from 0 to 2, with 0 being aligned, 1 being right angles, and 2 being anti-aligned.
    ///
    /// We can then sum these two parts of the hueristic based on the weights given in the
    /// params.
    ///
    /// So that the two parts of the heuristic are of the same order of magnitude we multiply
    /// the alignment cost by the remaining distance cost, then apply the weighting.
    ///
    ///
    fn get_heuristic(
        &self,
        cost_map: &CostMap,
        start_position: &Point2<f64>,
        end_pose: &NavPose,
        target_pose: &NavPose,
        avg_cost_per_m: f64,
    ) -> f64 {
        // Get remaining path cost
        let remaining_path_cost =
            match cost_map.get_cost_between_points(end_pose.position_m, target_pose.position_m) {
                Ok(CostMapData::Cost(c)) => c,
                _ => {
                    // trace!("No concrete cost to target, estimating from average path cost instead");
                    let dist = (target_pose.position_m - end_pose.position_m).norm();
                    avg_cost_per_m * dist
                }
            };

        // Calculate vectors between the start and target, and the last pose and target.
        let target_vec: Vector2<f64> = target_pose.position_m - *start_position;
        let last_vec: Vector2<f64> = target_pose.position_m - end_pose.position_m;

        // Calculate the alignment value
        let alignment_cost =
            1.0 - (last_vec.dot(&target_vec) / (target_vec.norm() * last_vec.norm()));

        // Multiply alignment cost by the path cost so that we get values on the same order of
        // magnitude, and include weightings
        remaining_path_cost
            * (self.params.heuristic_remaining_cost_weight
                + (self.params.heuristic_alignment_cost_weight * alignment_cost))
    }
}

impl PathCost {
    pub fn total(&self) -> f64 {
        self.raw_cost + self.heuristic
    }
}

impl PartialEq for PathCost {
    fn eq(&self, other: &Self) -> bool {
        self.total().eq(&other.total())
    }
}

impl Eq for PathCost {}

impl Ord for PathCost {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(&other).expect("Unexpected NaN path cost")
    }
}

impl PartialOrd for PathCost {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        // Note that we flip the order here so that the heap will be a min-heap, not a max-heap
        other.total().partial_cmp(&self.total())
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for Node {}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.cost.cmp(&other.cost)
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.cost.partial_cmp(&other.cost)
    }
}

impl NodeTree {
    pub fn get_by_id(&mut self, id: usize) -> Option<&mut NodeTree> {
        if let Some(ref n) = self.node {
            if n.id == id {
                return Some(self);
            }
        }

        if self.children.is_empty() {
            return None;
        }

        for child in &mut self.children {
            if let Some(n) = child.get_by_id(id) {
                return Some(n);
            }
        }

        None
    }
}
