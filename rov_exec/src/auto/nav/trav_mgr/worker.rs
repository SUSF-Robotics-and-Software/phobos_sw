//! Worker thread to allow map computation etc to occur without blocking the main thread.

// -----------------------------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------------------------

use std::sync::{
    mpsc::{Receiver, Sender},
    Arc,
};

use comms_if::eqpt::perloc::DepthImage;
use log::{debug, warn};
use util::session;

use crate::auto::{
    loc::Pose,
    map::CostMap,
    nav::{
        trav_mgr::{escape_boundary::EscapeBoundary, TraverseState},
        NavPose,
    },
    path::Path,
};

use super::{Shared, TravMgrError};

// -----------------------------------------------------------------------------------------------
// ENUMS
// -----------------------------------------------------------------------------------------------

#[derive(Debug)]
pub enum WorkerSignal {
    /// The worker should stop it's operations
    Stop,

    /// A new depth image was acquired, plus the pose when the image was taken, and if this image
    /// is associated with a kickstart or not.
    NewDepthImg(Box<DepthImage>, Pose, bool),

    /// Recompute the global cost map from the global terrain map, optionally applying the wrapped
    /// ground planned path
    RecalcGlobalCost(Option<Path>),

    /// The global cost and terrain maps have been updated
    GlobalMapsUpdated,

    /// The requested work has been completed
    Complete,

    /// Unhandlable error
    Error(Box<TravMgrError>),
}

// -----------------------------------------------------------------------------------------------
// FUNCTIONS
// -----------------------------------------------------------------------------------------------

pub(super) fn worker_thread(
    shared: Arc<Shared>,
    main_sender: Sender<WorkerSignal>,
    main_reciever: Receiver<WorkerSignal>,
) -> Result<(), TravMgrError> {
    // Wait for commands from main
    while let Ok(signal) = main_reciever.recv() {
        // Process the signal
        match signal {
            WorkerSignal::Stop => break,
            WorkerSignal::RecalcGlobalCost(ground_path) => {
                let gtm = shared.global_terr_map.read()?;
                let mut gcm = shared.global_cost_map.write()?;

                let mut map = match CostMap::calculate(shared.cost_map_params.clone(), &gtm) {
                    Ok(c) => c,
                    Err(e) => {
                        main_sender
                            .send(WorkerSignal::Error(Box::new(TravMgrError::CostMapError(e))))?;
                        continue;
                    }
                };

                if let Some(gpp) = ground_path {
                    match map.apply_ground_planned_path(&gpp) {
                        Ok(_) => (),
                        Err(e) => {
                            main_sender.send(WorkerSignal::Error(Box::new(
                                TravMgrError::CostMapError(e),
                            )))?;
                            continue;
                        }
                    }
                }

                *gcm = map;

                main_sender.send(WorkerSignal::Complete)?;
            }
            WorkerSignal::NewDepthImg(img, pose, is_kickstart) => {
                let nav_pose = NavPose::from_parent_pose(&pose);

                // Calculate the local terrain map from the depth image, we do this in a scope so
                // we can drop the per_mgr lock when we're done with it
                let mut local_terr_map = {
                    let per_mgr = shared.per_mgr.write()?;

                    // Calculate the local terrain map
                    match per_mgr.calculate(&img, &pose) {
                        Ok(t) => t,
                        Err(e) => {
                            main_sender
                                .send(WorkerSignal::Error(Box::new(TravMgrError::PerError(e))))?;
                            continue;
                        }
                    }
                };

                session::save_with_timestamp("local_terr_map/ltm.json", local_terr_map.clone());

                // Compute the new local cost map
                let mut local_cost_map =
                    match CostMap::calculate(shared.cost_map_params.clone(), &local_terr_map) {
                        Ok(c) => c,
                        Err(e) => {
                            main_sender.send(WorkerSignal::Error(Box::new(
                                TravMgrError::CostMapError(e),
                            )))?;
                            continue;
                        }
                    };

                // If there's a ground path apply it to the local cost map, since the gpp is in the
                // GM frame we must move the local cost map into that frame first, this isn't too
                // bad since move is actually a very cheap operation.
                {
                    if let Some(ref path) = *shared.ground_path.read()? {
                        // Move lcm
                        local_cost_map.move_map(&nav_pose);

                        // Apply gpp
                        if let Err(e) = local_cost_map.apply_ground_planned_path(path) {
                            main_sender.send(WorkerSignal::Error(Box::new(
                                TravMgrError::CostMapError(e),
                            )))?;
                            continue;
                        }

                        // Move lcm back to it's origin
                        local_cost_map.move_map(&NavPose::default());
                    }
                }

                session::save_with_timestamp("local_cost_map/lcm.json", local_cost_map.clone());

                // Calculate the escape boundary from the local cost map if there is a ground path
                // follow
                let esc_boundary = {
                    if shared.ground_path.read()?.is_some() {
                        match EscapeBoundary::calculate(
                            &shared.params.escape_boundary,
                            &local_cost_map,
                            &pose,
                        ) {
                            Ok(e) => Some(e),
                            Err(e) => {
                                main_sender.send(WorkerSignal::Error(Box::new(e)))?;
                                continue;
                            }
                        }
                    } else {
                        None
                    }
                };

                // Save escape boundary for debugging
                if let Some(ref eb) = esc_boundary {
                    session::save_with_timestamp("escape_boundaries/eb.json", eb.clone());
                }

                // Merge the local maps into the global maps, doing each in a scope so we can drop
                // their locks when needed.
                {
                    // First move the local map into the global map frame with the given pose
                    local_terr_map.move_map(&nav_pose);

                    // Get the lock on the global map
                    let mut gtm = shared.global_terr_map.write()?;

                    // Merge them
                    gtm.merge(&local_terr_map);

                    // Save the updated global terrain map
                    session::save_with_timestamp("global_terr_map/gtm.json", gtm.clone());
                }
                {
                    // Move the cost map
                    local_cost_map.move_map(&nav_pose);

                    // Get the lock on the global
                    let mut gcm = shared.global_cost_map.write()?;

                    // Merge them
                    gcm.merge(&local_cost_map);

                    // Save the updated global cost map
                    session::save_with_timestamp("global_cost_map/gcm.json", gcm.clone());
                }

                // Send signal that the maps have been updated
                main_sender.send(WorkerSignal::GlobalMapsUpdated)?;

                // If this is a kickstart we end here
                if is_kickstart {
                    main_sender.send(WorkerSignal::Complete)?;
                    continue;
                }

                // Plan paths
                //
                // This has to be done in one of two ways, either both paths at once, i.e. when
                // we're starting a traverse, or from the end of the primary when we're traversing.
                {
                    // Read the traverse state to decide which plan format to use, do this in a
                    // scope so we drop the lock immediately
                    let trav_state = { *shared.traverse_state.read()? };

                    // We can parameterise this based on the start point and number of paths, no
                    // need to have large blocks in the match
                    let (start_pose, num_paths) = if matches!(trav_state, TraverseState::FirstStop)
                    {
                        (NavPose::from_parent_pose(&pose), 2)
                    } else {
                        let start_pose = match *shared.primary_path.read()? {
                            Some(ref p) => NavPose::from_path_last_point(p),
                            None => {
                                main_sender.send(WorkerSignal::Error(Box::new(
                                    TravMgrError::NoPrimaryPath,
                                )))?;
                                continue;
                            }
                        };
                        (start_pose, 1)
                    };

                    // Get the target, either the global GOTO or from the escape boundary for a
                    // ground planned path
                    let target = match esc_boundary {
                        Some(ref eb) => eb.min_cost_target,
                        None => match *shared.global_target.read()? {
                            Some(ref t) => *t,
                            None => {
                                main_sender.send(WorkerSignal::Error(Box::new(
                                    TravMgrError::NoValidTarget,
                                )))?;
                                continue;
                            }
                        },
                    };

                    // Get the path planner and a read copy of the gcm
                    let path_planner = shared.path_planner.write()?;
                    let gcm = shared.global_cost_map.read()?;

                    // Plan the path(s)
                    let mut paths =
                        match path_planner.plan_direct(&gcm, &start_pose, &target, num_paths) {
                            Ok(p) => p,
                            Err(e) => {
                                main_sender.send(WorkerSignal::Error(Box::new(
                                    TravMgrError::NavError(e),
                                )))?;
                                continue;
                            }
                        };

                    // Put the secondary path into the shared data. We do this before the primary
                    // potential secondary path since we can use pop to take the last element out.
                    *shared.secondary_path.write()? = match paths.pop() {
                        Some(p) => Some(p),
                        None => {
                            main_sender.send(WorkerSignal::Error(Box::new(
                                TravMgrError::PathPlannerFailed,
                            )))?;
                            continue;
                        }
                    };

                    // If there should be a primary path pop it
                    if matches!(trav_state, TraverseState::FirstStop) {
                        *shared.primary_path.write()? = match paths.pop() {
                            Some(p) => Some(p),
                            None => {
                                main_sender.send(WorkerSignal::Error(Box::new(
                                    TravMgrError::PathPlannerFailed,
                                )))?;
                                continue;
                            }
                        };
                    }

                    // Signal completion of the planning operation to the main thread
                    main_sender.send(WorkerSignal::Complete)?;
                };
            }
            _ => warn!("Unexpected signal from main thread: {:?}", signal),
        }
    }

    Ok(())
}
