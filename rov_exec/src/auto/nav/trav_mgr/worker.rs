//! Worker thread to allow map computation etc to occur without blocking the main thread.

// -----------------------------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------------------------

use std::sync::{
    mpsc::{Receiver, Sender},
    Arc,
};

use comms_if::eqpt::perloc::DepthImage;
use log::warn;

use crate::auto::{
    loc::Pose,
    map::CostMap,
    nav::{
        trav_mgr::{EscapeBoundary, TraverseState},
        NavPose,
    },
};

use super::{Shared, TravMgrError};

// -----------------------------------------------------------------------------------------------
// ENUMS
// -----------------------------------------------------------------------------------------------

#[derive(Debug)]
pub enum WorkerSignal {
    /// The worker should stop it's operations
    Stop,

    /// A new depth image was acquired, plus the pose when the image was taken
    NewDepthImg(DepthImage, Pose),

    /// The previous depth image processing request has been completed
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
            WorkerSignal::NewDepthImg(img, pose) => {
                // Calculate the local terrain map from the depth image, we do this in a scope so
                // we can drop the per_mgr lock when we're done with it
                let local_terr_map = {
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

                // If there's a ground path apply it to the local cost map
                {
                    if let Some(ref path) = *shared.ground_path.read()? {
                        if let Err(e) = local_cost_map.apply_ground_planned_path(path) {
                            main_sender.send(WorkerSignal::Error(Box::new(
                                TravMgrError::CostMapError(e),
                            )))?;
                            continue;
                        }
                    }
                }

                // Calculate the escape boundary from the local cost map
                let esc_boundary = match EscapeBoundary::calculate(&local_cost_map) {
                    Ok(e) => e,
                    Err(e) => {
                        main_sender.send(WorkerSignal::Error(Box::new(e)))?;
                        continue;
                    }
                };

                // Merge the local maps into the global maps, doing each in a scope so we can drop
                // their locks when needed
                {
                    let mut gtm = shared.global_terr_map.write()?;
                    gtm.merge(&local_terr_map);
                }
                {
                    let mut gcm = shared.global_cost_map.write()?;
                    gcm.merge(&local_cost_map);
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

                    // Get the path planner and a read copy of the gcm
                    let path_planner = shared.path_planner.write()?;
                    let gcm = shared.global_cost_map.read()?;

                    // Plan the path(s)
                    let mut paths = match path_planner.plan_direct(
                        &gcm,
                        &start_pose,
                        &esc_boundary.min_cost_target,
                        num_paths,
                    ) {
                        Ok(p) => p,
                        Err(e) => {
                            main_sender
                                .send(WorkerSignal::Error(Box::new(TravMgrError::NavError(e))))?;
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
