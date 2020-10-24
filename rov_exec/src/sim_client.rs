//! # Simulation Client
//!
//! The SimClient provides simulation data from Webots to the rover software. It is to be used for
//! testing and development of systems rather than actual driving of the rover. The SimClient
//! currently provides:
//!
//! - Rover pose in the world - `rov_pose_lm`.
//! - True depth map from the left camera view point - `left_depth_map`.
//!
//! Further data may be added to the client in the future.
//!
//! The data provided by the system works in a publisher-subscriber model. Data is sent by the
//! simulation server (SimServer) as frequently as it can. Data may be accessed by the rover
//! simulation by calling the `get_x` functions in this namespace, for instance
//! `sim_client::get_rov_pose_lm()` would retrieve the latest copy of `rov_pose_lm` that the client 
//! has.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::{sync::{Arc, atomic::AtomicBool, Mutex}, thread::JoinHandle};
use crate::loc::Pose;
use comms_if::{
    eqpt::cam::CamImage,
    net::NetParams,
};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

struct SimClient {
    bg_jh: Option<JoinHandle<()>>,
    bg_run: Arc<AtomicBool>,
    rov_pose_lm: Arc<Mutex<Pose>>,
    left_depth_map: Arc<Mutex<CamImage>>
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// FUNCTIONS
// ------------------------------------------------------------------------------------------------

pub fn init(rov_params: NetParams) {

}