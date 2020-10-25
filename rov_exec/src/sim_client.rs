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
//! simulation server (SimServer) as frequently as it can. Unlike other clients the network
//! interface is defined here, as the simulation server (written in python) cannot use a
//! rust-defined struct.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::{sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}}, thread::{self, JoinHandle}};
use conquer_once::Lazy;
use log::{error, warn};
use serde::Deserialize;

use crate::auto::loc::Pose;
use comms_if::{
    eqpt::cam::{CamFrame, CamImage}, 
    net::{MonitoredSocket, MonitoredSocketError, NetParams, SocketOptions, zmq}
};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

pub struct SimClient {
    bg_jh: Option<JoinHandle<()>>,
    bg_run: Arc<AtomicBool>,
    rov_pose_lm: Arc<Mutex<Option<Pose>>>,
    left_depth_map: Arc<Mutex<Option<CamImage>>>
}

// ------------------------------------------------------------------------------------------------
// GLOBALS
// ------------------------------------------------------------------------------------------------

static SIM_CLIENT: Lazy<Mutex<Option<SimClient>>> = Lazy::new(|| Mutex::new(None));

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum SimClientError {

    #[error("Socket error: {0}")]
    SocketError(MonitoredSocketError),

    #[error("The client is not connected to the server")]
    NotConnected,

    #[error("Could not send demands to the server: {0}")]
    SendError(zmq::Error),

    #[error("Could not recieve a message from the server: {0}")]
    RecvError(zmq::Error),

    #[error("Could not serialize the data: {0}")]
    SerializationError(serde_json::Error),

    #[error("Could not deserialize the response from the server: {0}")]
    DeserializeError(serde_json::Error),

    #[error("The SimClient has not been initialised")]
    NotInit

}

/// Data from the simulation server
#[derive(Debug, Deserialize)]
enum SimData {
    Pose {
        /// The position in the LM frame
        position_m_lm: [f64; 3],

        /// The attitude of the rover in the LM frame. This is a quaternion that 
        /// will rotate an object from the LM frame into the RB frame.
        attitude_q_lm: [f64; 4]
    },
    LeftDepthMap(CamFrame)
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl SimClient {
    /// Create a new instance of the SimClient.
    pub fn init(ctx: &zmq::Context, params: &NetParams) -> Result<(), SimClientError> {
                // Create the socket options
        // TODO: Move these into a parameter file
        let socket_options = SocketOptions {
            connect_timeout: 1000,
            heartbeat_ivl: 500,
            heartbeat_ttl: 1000,
            heartbeat_timeout: 1000,
            linger: 1,
            recv_timeout: 10,
            send_timeout: 10,
            ..Default::default()
        };

        // Connect the socket
        let socket = MonitoredSocket::new(
            ctx, 
            zmq::SUB, 
            socket_options, 
            &params.sim_endpoint
        ).map_err(|e| SimClientError::SocketError(e))?;

        // Create the data shared objects
        let bg_run = Arc::new(AtomicBool::new(true));
        let rov_pose_lm = Arc::new(Mutex::new(None));
        let left_depth_map = Arc::new(Mutex::new(None));

        // Create clones of these to pass to the bg thread
        let bg_run_clone = bg_run.clone();
        let rov_pose_lm_clone = rov_pose_lm.clone();
        let left_depth_map_clone = left_depth_map.clone();

        // Start BG thread
        let bg_jh = Some(thread::spawn(move || {
            bg_thread(
                socket,
                bg_run_clone,
                rov_pose_lm_clone,
                left_depth_map_clone
            )
        }));

        // Set the global
        *SIM_CLIENT.lock().expect("SIM_CLIENT mutex poisoned") = Some(Self {
            bg_jh,
            bg_run,
            rov_pose_lm,
            left_depth_map
        });

        // Return success
        Ok(())
    }

    /// Get the rover pose from the simulation.
    pub fn rov_pose_lm(&self) -> Option<Pose> {
        let rp = self.rov_pose_lm.lock()
            .expect("SimClient: rov_pose_lm mutex poisoned");

        return *rp
    }

    /// Get the left depth map from the simulation.
    pub fn left_depth_map(&self) -> Option<CamImage> {
        let ldm = self.left_depth_map.lock()
            .expect("SimClient: left_depth_map mutex poisoned");

        return (*ldm).clone()
    }
}

// ------------------------------------------------------------------------------------------------
// FUNCTIONS
// ------------------------------------------------------------------------------------------------

/// Get the rover pose from the simulation.
pub fn rov_pose_lm() -> Option<Pose> {
    match *SIM_CLIENT.lock().expect("SIM_CLIENT mutex poisoned") {
        Some(ref c) => c.rov_pose_lm(),
        None => None
    }
}

/// Get the left depth map from the simulation.
pub fn left_depth_map() -> Option<CamImage> {
    match *SIM_CLIENT.lock().expect("SIM_CLIENT mutex poisoned") {
        Some(ref c) => c.left_depth_map(),
        None => None
    }
}

/// Background thread, updates the data in the SimClient when the server publishes something new.
fn bg_thread(
    socket: MonitoredSocket,
    run: Arc<AtomicBool>,
    rov_pose_lm: Arc<Mutex<Option<Pose>>>,
    left_depth_map: Arc<Mutex<Option<CamImage>>>
) {

    // While instructed to run
    while run.load(Ordering::Relaxed) {
        // Read string from the socket
        let msg = match socket.recv_string(0) {
            Ok(Ok(s)) => s,
            Ok(Err(_)) => {
                warn!("Non UTF-8 message from SimServer");
                continue
            },
            Err(zmq::Error::EAGAIN) => continue,
            Err(e) => {
                error!("Error receiving message from SimServer: {:?}", e);
                break
            }
        };

        // Deserialize the message
        let data: SimData = match serde_json::from_str(&msg) {
            Ok(d) => d,
            Err(e) => {
                warn!("Error deserialising message from SimServer: {:?}", e);
                continue
            }
        };

        // Parse the data and set in front end
        match data {
            SimData::Pose {position_m_lm, attitude_q_lm} => {

                // Buid pose struct
                let pose = Pose {
                    position_m_lm,
                    attitude_q_lm
                };

                // Set the pose in the front end
                {
                    let mut rp = rov_pose_lm.lock()
                        .expect("SimClient: rov_pose_lm mutex poisoned");

                    *rp = Some(pose);
                }
                
            },
            SimData::LeftDepthMap(frame) => {
                // Convert the frame to an image
                let image = match frame.to_cam_image() {
                    Ok(i) => i,
                    Err(e) => {
                        warn!("Could not convert left_depth_map CamFrame to CamImage: {:?}", e);
                        continue
                    }
                };

                // Set the image in the front end
                {
                    let mut ldm = left_depth_map.lock()
                        .expect("SimClient: left_depth_map mutex poisoned");

                    *ldm = Some(image);
                }
            }
        }
    }
}