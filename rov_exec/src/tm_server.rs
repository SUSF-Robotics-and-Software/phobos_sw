//! # TM Server

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------
use serde::{Serialize, Deserialize};

use comms_if::{eqpt::{cam::{CamFrame, ImageFormat}, mech::MechDems}, net::{MonitoredSocket, MonitoredSocketError, NetParams, SocketOptions, zmq}, tc::{Tc, TcParseError, TcResponse}};

use crate::data_store::DataStore;

use crate::loco_ctrl;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Telemetry server
pub struct TmServer {
    socket: MonitoredSocket
}

/// Telemetry packet that is output by the server.
#[derive(Debug, Serialize, Deserialize)]
pub struct TmPacket {
    pub sim_time_s: f64,

    pub left_cam_frame: Option<CamFrame>,

    pub right_cam_frame: Option<CamFrame>,

    pub safe: bool,

    pub safe_cause: String,

    pub loco_ctrl_output: MechDems,

    pub loco_ctrl_status_rpt: loco_ctrl::StatusReport,

    pub arm_ctrl_output: MechDems,
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum TmServerError {
    #[error("Socket error: {0}")]
    SocketError(MonitoredSocketError),

    #[error("Could not send telemetry: {0}")]
    SendError(zmq::Error),

    #[error("Could not serialize the telemetry: {0}")]
    SerializationError(serde_json::Error),
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl TmServer {
    /// Create a new instance of the TM Server.
    ///
    /// This function will not block until the server connects.
    pub fn new(ctx: &zmq::Context, params: &NetParams) -> Result<Self, TmServerError> {
        // Create the socket options
        // TODO: Move these into a parameter file
        let socket_options = SocketOptions {
            block_on_first_connect: false,
            bind: true,
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
            zmq::PUB,
            socket_options,
            &params.tm_endpoint
        ).map_err(|e| TmServerError::SocketError(e))?;

        // Create self
        Ok(Self {
            socket
        })
    }

    pub fn send(&mut self, ds: &DataStore) -> Result<(), TmServerError> {
        // Build packet
        let packet = TmPacket::from_datastore(ds);

        // Serialize packet
        let packet_string = serde_json::to_string(&packet)
            .map_err(|e| TmServerError::SerializationError(e))?;

        // Send the packet
        self.socket.send(&format!("{}", packet_string), 0)
            .map_err(|e| TmServerError::SendError(e))
    }
}

impl TmPacket {
    pub fn from_datastore(ds: &DataStore) -> Self {
        Self {
            sim_time_s: ds.sim_time_s,
            safe: ds.safe,
            safe_cause: ds.safe_cause_string.clone(),
            loco_ctrl_output: ds.loco_ctrl_output.clone(),
            loco_ctrl_status_rpt: ds.loco_ctrl_status_rpt.clone(),
            arm_ctrl_output: ds.arm_ctrl_output.clone(),

            left_cam_frame: match ds.left_cam_image {
                Some(ref i) => {
                    let frame = i.to_cam_frame(ImageFormat::Jpeg(75)).unwrap();
                    Some(frame)
                },
                None => None
            },
            right_cam_frame: match ds.right_cam_image {
                Some(ref i) => {
                    let frame = i.to_cam_frame(ImageFormat::Jpeg(75)).unwrap();
                    Some(frame)
                },
                None => None
            },
        }
    }
}
