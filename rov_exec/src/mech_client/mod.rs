//! # Mechanisms Client
//!
//! This module provides networking abstractions to connect to the mechanisms server.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use comms_if::{
    net::{zmq, MonitoredSocket, SocketOptions, MonitoredSocketError}, 
    eqpt::{MechDems, MechSensData, MechDemsResponse}
};

use crate::params::RovExecParams;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

pub struct MechClient {
    dems_socket: MonitoredSocket,

    _sens_socket: MonitoredSocket
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(thiserror::Error, Debug)]
pub enum MechClientError {

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

}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl MechClient {
    /// Create a new instance of the mechanisms client.
    pub fn new(ctx: &zmq::Context, params: &RovExecParams) -> Result<Self, MechClientError> {
        
        // Create the socket options
        let dems_socket_options = SocketOptions {
            connect_timeout: 1000,
            heartbeat_ivl: 500,
            heartbeat_ttl: 1000,
            heartbeat_timeout: 1000,
            linger: 1,
            recv_timeout: 10,
            send_timeout: 10,
            req_correlate: true,
            req_relaxed: true,
            ..Default::default()
        };
        let sens_socket_options = SocketOptions {
            ..Default::default()
        };

        // Create the sockets
        let dems_socket = MonitoredSocket::new(
            ctx,
            zmq::REQ,
            dems_socket_options,
            &params.mech_dems_endpoint
        ).map_err(|e| MechClientError::SocketError(e))?;
        let sens_socket = MonitoredSocket::new(
            ctx,
            zmq::REQ,
            sens_socket_options,
            &params.mech_sens_endpoint
        ).map_err(|e| MechClientError::SocketError(e))?;

        // Create self
        Ok(Self {
            dems_socket,
            _sens_socket: sens_socket
        })
    }

    /// Send demands to the server.
    ///
    /// Sends the given mechanisms demands to the server. If the server acknowledges the demands
    /// within the configured timeout then `Ok()` is returned, otherwise an `Err()` is returned.
    pub fn send_demands(
        &mut self, 
        demands: &MechDems
    ) -> Result<MechDemsResponse, MechClientError> {
        // If not connected return now
        if !self.dems_socket.connected() {
            return Err(MechClientError::NotConnected)
        }

        // Serialize the demands
        let dems_str = serde_json::to_string(demands)
            .map_err(|e| MechClientError::SerializationError(e))?;

        // Send the demands to the server
        self.dems_socket.send(&dems_str, 0)
            .map_err(|e| MechClientError::SendError(e))?;
        
        // Recieve response back from the server
        let msg = self.dems_socket.recv_msg(0);

        match msg {
            Ok(m) => {
                serde_json::from_str(m.as_str().unwrap_or(""))
                    .map_err(|e| MechClientError::DeserializeError(e))
            },
            Err(e) => {
                Err(MechClientError::RecvError(e))
            }
        }
    }

    /// Get the latest sensor data message from the server.
    ///
    /// If no sensor data is available `None` is returned.
    /// TODO: implement
    pub fn get_sensor_data(&mut self) -> Option<MechSensData> {
        todo!("Not yet implemented")
    }
}