//! # Mechanisms Server Module
//!
//! This module abstracts over the networking side of the mechanisms executable. The server accepts
//! connections from the client in the rover executable, allowing demands to be recieved from the
//! client and sensor data to be sent to the client.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use comms_if::{
    net::{zmq, MonitoredSocket, SocketOptions, MonitoredSocketError}, 
    eqpt::{MechDems, MechDemsResponse}
};
use log::warn;

use crate::params::MechExecParams;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// An abstraction over the networking part of the mechanisms executable.
///
/// The server accepts connections from the client in the rover executable, allowing demands to be 
/// recieved from the client and sensor data to be sent to the client.
///
/// TODO: Sensor data chain
pub struct MechServer {

    /// REP socket which accepts demands from the client
    dems_socket: MonitoredSocket,

    /// PUB socket which sends sensor data to the client
    _sens_socket: MonitoredSocket,
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Errors which can occur in the [`MechServer`]
#[derive(thiserror::Error, Debug)]
pub enum MechServerError {
    #[error("Socket error: {0}")]
    SocketError(MonitoredSocketError),

    // TODO: May not be needed?
    // #[error("Not connected to the client")]
    // NotConnected,

    #[error("Could not send data to the client: {0}")]
    SendError(zmq::Error)
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl MechServer {

    /// Create a new instance of the mechanisms server.
    ///
    /// This function will not wait for a connection from the client before returning.
    pub fn new(params: &MechExecParams) -> Result<Self, MechServerError> {

        // Create the zmq context
        let ctx = zmq::Context::new();

        // Create the socket options
        let dems_socket_options = SocketOptions {
            bind: true,
            block_on_first_connect: false,
            recv_timeout: 200,
            send_timeout: 10,
            ..Default::default()
        };
        let sens_socket_options =  SocketOptions {
            bind: true,
            block_on_first_connect: false,
            ..Default::default()
        };

        // Create the sockets
        let dems_socket = MonitoredSocket::new(
            &ctx, 
            zmq::REP,
            dems_socket_options, 
            &params.demands_endpoint
        )?;
        let sens_socket = MonitoredSocket::new(
            &ctx,
            zmq::PUB,
            sens_socket_options,
            &params.sensor_data_endpoint
        )?;

        // Create self
        Ok(Self {
            dems_socket,
            _sens_socket: sens_socket
        })
    }

    /// Retrieve a set of demands from the client.
    ///
    /// The user MUST call [`send_dems_response`] at the earliest opportunity in order to notify the
    /// client.
    ///
    /// `None` is returned if no valid demand is recieved. In this case the exec must stop the 
    /// mechanisms.
    pub fn get_demands(&mut self) -> Option<MechDems> {

        // Read from the socket
        let msg = self.dems_socket.recv_msg(0);

        match msg {
            Ok(m) => {
                match serde_json::from_str(m.as_str().unwrap_or("")) {
                    Ok(d) => Some(d),
                    Err(e) => {
                        warn!("Could not deserialize demands: {}", e);
                        None
                    }
                }
            },
            Err(_e) => {
                // warn!("Could not read from demands socket: {}", e);
                None
            }
        }
    }

    /// Send a response to the client based on the recieved demands.
    pub fn send_dems_response(
        &mut self, 
        response: &MechDemsResponse
    ) -> Result<(), MechServerError> {
        // TODO: This seems to hang the server, find out why
        // // If not connected return an error
        // if !self.dems_socket.connected() {
        //     return Err(MechServerError::NotConnected)
        // }

        // Serialize response
        let resp_str = serde_json::to_string(response)
            .expect("Response serialization failed. This should not happen");
        
        // Send response
        match self.dems_socket.send(&resp_str, 0) {
            Ok(_) => Ok(()),
            Err(e) => Err(MechServerError::SendError(e))
        }
    }
}

impl From<MonitoredSocketError> for MechServerError {
    fn from(e: MonitoredSocketError) -> Self {
        MechServerError::SocketError(e)
    }
}