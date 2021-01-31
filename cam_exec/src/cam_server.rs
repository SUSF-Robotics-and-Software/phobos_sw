//! # CamServer module
//!
//! Provides a unified CamServer interface for the cam_exec.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use comms_if::{
    net::{zmq, MonitoredSocket, SocketOptions, MonitoredSocketError}, 
    eqpt::cam::*
};
use log::warn;

use crate::params::CamExecParams;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// An abstraction over the networking part of the camera executable.
///
/// The server accepts connections from the client in the rover executable, allowing requests to be 
/// recieved from the client.
pub struct CamServer {
    /// REP socket which accepts requests from the client
    socket: MonitoredSocket,
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Errors which can occur in the [`CamServer`]
#[derive(thiserror::Error, Debug)]
pub enum CamServerError {
    #[error("Socket error: {0}")]
    SocketError(MonitoredSocketError),

    // TODO: May not be needed?
    // #[error("Not connected to the client")]
    // NotConnected,

    #[error("Could not send data to the client: {0}")]
    SendError(zmq::Error)
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl CamServer {
    
    /// Create a new instance of the camera server.
    ///
    /// This function will not wait for a connection from the client before returning.
    pub fn new(params: &CamExecParams) -> Result<Self, CamServerError> {

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

        // Create the socket
        let socket = MonitoredSocket::new(
            &ctx, 
            zmq::REP,
            dems_socket_options, 
            &params.requests_endpoint
        )?;

        // Create self
        Ok(Self {
            socket
        })
    }

    /// Get a request from the client if one is available.
    pub fn get_request(&mut self) -> Option<CamRequest> {

        // Read from the socket
        let msg = self.socket.recv_msg(0);

        match msg {
            Ok(m) => {
                match serde_json::from_str(m.as_str().unwrap_or("")) {
                    Ok(d) => Some(d),
                    Err(e) => {
                        warn!("Could not deserialize request: {}", e);
                        None
                    }
                }
            },
            Err(_e) => {
                // warn!("Could not read from socket: {}", e);
                None
            }
        }
    }

    /// Send the given response to the client.
    pub fn send_response(&mut self, response: &CamResponse) -> Result<(), CamServerError> {
        // Serialize response
        let resp_str = serde_json::to_string(response)
            .expect("Response serialization failed. This should not happen");
        
        // Send response
        match self.socket.send(&resp_str, 0) {
            Ok(_) => Ok(()),
            Err(e) => Err(CamServerError::SendError(e))
        }
    }
}

impl From<MonitoredSocketError> for CamServerError {
    fn from(e: MonitoredSocketError) -> Self {
        CamServerError::SocketError(e)
    }
}