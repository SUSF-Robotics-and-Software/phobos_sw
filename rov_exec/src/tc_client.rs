//! # Telecommand Client

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use comms_if::{
    net::{zmq, MonitoredSocket, SocketOptions, MonitoredSocketError}, 
    tc::{Tc, TcResponse}
};

use crate::params::RovExecParams;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// Telecommand client
pub struct TcClient {
    socket: MonitoredSocket
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum TcClientError {
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

    #[error("Could not parse the recieved telecommand")]
    TcParseError(serde_json::Error),

    #[error("The server sent a message which was not valid UTF-8")]
    NonUtf8Response
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl TcClient {

    /// Create a new instance of the TC Client.
    ///
    /// This function will not block until the server connects.
    pub fn new(ctx: &zmq::Context, params: &RovExecParams) -> Result<Self, TcClientError> {
        // Create the socket options
        // TODO: Move these into a parameter file
        let socket_options = SocketOptions {
            block_on_first_connect: false,
            connect_timeout: 1000,
            heartbeat_ivl: 500,
            heartbeat_ttl: 1000,
            heartbeat_timeout: 1000,
            linger: 1,
            recv_timeout: 10,
            send_timeout: 10,
            req_correlate: true,
            req_relaxed: false,
            ..Default::default()
        };

        // Connect the socket
        let socket = MonitoredSocket::new(
            ctx, 
            zmq::REP, 
            socket_options, 
            &params.tc_endpoint
        ).map_err(|e| TcClientError::SocketError(e))?;

        // Create self
        Ok(Self {
            socket
        })
    }

    /// Check if the client is connected to the server
    pub fn is_connected(&self) -> bool {
        self.socket.connected()
    }

    /// Recieve a single TC from the server.
    ///
    /// The protocol here is to call recieve_tc in a loop until `Ok(None)` is returned, indicating
    /// that there are no more pending TCs to be recieved. This does not mean that the server will
    /// not send another TC in the future, just that there are none to handle right now.
    ///
    /// After recieving a valid TC the client must send a response using `.send_response()` before
    /// attempting to recieve another TC. If an error occurs in receiving the TC the response will
    /// be sent automatically by this function.
    pub fn recieve_tc(&self) -> Result<Option<Tc>, TcClientError> {
        // Check the server is connected
        if !self.socket.connected() {
            return Err(TcClientError::NotConnected)
        }

        // Attempt to read a string from the socket
        let tc_str = match self.socket.recv_string(0) {
            // Valid message
            Ok(Ok(s)) => s,
            // Non UTF-8 message
            Ok(Err(_)) => {
                // Send invalid message response
                self.send_response(TcResponse::Invalid)?;

                return Err(TcClientError::NonUtf8Response)
            },
            // No message in timeout
            Err(zmq::Error::EAGAIN) => return Ok(None),
            // Recieve error
            Err(e) => {
                // No response is sent if we could not recieve
                return Err(TcClientError::RecvError(e))
            }
        };

        // Parse the TC
        Tc::from_json(&tc_str)
            .map_err(|e| {
                // Send the invalid response
                // TODO: add proper error handling here
                self.send_response(TcResponse::Invalid).ok();

                TcClientError::TcParseError(e)
            })
            .map(|t| Some(t))
    }

    /// Send the given response back to the server.
    ///
    /// This function must be called after recieving a TC.
    pub fn send_response(&self, response: TcResponse) -> Result<(), TcClientError> {
        // Check the server is connected
        if !self.socket.connected() {
            return Err(TcClientError::NotConnected)
        }

        // Serialise the response
        let response_str = serde_json::to_string(&response)
            .map_err(|e| TcClientError::SerializationError(e))?;

        // Send the response
        self.socket.send(&response_str, 0)
            .map_err(|e| TcClientError::SendError(e))
    }
}