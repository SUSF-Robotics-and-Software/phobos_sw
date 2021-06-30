//! # Perloc Client
//!
//! The perloc client requests and recieves depth images and streams localisation data from the
//! perloc exec (Perception-Localisation).

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::convert::TryInto;

use comms_if::{
    eqpt::{
        cam::*,
        perloc::{DepthImage, PerlocCmd, PerlocError, PerlocRep, PerlocServerError},
    },
    net::{zmq, MonitoredSocket, MonitoredSocketError, NetParams, SocketOptions},
};
use log::info;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// The perloc client
pub struct PerlocClient {
    /// Request-response socket for perloc commands and responses
    reqrep: MonitoredSocket,

    // /// Localisation stream subscriber socket
    // TODO
    // loc_stream: MonitoredSocket,
    /// True if waiting for a response on reqrep.
    awaiting_response: bool,
}

#[derive(Debug, thiserror::Error)]
pub enum PerlocClientError {
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

    #[error(
        "Could not make another request to the server since the client is still waiting for the \
        response to the last request"
    )]
    WaitingForResponse,

    #[error("Cannot recieve a response as no request has been made")]
    NoRequestMade,

    #[error("Error while deserialising depth image: {0}")]
    ImageDeserError(PerlocError),

    #[error("The server responed with a message which was not valid UTF-8")]
    NonUtf8Response,

    #[error("Expected a PerlocRep::DepthFrame response, got {0:?} instead.")]
    ExpectedFrame(PerlocRep),

    #[error("Perloc server error: {0}")]
    PerlocServerError(PerlocServerError),
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl PerlocClient {
    pub fn new(ctx: &zmq::Context, params: &NetParams) -> Result<Self, PerlocClientError> {
        // Create socket options for the reqrep socket
        let reqrep_opts = SocketOptions {
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

        // TODO: Create socket options for the localisation stream socket

        // Connect the sockets
        let reqrep = MonitoredSocket::new(ctx, zmq::REQ, reqrep_opts, &params.perloc_endpoint)
            .map_err(PerlocClientError::SocketError)?;

        Ok(Self {
            reqrep,
            awaiting_response: false,
        })
    }

    /// Request a depth image from the perloc system
    pub fn request_depth_img(&mut self) -> Result<(), PerlocClientError> {
        // If not connected return an error
        // TODO: Reset the await flag?
        if !self.reqrep.connected() {
            return Err(PerlocClientError::NotConnected);
        }

        // If still waiting return an error
        if self.awaiting_response {
            return Err(PerlocClientError::WaitingForResponse);
        }

        // Build the request
        let request = PerlocCmd::AcqDepthFrame;

        // Serialize the request
        let request_str =
            serde_json::to_string(&request).map_err(PerlocClientError::SerializationError)?;

        // Send the request
        self.reqrep
            .send(&request_str, 0)
            .map_err(PerlocClientError::SendError)?;

        // Set the await flag
        self.awaiting_response = true;

        Ok(())
    }

    /// Get the depth image returned by perloc.
    ///
    /// If the image hasn't been acquired yet `Ok(None)` is returned.
    ///
    /// Receiving an image while not awaiting a response to a request will result in an error. You
    /// should call [`PerlocClient::request_depth_img`] before calling this function.
    pub fn receive_depth_img(&mut self) -> Result<Option<DepthImage>, PerlocClientError> {
        // If not connected return an error
        // TODO: Reset the await flag?
        if !self.reqrep.connected() {
            return Err(PerlocClientError::NotConnected);
        }

        // If not waiting for a response return an error
        if !self.awaiting_response {
            return Err(PerlocClientError::NoRequestMade);
        }

        // Read message from the server
        let response_str = match self.reqrep.recv_string(0) {
            // Valid response
            Ok(Ok(s)) => s,
            // Invalid response
            Ok(Err(_)) => return Err(PerlocClientError::NonUtf8Response),
            // No response
            Err(zmq::Error::EAGAIN) => return Ok(None),
            // Recv error
            Err(e) => return Err(PerlocClientError::RecvError(e)),
        };

        // Unset the awaiting response flag
        self.awaiting_response = false;

        // Deserialize the response
        let response: PerlocRep =
            serde_json::from_str(&response_str).map_err(PerlocClientError::DeserializeError)?;

        // Check that the response is a `DepthFrame` object
        let frame = match response {
            PerlocRep::DepthFrame(f) => f,
            r => return Err(PerlocClientError::ExpectedFrame(r)),
        };

        // Convert the frame into an image
        Ok(Some(
            frame
                .try_into()
                .map_err(PerlocClientError::ImageDeserError)?,
        ))
    }
}
