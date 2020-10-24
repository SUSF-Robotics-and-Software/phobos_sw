//! # Camera Client
//!
//! The camera client requests and recieves images back from the camera server

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::collections::HashMap;

use comms_if::{
    net::{zmq, MonitoredSocket, SocketOptions, MonitoredSocketError}, 
    eqpt::cam::*
};

use crate::params::RovExecParams;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

/// The camera client
pub struct CamClient {
    socket: MonitoredSocket,

    awaiting_response: bool
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum CamClientError {

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
        response to the last request")]
    WaitingForResponse,

    #[error("Cannot recieve a response as no request has been made")]
    NoRequestMade,

    #[error("Error while deserialising data from {0:?}: {1}")]
    ImageDeserError(CamId, image::ImageError),

    #[error("The server responed with a message which was not valid UTF-8")]
    NonUtf8Response

}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl CamClient {
    /// Create a new instance of the camera client
    pub fn new(ctx: &zmq::Context, params: &RovExecParams) -> Result<Self, CamClientError> {
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
            req_correlate: true,
            req_relaxed: true,
            ..Default::default()
        };

        // Connect the socket
        let socket = MonitoredSocket::new(
            ctx, 
            zmq::REQ, 
            socket_options, 
            &params.cam_endpoint
        ).map_err(|e| CamClientError::SocketError(e))?;

        // Create self
        Ok(Self {
            socket,
            awaiting_response: false
        })
    }

    /// Send request for images.
    ///
    /// Sending a request while still waiting on the response to a previous request will result in
    /// an error.
    pub fn request_frames(
        &mut self, 
        cameras: Vec<CamId>, 
        format: ImageFormat
    ) -> Result<(), CamClientError> {
        // If not connected return an error
        // TODO: Reset the await flag?
        if !self.socket.connected() {
            return Err(CamClientError::NotConnected)
        }

        // If still waiting return an error
        if self.awaiting_response {
            return Err(CamClientError::WaitingForResponse)
        }

        // Build the request
        let request = CamRequest {
            cameras,
            format
        };

        // Serialize the request
        let request_str = serde_json::to_string(&request)
            .map_err(|e| CamClientError::SerializationError(e))?;

        // Send the request
        self.socket.send(&request_str, 0)
            .map_err(|e| CamClientError::SendError(e))?;

        // Set the awaiting response flag
        self.awaiting_response = true;

        Ok(())
    }

    /// Receive the frames in response to a request.
    ///
    /// Returns a hashmap of `CamId`s to `CamFrames`s, or `None` if no response was recieved within
    /// the client's `recv_timeout`.
    ///
    /// Receiving images while not awaiting a response to a request will result in an error.
    pub fn recieve_frames(&mut self) -> Result<Option<HashMap<CamId, CamFrame>>, CamClientError> {
        // If not connected return an error
        // TODO: Reset the await flag?
        if !self.socket.connected() {
            return Err(CamClientError::NotConnected)
        }

        // If not waiting for a response return an error
        if !self.awaiting_response {
            return Err(CamClientError::NoRequestMade)
        }

        // Read message from the server
        let response_str = match self.socket.recv_string(0) {
            // Valid response
            Ok(Ok(s)) => s,
            // Invalid response
            Ok(Err(_)) => return Err(CamClientError::NonUtf8Response),
            // No response
            Err(zmq::Error::EAGAIN) => return Ok(None),
            // Recv error
            Err(e) => return Err(CamClientError::RecvError(e))
        };

        // Unset the awaiting response flag
        self.awaiting_response = false;

        // Deserialize the response
        Ok(serde_json::from_str(&response_str)
            .map_err(|e| CamClientError::DeserializeError(e))?
        )
    }

    /// Recieve the images in response to a request.
    ///
    /// Returns a hashmap of `CamId`s to `CamImage`s, or `None` if no response was recieved within
    /// the client's `recv_timeout`.
    ///
    /// Receiving images while not awaiting a response to a request will result in an error.
    pub fn recieve_images(&mut self) -> Result<Option<HashMap<CamId, CamImage>>, CamClientError> {
        // First recieve frames
        let frames = match self.recieve_frames()? {
            Some(f) => f,
            None => return Ok(None)
        };

        // Map each frame into an image
        let mut images = HashMap::<CamId, CamImage>::new();
        for (id, frame) in frames {
            images.insert(
                id, 
                frame.to_cam_image()
                    .map_err(|e| CamClientError::ImageDeserError(id, e))?
            );
        }

        Ok(Some(images))
    }
}
