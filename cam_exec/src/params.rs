//! # Camera Executable Parameters

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::Deserialize;

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Deserialize, Default)]
pub struct CamExecParams {

    /// Network endpoint for the camera server requests socket
    pub requests_endpoint: String,

    /// Width of the stream in pixels
    pub stream_width: u32,

    /// Height of the stream in pixels
    pub stream_height: u32,

    /// Linux device path for left nav camera
    pub left_nav_video_device: String,

    /// Linux device path for right nav camera
    pub right_nav_video_device: String
}