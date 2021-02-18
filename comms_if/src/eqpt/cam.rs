//! # Camera Equipment Communications Module

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use chrono::{DateTime, Utc, serde::ts_milliseconds};
use image::{DynamicImage, ImageResult};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------



/// A request for an invidual frame from one or more cameras.
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct FrameRequest {
    /// List of cameras to acquire a frame from
    pub cameras: Vec<CamId>,

    /// Format of the images to acquire
    pub format: ImageFormat
}

/// Settings that can be used to create camera streams for use by the operator.
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct StreamSettings {
    /// The camera to stream, or None to disable the stream.
    pub camera: Option<CamId>,

    /// Address of the target to stream to, in (Host IP, Port) format.
    pub target_addr: (String, String)
}

/// An individual frame from a camera
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CamFrame {

    /// UTC timestamp at which the frame was acquired
    #[serde(with = "ts_milliseconds")]
    pub timestamp: DateTime<Utc>,

    /// The format of this frame
    pub format: ImageFormat,

    /// The formatted image data, encoded in base64.
    pub b64_data: String
}

#[derive(Clone)]
pub struct CamImage {
    /// UTC timestamp at which the frame was acquired
    pub timestamp: DateTime<Utc>,

    /// The image itself
    pub image: DynamicImage
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

/// Request to be sent by the camera client to the server
#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum CamRequest {
    /// Request an individual frame from the cameras to be sent directly back
    /// to the client.
    FrameRequest(FrameRequest),

    /// Request to setup camera stream
    StreamSettingsRequest(StreamSettings),
}

/// Possible responses from the camera server to the client
#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum CamResponse {
    /// A selection of CamFrames for the given cameras.
    Frames(HashMap<CamId, CamFrame>),

    /// Indicates that a StreamSettings request was OK.
    StreamSettingsAccepted,

    /// Indicates that a StreamSettings request was rejected.
    StreamSettingsRejected
}

/// Cameras available on the rover
#[derive(Debug, Serialize, Deserialize, Copy, Clone, Hash, Eq, PartialEq)]
pub enum CamId {
    /// The left navigation camera
    LeftNav,

    /// The right navigation cameras
    RightNav,
}

/// Possible formats for camera images. This is used rather than image::ImageFormat to:
///     1. Restrict the formats that can be sent back and forth
///     2. Allow serialisation as image::ImageFormat does not implement serde.
#[derive(Debug, Serialize, Deserialize, Copy, Clone)]
pub enum ImageFormat {
    /// PNG image
    Png,

    /// JPEG image with a quality value between 1 and 100, where 100 is best.
    Jpeg(u8)
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl CamFrame {
    /// Convert this camera frame into a camera image
    pub fn to_cam_image(&self) -> image::ImageResult<CamImage>  {
        // Decode the image from base64 to raw bytes
        // TODO: support decode failure
        let raw_data = match base64::decode(self.b64_data.clone()) {
            Ok(v) => v,
            Err(_) => panic!("Decode error!")
        };

        // Convert the data
        let image = match self.format {
            ImageFormat::Png => 
                image::load_from_memory_with_format(
                    &raw_data, 
                    image::ImageFormat::Png
                )?,
            ImageFormat::Jpeg(_) =>
                image::load_from_memory_with_format(
                    &raw_data, 
                    image::ImageFormat::Jpeg
                )?
        };

        Ok(CamImage {
            timestamp: self.timestamp,
            image
        })
    }

    /// Convert an `image::DynamicImage` to a `CamFrame`.
    pub fn from_dyn_image(
        image: DynamicImage, 
        target_format: ImageFormat, 
        timestamp: DateTime<Utc>
    ) -> ImageResult<CamFrame> {
        // Create the CamImage
        let cam_image = CamImage {
            timestamp,
            image
        };

        // Format it into a frame
        cam_image.to_cam_frame(target_format)
    }
}

impl CamImage {
    /// Convert this camera image into a camera frame with the given format
    pub fn to_cam_frame(&self, format: ImageFormat) -> ImageResult<CamFrame> {
        // Write data to the buffer
        let mut data = Vec::<u8>::new();

        // Get the output format type
        let output_format = match format {
            ImageFormat::Png => image::ImageOutputFormat::Png,
            ImageFormat::Jpeg(q)  => image::ImageOutputFormat::Jpeg(q)
        };

        self.image.write_to(&mut data, output_format)?;

        // Return the frame
        Ok(CamFrame {
            timestamp: self.timestamp,
            format,
            b64_data: base64::encode(data)
        })
    }
}

impl Default for StreamSettings {
    fn default() -> Self {
        Self {
            camera: Some(CamId::LeftNav),
            target_addr: ("127.0.0.1".into(), "5011".into())
        }
    }
}