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

/// Request to be sent by the camera client to the server
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CamRequest {
    /// List of cameras to acquire a frame from
    pub cameras: Vec<CamId>,

    /// Format of the images to acquire
    pub format: ImageFormat
}

/// Response to be sent by the server to the client
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CamResponse {
    /// Frames acquired in response to the request
    pub frames: HashMap<CamId, CamFrame>
}

/// An individual frame from a camera
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct CamFrame {

    /// UTC timestamp at which the frame was acquired
    #[serde(with = "ts_milliseconds")]
    pub timestamp: DateTime<Utc>,

    /// The format of this frame
    pub format: ImageFormat,

    /// The formatted image data
    pub data: Vec<u8>
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
        // Convert the data
        let image = match self.format {
            ImageFormat::Png => 
                image::load_from_memory_with_format(
                    &self.data, 
                    image::ImageFormat::Png
                )?,
            ImageFormat::Jpeg(_) =>
                image::load_from_memory_with_format(
                    &self.data, 
                    image::ImageFormat::Jpeg
                )?
        };

        Ok(CamImage {
            timestamp: self.timestamp,
            image
        })
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
            data
        })
    }
}