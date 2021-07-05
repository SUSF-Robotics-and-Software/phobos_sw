//! # Perloc Equipment Communications Module

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::{convert::TryFrom, ops::Deref};

use base64::DecodeError;
use byteorder::{BigEndian, ByteOrder};
use chrono::{serde::ts_milliseconds, DateTime, Utc};
use image::{ImageBuffer, Luma, Pixel, Primitive};
use serde::{Deserialize, Serialize, Serializer};

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct LocStreamStatus {
    /// If the stream is running
    pub running: bool,

    /// The time in seconds the stream has been running for, will be zero if `running == false`.
    pub uptime_s: f64,
}

/// A serialisable depth image frame
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct DepthFrame {
    /// UTC timestamp at which the frame was acquired
    #[serde(with = "ts_milliseconds")]
    pub timestamp: DateTime<Utc>,

    /// Width of the image in pixels
    pub width: u32,

    /// Height of the image in pixels
    pub height: u32,

    /// The formatted image data, encoded in base64, by first flattening the 16 bit image, then
    /// converting to a bigendian Vec<u8>, then encoding in base64.
    pub b64_data: String,
}

/// Represents a concrete image of depth in mm.
#[derive(Debug, Clone, Serialize)]
pub struct DepthImage {
    /// UTC timestamp at which the frame was acquired
    pub timestamp: DateTime<Utc>,

    /// The 16 bit greyscale image which describes z depth from the camera's optical centre, in
    /// millimeters.
    #[serde(serialize_with = "serialize_image_buffer_luma")]
    pub image: ImageBuffer<Luma<u16>, Vec<u16>>,
}

#[derive(Debug, Serialize)]
struct SerdeImgBuff<Container> {
    width: u32,
    height: u32,
    data: Container,
}

// -----------------------------------------------------------------------------------------------
// ENUMS
// -----------------------------------------------------------------------------------------------

/// Commands that can be sent to perloc
#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum PerlocCmd {
    /// Acquire depth image command
    AcqDepthFrame,

    /// Start localisation data stream command
    StartLocStream,

    /// Stop localisation data stream command
    StopLocStream,
}

/// Replies that can be sent by perloc
#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum PerlocRep {
    /// A depth image encoded as a [`DepthFrame`] object.
    DepthFrame(DepthFrame),

    /// The status of the localisation stream
    LocStreamStatus(LocStreamStatus),

    /// An error occured in the server
    Error(PerlocServerError),
}

#[derive(Debug, Serialize, Deserialize, Clone, thiserror::Error)]
pub enum PerlocServerError {}

#[derive(Debug, thiserror::Error, Clone)]
pub enum PerlocError {
    #[error("Failed to decode depth image from base64: {0}")]
    DepthDecodeError(DecodeError),

    #[error("The encoded frame data was the wrong size")]
    FrameWrongSize,
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl TryFrom<DepthFrame> for DepthImage {
    type Error = PerlocError;

    fn try_from(frame: DepthFrame) -> Result<Self, Self::Error> {
        // Decode the bytes from the base64 string
        let bytes = match base64::decode(frame.b64_data) {
            Ok(v) => v,
            Err(e) => return Err(PerlocError::DepthDecodeError(e)),
        };

        // Put those bytes (which are bigendian) into a buffer
        let buffer_len = bytes.len() / 2;
        let mut buff = vec![0u16; buffer_len];
        BigEndian::read_u16_into(&bytes, &mut buff);

        // Build the image from the raw data
        let image = ImageBuffer::from_raw(frame.width, frame.height, buff)
            .ok_or(PerlocError::FrameWrongSize)?;

        // Construct self
        Ok(Self {
            timestamp: frame.timestamp,
            image,
        })
    }
}

impl<P, Container> From<&ImageBuffer<P, Container>> for SerdeImgBuff<Container>
where
    P: Pixel + 'static,
    P::Subpixel: 'static,
    Container: Deref<Target = [P::Subpixel]> + Clone,
{
    fn from(img: &ImageBuffer<P, Container>) -> Self {
        Self {
            width: img.width(),
            height: img.height(),
            data: img.as_raw().clone(),
        }
    }
}

// -----------------------------------------------------------------------------------------------
// FUNCTIONS
// -----------------------------------------------------------------------------------------------

fn _serialize_image_buffer<S, P, Container>(
    img: &ImageBuffer<P, Container>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: Serializer,
    P: Pixel + Serialize + 'static,
    P::Subpixel: 'static,
    Container: Serialize + Deref<Target = [P::Subpixel]> + Clone,
{
    SerdeImgBuff::from(img).serialize(serializer)
}

fn serialize_image_buffer_luma<S, T, Container>(
    img: &ImageBuffer<Luma<T>, Container>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: Serializer,
    T: Serialize + Primitive + 'static,
    <Luma<T> as Pixel>::Subpixel: 'static,
    Container: Serialize + Deref<Target = [<Luma<T> as Pixel>::Subpixel]> + Clone,
{
    SerdeImgBuff::from(img).serialize(serializer)
}
