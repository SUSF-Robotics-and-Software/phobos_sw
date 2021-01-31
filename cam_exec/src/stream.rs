//! # Camera Stream
//!
//! This module provides a wrapper around a Gstreamer stream from a camera
//! object.

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use std::collections::HashMap;

use comms_if::eqpt::cam::{CamId, CamResponse, StreamSettings};
use gstreamer::{Bus, Element, prelude::*};
use log::info;

use crate::params::CamExecParams;

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

pub struct Stream {
    cam_devices: HashMap<CamId, String>,
    stream_res: (u32, u32),
    pipeline: Option<Element>,
    bus: Option<Bus>
}

// -----------------------------------------------------------------------------------------------
// ENUMS
// -----------------------------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum StreamError {

    #[error("Failed to initialise gstreamer: {0}")]
    GstInitError(gstreamer::glib::Error),

    #[error("Couldn't change the pipeline state: {0}")]
    PipelineStateChangeError(gstreamer::StateChangeError),

    #[error("Pipeline has no bus attached")]
    NoBus,

    #[error("Could not parse the pipeline: {0}")]
    PipelineParseError(gstreamer::glib::Error)
}

/// Possible messages from the stream
#[derive(Debug)]
pub enum StreamMessage {
    EndOfStream,
    Error(String)
}

// -----------------------------------------------------------------------------------------------
// IMPLS
// -----------------------------------------------------------------------------------------------

impl Stream {
    /// Create a new stream with the given settings.
    pub fn new(
        params: &CamExecParams, 
        settings: &StreamSettings
    ) -> Result<Self, StreamError> {

        // Init gstreamer
        gstreamer::init().map_err(|e| StreamError::GstInitError(e))?;

        // Create device map
        let mut cam_devices = HashMap::new();
        cam_devices.insert(CamId::LeftNav, params.left_nav_video_device.clone());
        cam_devices.insert(CamId::RightNav, params.right_nav_video_device.clone());

        // Build stream object, which isn't currently running
        let mut stream = Self {
            cam_devices,
            stream_res: (params.stream_width, params.stream_height),
            pipeline: None,
            bus: None
        };

        // Change the settings to default
        stream.change_settings(settings)?;

        // Return the stream
        Ok(stream)
    }

    /// Update the settings for the stream
    pub fn change_settings(
        &mut self, 
        settings: &StreamSettings
    ) -> Result<CamResponse, StreamError> {

        // If there's already a loaded pipeline stop it
        match self.pipeline {
            Some(ref p) => {
                p.set_state(gstreamer::State::Null)
                    .map_err(|e| StreamError::PipelineStateChangeError(e))?;
            },
            None => ()
        }

        // Create the new pipeline
        match self.build_pipeline(settings)? {
            Some(p) => {
                // Start the pipeline
                p.set_state(gstreamer::State::Playing)
                    .map_err(|e| StreamError::PipelineStateChangeError(e))?;
            
                // Set the bus
                self.bus = Some(p.get_bus().ok_or(StreamError::NoBus)?);

                self.pipeline = Some(p);
            },
            None => {
                self.pipeline = None;
                self.bus = None;
            }
        };

        Ok(CamResponse::StreamSettingsAccepted)
    }

    pub fn get_message(&mut self) -> Result<Option<StreamMessage>, StreamError> {
        match self.bus {
            Some(ref bus) => {
                // If bus has pending message
                if bus.have_pending() {
                    use gstreamer::MessageView;

                    let msg = bus.pop();

                    match msg {
                        Some(msg) => match msg.view() {
                            MessageView::Eos(..) => {
                                Ok(Some(StreamMessage::EndOfStream))
                            },
                            MessageView::Error(e) => {
                                Ok(Some(StreamMessage::Error(
                                    format!("Error from {:?}: {} ({:?})",
                                    e.get_src().map(|s| s.get_path_string()),
                                    e.get_error(),
                                    e.get_debug()
                                ))))
                            },
                            _ => Ok(None)
                        },
                        None => Ok(None)
                    }
                }
                else {
                    Ok(None)
                }
            },
            None => Ok(None)
        }
    }

    /// Convert a StreamSettings to a Pipeline
    fn build_pipeline(&self, settings: &StreamSettings) -> Result<Option<Element>, StreamError> {
        // If settings have None as a camera return no element
        if settings.camera.is_none() {
            return Ok(None)
        }

        // Build the pipeline string
        let pipeline_spec = [
            format!("v4l2src device=\"{}\"", self.cam_devices[&settings.camera.unwrap()]),
            format!("video/x-raw,width={},heigh={}", self.stream_res.0, self.stream_res.1),
            "queue".into(),
            "jpegenc".into(),
            "rtpjpegpay".into(),
            format!("udpsink host={} port={}", settings.target_addr.0, settings.target_addr.1)
        ].join(" ! ");

        info!("Gstreamer pipeline: {}", pipeline_spec.as_str());

        // Parse the pipeline
        let pipeline = gstreamer::parse_launch(pipeline_spec.as_str())
            .map_err(|e| StreamError::PipelineParseError(e))?;

        // Return the pipeline
        Ok(Some(pipeline))
    }
}
