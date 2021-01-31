//! # Camera Executable
//!
//! This executable produces the camera streams for the rover.

// -----------------------------------------------------------------------------------------------
// MODULES
// -----------------------------------------------------------------------------------------------

mod cam_server;
mod stream;
mod params;

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use std::collections::HashMap;

use chrono::Utc;
use color_eyre::{Result, eyre::WrapErr};
use comms_if::eqpt::cam::{CamFrame, CamId, CamRequest, CamResponse, FrameRequest, StreamSettings};
use image::DynamicImage;
use log::{debug, info};
use rscam::{Camera, Config};

use cam_server::CamServer;
use stream::Stream;
use util::{
    host,
    logger::{logger_init, LevelFilter},
    session::Session,
};

// -----------------------------------------------------------------------------------------------
// STRUCTS
// -----------------------------------------------------------------------------------------------

/// Stores all rscam::Camera objects for the camera avaialble on the rover.
//TODO: Add right cam and middle cam when available
pub struct Cameras {
    left_nav: Camera,
    // right_nav: Camera
}

// -----------------------------------------------------------------------------------------------
// MAIN
// -----------------------------------------------------------------------------------------------

fn main() -> Result<()> {
    // ---- EARLY INITIALISATION ----

    // Initialise session
    let session = Session::new(
        "mech_exec", 
        "sessions"
    ).wrap_err("Failed to create the session")?;

    // Initialise logger
    logger_init(LevelFilter::Trace, &session)
        .wrap_err("Failed to initialise logging")?;

    // Log information on this execution.
    info!("Camera Executable\n");
    info!(
        "Running on: {:#?}", 
        host::get_uname().wrap_err("Failed to get host information")?
    );
    info!("Session directory: {:?}\n", session.session_root);

    info!("Initialising...");

    // ---- LOAD PARAMETERS ----

    let params = util::params::load("cam_exec.toml")?;

    info!("Parameters loaded");

    // ---- INIT SERVER ----

    let mut cam_server: CamServer = CamServer::new(&params)
        .wrap_err("Failed to initialise CamServer")?;

    info!("CamServer initialised");
    
    // ---- INIT CAMERAS OBJECT ----

    debug!("{}", params.left_nav_video_device);

    let mut cameras = Cameras {
        left_nav: Camera::new(&params.left_nav_video_device)?,
        // right_nav: Camera::new(&params.right_nav_video_device)?
    };

    // Set camera settings
    cameras.left_nav.start(&Config {
        interval: (1, 30),
        resolution: (640, 480),
        format: b"MJPG",
        ..Default::default()
    })?;
    // TODO:
    // cameras.right_nav.start(&Config {
    //     interval: (1, 30),
    //     resolution: (640, 480),
    //     format: b"MJPG",
    //     ..Default::default()
    // })?;

    info!("Cameras initialised");

    // ---- INIT STREAM ----

    // let mut stream = Stream::new(&params, &StreamSettings::default())
    //     .wrap_err("Failed to initialise stream")?;

    info!("Stream initialised");

    // ---- MAIN LOOP ----

    loop {
        // Get request from client
        let request = match cam_server.get_request() {
            Some(r) => r,
            None => continue
        };

        // Print the request
        info!("Got request from server: {:#?}", request);

        // Process the request depending on if it's a frame or settings request
        let response = match request {
            CamRequest::FrameRequest(req) => process_frame_request(&mut cameras, req)?,
            CamRequest::StreamSettingsRequest(settings) => panic!(),
                // stream.change_settings(&settings)?
        };

        // Send the response to the client
        cam_server.send_response(&response)?;

        // // Process any stream messages
        // match stream.get_message()? {
        //     Some(m) => info!("Stream message: {:?}", m),
        //     None => (),
        // }
    }
}

// -----------------------------------------------------------------------------------------------
// FUNCTIONS
// -----------------------------------------------------------------------------------------------

fn process_frame_request(cameras: &mut Cameras, req: FrameRequest) -> Result<CamResponse> {

    // Empty map of frames
    let mut frames = HashMap::<CamId, CamFrame>::new();

    // Loop over all requested cameras
    for cam_id in req.cameras {
        // Get the frame for this camera
        let raw_frame = match cam_id {
            CamId::LeftNav => {
                cameras.left_nav.capture()
                    .wrap_err("Could not capture left camera frame")?
            }
            CamId::RightNav => {
                cameras.left_nav.capture()
                    .wrap_err("Could not capture left camera frame")?
            }
        };
        let timestamp = Utc::now();

        // Convert to a dynamic image in the format requested
        let dyn_image = rscam_frame_to_dynamic_image(
            &raw_frame, 
            image::ImageFormat::Jpeg
        )?;

        // Get a CamFrame from that image and the timestamp
        let cam_frame = CamFrame::from_dyn_image(
            dyn_image,
            req.format,
            timestamp
        )?;

        // Add that frame into the frames map
        frames.insert(cam_id, cam_frame);
    }

    Ok(CamResponse::Frames(frames))
}

/// Convert an `rscam::Frame` struct into an `image::DynamicImage` struct.
fn rscam_frame_to_dynamic_image(
    frame: &rscam::Frame, 
    frame_format: image::ImageFormat
) -> Result<DynamicImage> {
    image::load_from_memory_with_format(frame, frame_format)
        .wrap_err("Couldn't convert from rscam::Frame to image::DynamicImage")
}

// fn u64_timestamp_to_utc(timestamp: u64) -> DateTime<Utc> {
//     // To naive time
//     let naive = NaiveDateTime::from_timestamp(timestamp as i64, 0);

//     // To DateTime
//     DateTime::from_utc(naive, Utc)
// }