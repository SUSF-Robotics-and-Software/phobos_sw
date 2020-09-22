//! # Mechanisms Control Executable
//!
//! This executable is responsible for controlling the mechanisms of the rover:
//! - Locomotion actuators (6 steer, 6 drive)
//! - Arm actuators (TODO)
//! - Any other actuators that may be added to the rover baseline

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

/// Driver used to control servos.
mod servo_ctrl;

/// Mechanisms server abstraction.
mod mech_server;

/// Parameters for the mechanisms executable.
mod params;

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

// External
use comms_if::eqpt::MechDemsResponse;
use log::{info, warn, trace};
use color_eyre::{Result, eyre::WrapErr};

// Internal
use mech_server::MechServer;
use util::{
    host,
    logger::{logger_init, LevelFilter},
    session::Session,
};

// ------------------------------------------------------------------------------------------------
// MAIN
// ------------------------------------------------------------------------------------------------

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
    info!("Mechanisms Control Executable\n");
    info!(
        "Running on: {:#?}", 
        host::get_uname().wrap_err("Failed to get host information")?
    );
    info!("Session directory: {:?}\n", session.session_root);

    info!("Initialising...");

    // ---- LOAD PARAMETERS ----

    let params = util::params::load("mech_exec.toml")?;

    info!("Parameters loaded");

    // ---- SERVER INITIALISATION ----

    let mut server: MechServer = MechServer::new(&params)
        .wrap_err("Failed to initialise server")?;
    
    info!("Server initialised");

    // ---- MAIN LOOP ----

    info!("Initialisation complete, entering main loop in safe mode");

    let mut safe_mode = true;

    loop {
        // Get demands from client
        let dems = match server.get_demands() {
            Some(d) => {
                if safe_mode {
                    info!("Recieved valid demand, exiting safe mode");
                    safe_mode = false;
                }
                d
            },
            None => {
                if !safe_mode {
                    warn!("Entering safe mode");
                    safe_mode = true;
                }
                continue
            }
        };

        trace!("Recieved demands, validating...");
        
        // TODO: Validate demands

        trace!("Validated, sending response...");

        // Send response to client
        match server.send_dems_response(&MechDemsResponse::DemsOk) {
            Ok(_) => (),
            Err(_) => {
                warn!("Couldn't send response to client, entering safe mode");
                safe_mode = true;
                continue
            }
        }

        // TODO: Actuate demands
        info!("Actuating {:#?}", dems);
    }
}
