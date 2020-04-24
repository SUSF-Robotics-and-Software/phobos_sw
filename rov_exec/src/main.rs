//! Main rover-side executable entry point.
//! 
//! # Architecture
//! 
//! The general execution methodology consists of:
//! 
//!     - Initialise all modules
//!     - Main loop:
//!         - System input acquisition:
//!             - Actuator sensing
//!             - IMU sensing
//!         - Telecommand processing and handling
//!         - Autonomy processing:
//!             - Position determination
//!             - Path planning and navigation
//!         - Trajcetory control processing
//!         - Locomotion control processing
//!         - Electronics driver execution
//! 
//! # Modules
//! 
//! All modules (e.g. `loco_ctrl`) shall meet the following requirements:
//!     1. Provide a public struct implementing the `util::module::State` trait.
//!     

// ---------------------------------------------------------------------------
// MODULES
// ---------------------------------------------------------------------------

pub mod loco_ctrl;

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use log::{info, error};

// Internal
use util::{
    raise_error,
    host, 
    module::State,
    logger::{logger_init, LevelFilter}
};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Global data store for the executable.
#[derive(Default)]
struct DataStore {
    loco_ctrl: loco_ctrl::LocoCtrl
}

// ---------------------------------------------------------------------------
// FUNCTIONS
// ---------------------------------------------------------------------------

/// Executable main function, entry point.
fn main() {

    // ---- EARLY INITIALISATION ----

    // Initialise logger
    match logger_init(LevelFilter::Trace, "rov_exec.log") {
        Ok(_) => (),
        Err(e) => panic!("Error initialising logging: {:?}", e)
    };

    // Log information on this execution.
    info!("Phobos Rover Executable\n");
    info!("Running on: {:#?}\n", host::get_uname().unwrap());

    // ---- INITIALISE DATASTORE ----

    info!("Initialising modules...");

    let mut ds = DataStore::default();

    // ---- INITIALISE MODULES ----

    match ds.loco_ctrl.init(loco_ctrl::InitData {
        params_path: "params/loco_ctrl.toml"
    }) {
        Ok(_) => info!("LocoCtrl init complete"),
        Err(e) => raise_error!("Error initialising LocoCtrl: {:#?}", e)
    };

    info!("Module initialisation complete\n");

    // ---- MAIN LOOP ----

    info!("Begining main loop\n");

    loop {
        // ---- DATA INPUT ----

        // ---- TELECOMMAND PROCESSING ----

        // ---- AUTONOMY PROCESSING ----

        // ---- CONTROL ALGORITHM PROCESSING ----

        // LocoCtrl processing
        let (loco_ctrl_output, loco_ctrl_status_rpt) = match ds.loco_ctrl.proc(
            loco_ctrl::InputData {
                cmd: Some(loco_ctrl::MnvrCommand {
                    mnvr_type: loco_ctrl::MnvrType::None,
                    curvature_m: None,
                    speed_ms: None,
                    turn_rate_rads: None
                })
            }
        ) {
            Ok((o, r)) => (Some(o), Some(r)),
            Err(e) => { 
                error!("LocoCtrl error: {:?}", e);
                (None, None)
            }
        };

        break;
    }
}
