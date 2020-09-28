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
// USE MODULES FROM LIBRARY
// ---------------------------------------------------------------------------

#[cfg(feature = "mech")]
use mech_client::MechClient;
use comms_if::eqpt::{MechDemsResponse, MechDems};
use params::RovExecParams;
use rov_lib::*;

mod tc_processor;

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use log::{debug, info, warn};
use std::env;
use std::thread;
use std::time::{Duration, Instant};
use color_eyre::{Report, eyre::{WrapErr, eyre}};

// Internal
use util::{
    raise_error,
    host, 
    module::State,
    logger::{logger_init, LevelFilter},
    session::Session,
    script_interpreter::{ScriptInterpreter, PendingTcs},
    //archive::Archived
};

// ---------------------------------------------------------------------------
// CONSTANTS
// ---------------------------------------------------------------------------

/// Target period of one cycle.
const CYCLE_PERIOD_S: f64 = 0.1;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Global data store for the executable.
#[derive(Default)]
struct DataStore {
    /// Determines if the rover is in safe mode.
    make_safe: bool,

    // LocoCtrl

    loco_ctrl: loco_ctrl::LocoCtrl,
    loco_ctrl_input: loco_ctrl::InputData,
    loco_ctrl_output: MechDems,
    loco_ctrl_status_rpt: loco_ctrl::StatusReport,

    // Monitoring Counters
    
    /// Number of consecutive cycle overruns
    num_consec_cycle_overruns: u64
}

// ---------------------------------------------------------------------------
// FUNCTIONS
// ---------------------------------------------------------------------------

/// Executable main function, entry point.
fn main() -> Result<(), Report> {

    // ---- EARLY INITIALISATION ----

    // Initialise session
    let session = Session::new(
        "rov_exec", 
        "sessions"
    ).wrap_err("Failed to create the session")?;

    // Initialise logger
    logger_init(LevelFilter::Trace, &session)
        .wrap_err("Failed to initialise logging")?;

    // Log information on this execution.
    info!("Phobos Rover Executable\n");
    info!(
        "Running on: {:#?}", 
        host::get_uname().wrap_err("Failed to get host information")?
    );
    info!("Session directory: {:?}\n", session.session_root);

    // ---- LOAD PARAMETERS ----

    let rov_exec_params: RovExecParams = util::params::load(
        "rov_exec.toml"
    ).wrap_err("Could not load rov_exec params")?;

    info!("Exec parameters loaded");

    // ---- INITIALISE TC SOURCE ----

    // TC source is used to determine whether we're getting TCs from a script
    // or from the ground.
    let mut tc_source = TcSource::None;

    // Collect all arguments
    let args: Vec<String> = env::args().collect();

    debug!("CLI arguments: {:?}", args);
    
    // If we have a single argument use it as the script path
    if args.len() == 2 {

        info!("Loading script from \"{}\"", &args[1]);

        // Load the script interpreter
        let si = ScriptInterpreter::new(
            &args[1]).wrap_err("Failed to load script")?;

        // Display some info
        info!(
            "Loaded script lasts {:.02} s and contains {} TCs",
            si.get_duration(),
            si.get_num_tcs()
        );

        // Set the interpreter in the source
        tc_source = TcSource::Script(si);
    }
    else if args.len() == 1 {
        // TODO: init remote control from ground
    }
    else {
        return Err(eyre!(
            "Expected either zero or one argument, found {}", args.len() - 1)
        );
    }

    // ---- INITIALISE DATASTORE ----

    info!("Initialising modules...");

    let mut ds = DataStore::default();

    // ---- INITIALISE MODULES ----

    ds.loco_ctrl.init("loco_ctrl.toml", &session)
        .wrap_err("Failed to initialise LocoCtrl")?;
    info!("LocoCtrl init complete");

    info!("Module initialisation complete\n");

    // ---- INITIALISE NETWORK ----

    info!("Initialising network");

    let zmq_ctx = comms_if::net::zmq::Context::new();

    #[cfg(feature = "mech")]
    let mut mech_client = {
        let c = MechClient::new(&zmq_ctx, &rov_exec_params)
            .wrap_err("Failed to initialise MechClient")?;
        info!("MechClient initialised");
        c
    };

    info!("Network initialisation complete");

    // ---- MAIN LOOP ----

    info!("Begining main loop\n");

    loop {

        // Get cycle start time
        let cycle_start_instant = Instant::now();

        // Clear items that need wiping at the start of the cycle
        ds.cycle_start();

        // ---- DATA INPUT ----

        // ---- TELECOMMAND PROCESSING ----

        // Get the list of pending TCs
        
        let pending_tcs = match tc_source {
            // If no source no point in continuing so break
            TcSource::None => raise_error!("No TC source present"),

            // Currently ground command not supported
            TcSource::Ground => raise_error!(
                "Ground commanding not yet supported"),

            TcSource::Script(ref mut si) => 
                si.get_pending_tcs()
        };

        match pending_tcs {
            PendingTcs::None => (),
            PendingTcs::Some(tc_vec) => {
                for tc in tc_vec.iter() {
                    tc_processor::exec(&mut ds, tc);
                }
            }
            // Exit if end of script reached
            PendingTcs::EndOfScript => {
                info!("End of TC script reached, stopping");
                break
            }
        }

        // ---- AUTONOMY PROCESSING ----

        // ---- CONTROL ALGORITHM PROCESSING ----

        // LocoCtrl processing
        match ds.loco_ctrl.proc(&ds.loco_ctrl_input) {
            Ok((o, r)) => {
                ds.loco_ctrl_output = o;
                ds.loco_ctrl_status_rpt = r;
            },
            Err(e) => {
                // No loco_ctrl error is unrecoverable, instead the approach 
                // taken is to make the rover safe.
                ds.make_safe = true;
                warn!("Error during LocoCtrl processing: {}", e)
            }
        };

        // Send demands to mechanisms
        #[cfg(feature = "mech")]
        match mech_client.send_demands(&ds.loco_ctrl_output) {
            Ok(MechDemsResponse::DemsOk) => (),
            Ok(r) => warn!(
                "Recieved non-nominal response from MechServer: {:?}", 
                r
            ),
            Err(e) => warn!("{}", e)
        }

        // ---- WRITE ARCHIVES ----
        // FIXME: Currently disabled as archiving isn't working quite right
        // ds.loco_ctrl.write().unwrap();

        // ---- CYCLE MANAGEMENT ----

        let cycle_dur = Instant::now() - cycle_start_instant;

        // Get sleep duration
        match Duration::from_secs_f64(CYCLE_PERIOD_S)
            .checked_sub(cycle_dur)
        {
            Some(d) => {
                ds.num_consec_cycle_overruns = 0;
                thread::sleep(d);
            },
            None => {
                warn!(
                    "Cycle overran by {:.06} s", 
                    cycle_dur.as_secs_f64() 
                        - Duration::from_secs_f64(CYCLE_PERIOD_S).as_secs_f64()
                );
                ds.num_consec_cycle_overruns += 1;

                // If number of overruns greater than the limit exit
                // TODO impl as param?
                // if ds.num_consec_cycle_overruns > 500 {
                //     raise_error!("More than 500 consecutive cycle overruns!");
                // }
            }
        }
    }

    // ---- SHUTDOWN ----

    info!("End of execution");

    Ok(())
}

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Various sources for the telecommands incoming to the exec.
#[allow(dead_code)]
enum TcSource {
    None,
    Ground,
    Script(ScriptInterpreter)
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl DataStore {

    /// Clears those items that need clearing at the start of a cycle.
    fn cycle_start(&mut self) {
        
        self.loco_ctrl_input = loco_ctrl::InputData::default();
        self.loco_ctrl_output = MechDems::default();
        self.loco_ctrl_status_rpt = loco_ctrl::StatusReport::default();
    }
}