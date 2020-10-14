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
// use cam_client::CamClient;
use comms_if::{eqpt::mech::{MechDemsResponse, MechDems}, tc::Tc, tc::TcResponse};
use params::RovExecParams;
use rov_lib::{*, mech_client::MechClientError, tc_client::{TcClient, TcClientError}};

mod tc_processor;

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use log::{debug, error, info, warn};
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
const CYCLE_PERIOD_S: f64 = 0.05;

/// Limit of the number of times recieve errors from the mech server can be created consecutively
/// before safe mode will be engaged.
const MAX_MECH_RECV_ERROR_LIMIT: u64 = 5;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// Global data store for the executable.
#[derive(Default)]
struct DataStore {
    /// Determines if the rover is in safe mode.
    safe: bool,
    
    /// Gives the reason for the rover being in safe mode.
    safe_cause: Option<SafeModeCause>,

    
    // LocoCtrl
    
    loco_ctrl: loco_ctrl::LocoCtrl,
    loco_ctrl_input: loco_ctrl::InputData,
    loco_ctrl_output: MechDems,
    loco_ctrl_status_rpt: loco_ctrl::StatusReport,
    
    // Monitoring Counters
    
    /// Number of consecutive cycle overruns
    num_consec_cycle_overruns: u64,

    /// Number of consecutive mechanisms client recieve errors
    num_consec_mech_recv_errors: u64
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
    let mut use_tc_client = false;

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
            "Loaded script lasts {:.02} s and contains {} TCs\n",
            si.get_duration(),
            si.get_num_tcs()
        );

        // Set the interpreter in the source
        tc_source = TcSource::Script(si);
    }
    // If no arguments then setup the tc client
    else if args.len() == 1 {

        info!("No script provided, remote control via the TcClient will be used\n");
        use_tc_client = true;

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

    if use_tc_client {
        tc_source = TcSource::Remote(
            TcClient::new(&zmq_ctx, &rov_exec_params)
                .wrap_err("Failed to initialise the TcClient")?
        );
        info!("TcClient initialised");
    }

    #[cfg(feature = "mech")]
    let mut mech_client = {
        let c = MechClient::new(&zmq_ctx, &rov_exec_params)
            .wrap_err("Failed to initialise MechClient")?;
        info!("MechClient initialised");
        c
    };

    #[cfg(feature = "cam")]
    let mut _cam_client = {
        let c = CamClient::new(&zmq_ctx, &rov_exec_params)
            .wrap_err("Failed to initialise CamClient")?;
        info!("CamClient initialised");
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

        // Branch depending on the source
        match tc_source {
            // If no source no point in continuing so break
            TcSource::None => raise_error!("No TC source present"),

            // Currently ground command not supported
            TcSource::Remote(ref client) => {
                // If the client is connected remove any safe mode, otherwise make safe
                if client.is_connected() {
                    ds.make_unsafe(SafeModeCause::TcClientNotConnected).ok();
                }
                else {
                    ds.make_safe(SafeModeCause::TcClientNotConnected);
                }

                // Get commands until none remain
                loop {
                    match client.recieve_tc() {
                        Ok(Some(tc)) => {
                            // Branch based on safe mode. If we are in safe mode we need to send the
                            // cannot execute response and should not process the TC, unless it is
                            // the make unsafe TC
                            let response_result = match ds.safe {
                                true => {
                                    // Execute TC if make unsafe
                                    match tc {
                                        Tc::MakeUnsafe => {
                                            tc_processor::exec(&mut ds, &tc);
                                            client.send_response(TcResponse::Ok)
                                        }
                                        _ => 
                                            client.send_response(TcResponse::CannotExecute)
                                    }
                                },
                                false => {
                                    // Process the TC
                                    tc_processor::exec(&mut ds, &tc);

                                    // Send response
                                    client.send_response(TcResponse::Ok)
                                }
                            };

                            // Print warning if couldn't send the response
                            match response_result {
                                Ok(_) => (),
                                Err(e) => warn!("Could not respond to TC: {}", e)
                            }


                        },
                        Ok(None) => {
                            break
                        },
                        // If not connected go into safe mode
                        Err(TcClientError::NotConnected) => {
                            error!("Connection to TcServer lost");
                            ds.make_safe(SafeModeCause::TcClientNotConnected);
                            break;
                        },
                        Err(e) => return Err(e)
                            .wrap_err("An error occured while receiving TCs from the server")
                    }
                }
            },

            TcSource::Script(ref mut si) => 
                match si.get_pending_tcs() {
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
        };

        // ---- AUTONOMY PROCESSING ----

        // ---- CONTROL ALGORITHM PROCESSING ----

        // LocoCtrl processing
        match ds.loco_ctrl.proc(&ds.loco_ctrl_input) {
            Ok((o, r)) => {
                ds.loco_ctrl_output = o;
                ds.loco_ctrl_status_rpt = r;
            },
            Err(e) => {
                // LocoCtrl errors usually just mean you sent the wrong TC, so just issue the
                // warning and continue.
                warn!("Error during LocoCtrl processing: {}", e)
            }
        };

        // Send demands to mechanisms
        #[cfg(feature = "mech")]
        match mech_client.send_demands(&ds.loco_ctrl_output) {
            Ok(MechDemsResponse::DemsOk) => {
                ds.make_unsafe(SafeModeCause::MechClientNotConnected).ok();

                // Reset the recieve error counter
                ds.num_consec_mech_recv_errors = 0;
            },
            Ok(r) => warn!(
                "Recieved non-nominal response from MechServer: {:?}", 
                r
            ),
            Err(MechClientError::NotConnected) => {
                if !ds.safe {
                    error!("Connection to the MechServer lost");
                }
                ds.make_safe(SafeModeCause::MechClientNotConnected);
            }
            Err(MechClientError::RecvError(_)) => {
                ds.num_consec_mech_recv_errors += 1;

                // If over the limit print error and enter safe mode
                if ds.num_consec_mech_recv_errors > MAX_MECH_RECV_ERROR_LIMIT {
                    if !ds.safe {
                        error!(
                            "Maximum number of MechClient Recieve Errors ({}) has been exceeded",
                            MAX_MECH_RECV_ERROR_LIMIT
                        );
                    }
                    ds.make_safe(SafeModeCause::MechClientNotConnected);
                }
            },
            Err(e) => warn!("MechClient processing error: {}", e)
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
    Remote(TcClient),
    Script(ScriptInterpreter)
}

/// Gives the reason the rover has been put into safe mode
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
enum SafeModeCause {
    MakeSafeTc,
    TcClientNotConnected,
    MechClientNotConnected,
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl DataStore {

    /// Puts the rover into safe mode with the given cause.
    fn make_safe(&mut self, cause: SafeModeCause) {
        if !self.safe {
            warn!("Make safe requested, cause: {:?}", cause);
            self.safe = true;
            self.safe_cause = Some(cause);

            // Make loco_ctrl safe
            self.loco_ctrl.make_safe();
        }
    }

    /// Attempts to disable the safe mode by clearing the given cause.
    ///
    /// Returns `Ok(())` if this cause was cleared and safe mode was disabled, or `Err(())` 
    /// otherwise. To remove safe mode the provided cause must match the initial reason for safe 
    /// mode being enabled.
    ///
    /// If safe mode was not enabled `Ok(())` is returned
    fn make_unsafe(&mut self, cause: SafeModeCause) -> Result<(), ()> {
        if !self.safe {
            return Ok(())
        }

        match self.safe_cause {
            Some(root_cause) => {
                if cause == root_cause {
                    self.safe = false;
                    self.safe_cause = None;
                    info!("Make unsafe requested, root cause match, safe mode disabled");
                    Ok(())
                }
                else {
                    // info!(
                    //     "Make unsafe requested, root cause ({:?}) differs from response ({:?}), \
                    //     rejected", 
                    //     root_cause,
                    //     cause
                    // );
                    Err(())
                }
            },
            None => Ok(())
        }
    }

    /// Clears those items that need clearing at the start of a cycle.
    fn cycle_start(&mut self) {
        
        self.loco_ctrl_input = loco_ctrl::InputData::default();
        self.loco_ctrl_output = MechDems::default();
        self.loco_ctrl_status_rpt = loco_ctrl::StatusReport::default();
    }
}