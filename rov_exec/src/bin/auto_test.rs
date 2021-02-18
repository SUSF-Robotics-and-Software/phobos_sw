//! # Autonomy Test
//!
//! This binary allows the autonomy system to be run without requiring the simulation or phyiscal
//! rover. It is designed to allow quick and easy development of the autonomy system itself.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::{env, time::Instant};

use color_eyre::{Result, eyre::{eyre, WrapErr}};
use comms_if::tc::{Tc, auto::AutoCmd};
use log::{debug, info};

use rov_lib::auto::AutoMgr;
use util::{host, logger::{LevelFilter, logger_init}, script_interpreter::{PendingTcs, ScriptInterpreter}, session::Session};

// ------------------------------------------------------------------------------------------------
// MAIN
// ------------------------------------------------------------------------------------------------ 

fn main() -> Result<()> {

    // ---- EARLY INITIALISATION ----

    // Initialise session
    let session = Session::new(
        "auto_test", 
        "sessions"
    ).wrap_err("Failed to create the session")?;

    // Initialise logger
    logger_init(LevelFilter::Trace, &session)
        .wrap_err("Failed to initialise logging")?;

    // Log information on this execution.
    info!("Autonomy Test\n");
    info!(
        "Running on: {:#?}", 
        host::get_uname().wrap_err("Failed to get host information")?
    );
    info!("Session directory: {:?}\n", session.session_root);

    // ---- INITIALISE TC SCRIPT ----

    // Collect all arguments
    let args: Vec<String> = env::args().collect();

    debug!("CLI arguments: {:?}", args);

    let mut script_interpreter: ScriptInterpreter;
    
    // If we have a single argument use it as the script path
    if args.len() == 2 {

        info!("Loading script from \"{}\"", &args[1]);

        // Load the script interpreter
        script_interpreter = ScriptInterpreter::new(
            &args[1]
        ).wrap_err("Failed to load script")?;

        // Display some info
        info!(
            "Loaded script lasts {:.02} s and contains {} TCs\n",
            script_interpreter.get_duration(),
            script_interpreter.get_num_tcs()
        );
    }
    // If no arguments error out
    else {
        return Err(eyre!("Expected either path to TC script as only argument"));
    }

    // ---- MODULE INIT ----

    let mut auto_mgr = AutoMgr::init("auto_mgr.toml")
        .wrap_err("Failed to initialise AutoMgr")?;
    info!("AutoMgr init complete");

    // ---- MAIN LOOP ----

    info!("Begining main loop\n");

    loop {

        // Get cycle start time
        let cycle_start_instant = Instant::now();

        // ---- TELECOMMAND PROCESSING ----

        let mut auto_tc: Option<AutoCmd>;
        
        match script_interpreter.get_pending_tcs() {
            PendingTcs::None => (),
            PendingTcs::Some(tc_vec) => {
                for tc in tc_vec.iter() {
                    // TODO: add support for non-auto tcs
                    match tc {
                        Tc::Autonomy(a) => auto_tc = Some(a.clone()),
                        _ => auto_tc = None
                    }
                }
            }
            // Exit if end of script reached
            PendingTcs::EndOfScript => {
                info!("End of TC script reached, stopping");
                break
            }
        }

    }

    Ok(())
}