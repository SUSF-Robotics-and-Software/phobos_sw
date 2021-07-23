//! # Autonomy Test
//!
//! This binary allows the autonomy system to be run without requiring the simulation or phyiscal
//! rover. It is designed to allow quick and easy development of the autonomy system itself.

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::{
    env, thread,
    time::{Duration, Instant},
};

use chrono::Utc;
use color_eyre::{
    eyre::{eyre, WrapErr},
    Result,
};
use comms_if::eqpt::perloc::DepthImage;
use image::ImageBuffer;
use log::{debug, info, warn};

use nalgebra::{UnitQuaternion, UnitVector3, Vector3};
use rov_lib::{
    auto::{auto_mgr::AutoMgrOutput, loc::Pose, AutoMgr},
    data_store::DataStore,
    tc_processor,
};
use util::{
    host,
    logger::{logger_init, LevelFilter},
    script_interpreter::{PendingTcs, ScriptInterpreter},
    session::Session,
};

// ------------------------------------------------------------------------------------------------
// CONSTANTS
// ------------------------------------------------------------------------------------------------

/// Target period of one cycle.
const CYCLE_PERIOD_S: f64 = 0.10;

/// Number of cycles per second
const CYCLE_FREQUENCY_HZ: f64 = 1.0 / CYCLE_PERIOD_S;

// ------------------------------------------------------------------------------------------------
// MAIN
// ------------------------------------------------------------------------------------------------

fn main() -> Result<()> {
    // ---- EARLY INITIALISATION ----

    // Initialise session
    let session = Session::new("auto_test", "sessions").wrap_err("Failed to create the session")?;

    // Initialise logger
    logger_init(LevelFilter::Trace, &session).wrap_err("Failed to initialise logging")?;

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
        script_interpreter = ScriptInterpreter::new(&args[1]).wrap_err("Failed to load script")?;

        // Display some info
        info!(
            "Loaded script lasts {:.02} s and contains {} TCs\n",
            script_interpreter.get_duration(),
            script_interpreter.get_num_tcs()
        );
    }
    // If no arguments error out
    else {
        return Err(eyre!("Expected path to TC script as only argument"));
    }

    // ---- MODULE INIT ----

    let mut ds = DataStore::default();

    let mut auto_mgr =
        AutoMgr::init("auto_mgr.toml", session.clone()).wrap_err("Failed to initialise AutoMgr")?;
    info!("AutoMgr init complete");

    // Starting pose
    let start_pose = Pose::new(
        Vector3::new(1.0, 1.0, 0.0),
        // UnitQuaternion::identity()
        UnitQuaternion::from_axis_angle(
            &UnitVector3::new_unchecked(Vector3::z()),
            std::f64::consts::FRAC_PI_2,
        ),
    );

    // Set initial pose
    auto_mgr.persistant.loc_mgr.set_pose(start_pose);

    // ---- MAIN LOOP ----

    info!("Begining main loop\n");

    let mut end_of_script = false;

    loop {
        // Get cycle start time
        let cycle_start_instant = Instant::now();

        // Clear items that need wiping at the start of the cycle
        ds.cycle_start(CYCLE_FREQUENCY_HZ);

        // ---- TELECOMMAND PROCESSING ----

        if !end_of_script {
            match script_interpreter.get_pending_tcs() {
                PendingTcs::None => (),
                PendingTcs::Some(tc_vec) => {
                    for tc in tc_vec.iter() {
                        tc_processor::exec(&mut ds, tc);
                    }
                }
                // Exit if end of script reached
                PendingTcs::EndOfScript => {
                    info!("End of TC script reached, waiting for AutoMgr to be in Off");
                    end_of_script = true;
                }
            }
        }

        // ---- AUTONOMY PROCESSING ----

        // Step the autonomy manager
        let auto_mgr_output = auto_mgr
            .step(ds.auto_cmd.take())
            .wrap_err("Error stepping the autonomy manager")?;

        // If autonomy requested a depth image set an empty one since we're using per's
        // internal global terrain map
        if matches!(
            auto_mgr_output,
            AutoMgrOutput::PerlocCmd(comms_if::eqpt::perloc::PerlocCmd::AcqDepthFrame)
        ) {
            auto_mgr.set_depth_img(DepthImage {
                timestamp: Utc::now(),
                image: ImageBuffer::new(0, 0),
            });
            info!("Empty depth image set in AutoMgr");
        }

        // ---- CYCLE MANAGEMENT ----

        let cycle_dur = Instant::now() - cycle_start_instant;

        // Get sleep duration
        match Duration::from_secs_f64(CYCLE_PERIOD_S).checked_sub(cycle_dur) {
            Some(d) => {
                ds.num_consec_cycle_overruns = 0;
                thread::sleep(d);
            }
            None => {
                warn!(
                    "Cycle overran by {:.06} s",
                    cycle_dur.as_secs_f64() - Duration::from_secs_f64(CYCLE_PERIOD_S).as_secs_f64()
                );
                ds.num_consec_cycle_overruns += 1;
            }
        }

        // Increment cycle counter
        ds.num_cycles += 1;

        // If at the end of the script and the auto mgr is off exit
        if end_of_script && auto_mgr.is_off() {
            info!("End of script and AutoMgr is in Off, exiting");
            break;
        }
    }

    session.exit();

    Ok(())
}
