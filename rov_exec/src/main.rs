//! Main rover-side executable entry point.

// ---------------------------------------------------------------------------
// MODULES
// ---------------------------------------------------------------------------

pub mod loco_ctrl;

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External

// Internal
use util::logger::{logger_init, LevelFilter};

// ---------------------------------------------------------------------------
// FUNCTIONS
// ---------------------------------------------------------------------------

/// Executable main function, entry point.
fn main() {
    // Initialise logger
    match logger_init(LevelFilter::Trace, "rov_exec.log") {
        Ok(_) => (),
        Err(e) => panic!("Error initialising logging: {:?}", e)
    };
}
