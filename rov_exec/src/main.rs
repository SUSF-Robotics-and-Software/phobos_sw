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
