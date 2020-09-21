//! Session management

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External imports
use chrono::{DateTime, Utc};
use conquer_once::OnceCell;
use std::path::PathBuf;
use std::fs;
use thiserror::Error;

// Internal imports
use crate::time;

// ---------------------------------------------------------------------------
// STATICS
// ---------------------------------------------------------------------------

static SESSION_EPOCH: OnceCell<DateTime<Utc>> = OnceCell::uninit();

// ---------------------------------------------------------------------------
// CONSTANTS
// ---------------------------------------------------------------------------

/// A chrono format string which diplays a timestamp. See 
/// https://docs.rs/chrono/0.4.11/chrono/format/strftime/index.html for more
/// information.
const TIMESTAMP_FORMAT: &'static str = "%Y%m%d_%H%M%S";

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A struct storing information about the current session
pub struct Session {
    /// The root directory for this session
    pub session_root: PathBuf,

    /// The root directory for this session's archives
    pub arch_root: PathBuf,

    /// The path to the session's log file
    pub log_file_path: PathBuf,
}

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Possible errors associated with the session module.
#[derive(Error, Debug)]
pub enum SessionError {
    #[error("The software root environment variable (SUSF_PHOBOS_SW_ROOT) is not set")]
    SwRootNotSet,

    #[error("Cannot create the session directory: {0}")]
    CannotCreateDir(std::io::Error),

    #[error(
        "Cannot initialise the session epoch, have you already initialised the\
         session? (conquer_once error: {0})")]
    CannotInitEpoch(conquer_once::TryInitError),

    #[error("Cannot get the epoch time, did you forget to initialise the session?")]
    CannotGetEpoch
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Session {

    /// Start a new session within the given directory.
    ///
    /// This will create a new session directory named `{exec_name}_{timestamp}` 
    pub fn new(
        exec_name: &str, sessions_dir: &str
    ) -> Result<Self, SessionError> {
        
        // Set the session epoch
        match SESSION_EPOCH.try_init_once(||
            Utc::now()
        ) {
            Ok(_) => (),
            Err(e) => return Err(SessionError::CannotInitEpoch(e))
        };

        // Format the session epoch as a timestamp
        let timestamp = match SESSION_EPOCH.get() {
            Some(e) => e.format(TIMESTAMP_FORMAT),
            None => return Err(SessionError::CannotGetEpoch)
        };

        // Get the root directory
        let root = crate::host::get_phobos_sw_root()
            .map_err(|_| SessionError::SwRootNotSet)?;

        // Create the session path
        let mut path: PathBuf = root.clone();
        path.push(String::from(sessions_dir));
        path.push(format!("{}_{}", exec_name, timestamp));

        // Create the directory
        match fs::create_dir_all(path.clone()) {
            Ok(_) => (),
            Err(e) => return Err(SessionError::CannotCreateDir(e))
        };

        // Create the archive dir
        let mut arch_path: PathBuf = path.clone();
        arch_path.push("arch");
        match fs::create_dir_all(arch_path.clone()) {
            Ok(_) => (),
            Err(e) => return Err(SessionError::CannotCreateDir(e))
        };

        // Create the log file path
        let mut log_file_path = path.clone();
        log_file_path.push(format!("{}.log", exec_name));

        // Build the session struct
        Ok(Session {
            session_root: path,
            arch_root: arch_path,
            log_file_path
        })
    }
}

// ---------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ---------------------------------------------------------------------------

/// Get the number of seconds elapsed since the start of the session.
///
/// # Panics
/// - This function will panic if the session epoch has not been 
///   initialised, which is performed on creating a new Session instance.
pub fn get_elapsed_seconds() -> f64 {
    match SESSION_EPOCH.get() {
        Some(e) => {
            let elapsed = Utc::now() - *e;
            match time::duration_to_seconds(elapsed) {
                Some(s) => s,
                None => std::f64::NAN
            }
        },
        None => panic!("Cannot get the session epoch!")
    }
}

/// Return a reference to the session's epoch.
///
/// # Panics
/// - This function will panic if the session epoch has not been 
///   initialised, which is performed on creating a new Session instance.
pub fn get_epoch() -> &'static DateTime<Utc> {
    match SESSION_EPOCH.get() {
        Some(e) => e,
        None => panic!("Cannot get the session epoch!")
    }
}