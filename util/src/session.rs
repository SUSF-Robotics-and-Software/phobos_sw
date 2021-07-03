//! Session management

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External imports
use chrono::{DateTime, Utc};
use conquer_once::OnceCell;
use erased_serde::Serialize;
use log::info;
use log::warn;
use std::ffi::OsStr;
use std::fs;
use std::fs::OpenOptions;
use std::path::Path;
use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::sync::mpsc::channel;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::thread::JoinHandle;
use thiserror::Error;

// Internal imports
use crate::time;

// ---------------------------------------------------------------------------
// STATICS
// ---------------------------------------------------------------------------

static SESSION_EPOCH: OnceCell<DateTime<Utc>> = OnceCell::uninit();
static SAVE_SENDER: OnceCell<Mutex<Sender<(PathBuf, Box<dyn Serialize + Send>)>>> =
    OnceCell::uninit();

// ---------------------------------------------------------------------------
// CONSTANTS
// ---------------------------------------------------------------------------

/// A chrono format string which diplays a timestamp. See
/// https://docs.rs/chrono/0.4.11/chrono/format/strftime/index.html for more
/// information.
const TIMESTAMP_FORMAT: &str = "%Y%m%d_%H%M%S";

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A struct storing information about the current session
#[derive(Clone)]
pub struct Session {
    /// The root directory for this session
    pub session_root: PathBuf,

    /// The root directory for this session's archives
    pub arch_root: PathBuf,

    /// The path to the session's log file
    pub log_file_path: PathBuf,

    save_sender: Sender<(PathBuf, Box<dyn Serialize + Send>)>,

    save_stop: Arc<AtomicBool>,
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
         session? (conquer_once error: {0})"
    )]
    CannotInitEpoch(conquer_once::TryInitError),

    #[error("Cannot get the epoch time, did you forget to initialise the session?")]
    CannotGetEpoch,
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Session {
    /// Start a new session within the given directory.
    ///
    /// This will create a new session directory named `{exec_name}_{timestamp}`
    pub fn new(exec_name: &str, sessions_dir: &str) -> Result<Self, SessionError> {
        // Set the session epoch
        match SESSION_EPOCH.try_init_once(Utc::now) {
            Ok(_) => (),
            Err(e) => return Err(SessionError::CannotInitEpoch(e)),
        };

        // Format the session epoch as a timestamp
        let timestamp = match SESSION_EPOCH.get() {
            Some(e) => e.format(TIMESTAMP_FORMAT),
            None => return Err(SessionError::CannotGetEpoch),
        };

        // Get the root directory
        let root = crate::host::get_phobos_sw_root().map_err(|_| SessionError::SwRootNotSet)?;

        // Create the session path
        let mut path: PathBuf = root;
        path.push(String::from(sessions_dir));
        path.push(format!("{}_{}", exec_name, timestamp));

        // Create the directory
        match fs::create_dir_all(path.clone()) {
            Ok(_) => (),
            Err(e) => return Err(SessionError::CannotCreateDir(e)),
        };

        // Create the archive dir
        let mut arch_path: PathBuf = path.clone();
        arch_path.push("arch");
        match fs::create_dir_all(arch_path.clone()) {
            Ok(_) => (),
            Err(e) => return Err(SessionError::CannotCreateDir(e)),
        };

        // Create the log file path
        let mut log_file_path = path.clone();
        log_file_path.push(format!("{}.log", exec_name));

        // Create sender/receiver
        let (tx, rx) = channel();

        // Save copy in the static
        SAVE_SENDER.init_once(|| Mutex::new(tx.clone()));

        // Create the stop flag
        let save_stop = Arc::new(AtomicBool::new(false));

        // Spawn background thread
        let session_root = path.clone();
        let stop = save_stop.clone();
        thread::spawn(move || save_thread(stop, session_root, rx));

        // Build the session struct
        Ok(Session {
            session_root: path,
            arch_root: arch_path,
            log_file_path,
            save_sender: tx,
            save_stop,
        })
    }

    /// Exit the session, waiting for the save thread to finish any pending actions
    pub fn exit(self) {
        self.save_stop
            .store(true, std::sync::atomic::Ordering::Relaxed);

        info!("Stopping save thread");

        // Wait for the stop to be set to false, which is the save thread's way of indicating it's
        // finished saving data
        while self.save_stop.load(std::sync::atomic::Ordering::Relaxed) {}

        info!("Save thread exited");
    }

    /// Saves the given data to the given session-relative path in a background thread.
    pub fn save<P: AsRef<Path>, T: Serialize + Send + 'static>(&self, path: P, data: T) {
        if let Err(e) = self
            .save_sender
            .send((path.as_ref().to_path_buf(), Box::new(data)))
        {
            warn!(
                "Could not send data to be saved to path {:?}: {}",
                path.as_ref(),
                e
            )
        }
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
                None => std::f64::NAN,
            }
        }
        None => panic!("Cannot get the session epoch!"),
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
        None => panic!("Cannot get the session epoch!"),
    }
}

/// Save the given data into the session-relative path
pub fn save<P: AsRef<Path>, T: Serialize + Send + 'static>(path: P, data: T) {
    match SAVE_SENDER.get() {
        Some(m) => match m.lock() {
            Ok(s) => match s.send((path.as_ref().to_path_buf(), Box::new(data))) {
                Ok(_) => (),
                Err(e) => warn!(
                    "Couldn't send data to save thread for file {:?}: {}",
                    path.as_ref(),
                    e
                ),
            },
            Err(_) => {
                warn!("Couldn't get lock on save sender");
            }
        },
        None => {
            warn!("Cannot save data as session is not initialised yet");
        }
    }
}

/// Saves the given data to the path, appending a timestamp before the path's extension
pub fn save_with_timestamp<P: AsRef<Path>, T: Serialize + Send + 'static>(path: P, data: T) {
    let stem = path.as_ref().file_stem().unwrap_or(OsStr::new(""));

    let mut file_name = stem.to_os_string();
    file_name.push("_");
    file_name.push(Utc::now().format(TIMESTAMP_FORMAT).to_string());

    if let Some(ext) = path.as_ref().extension() {
        file_name.push(".");
        file_name.push(ext);
    }

    let path = path.as_ref().with_file_name(file_name);

    save(path, data);
}

// -----------------------------------------------------------------------------------------------
// PRIVATE FUNCTIONS
// -----------------------------------------------------------------------------------------------

fn save_thread(
    stop: Arc<AtomicBool>,
    session_root: PathBuf,
    receiver: Receiver<(PathBuf, Box<dyn Serialize + Send>)>,
) {
    loop {
        if let Ok((path, data)) = receiver.try_recv() {
            let full_path = session_root.join(path);
            match full_path.extension().map(|s| s.to_str()).flatten() {
                Some("json") => {
                    // Create the parent path if needed
                    let parent = match full_path.parent() {
                        Some(p) => p,
                        None => {
                            warn!("Couldn't find parent directory for {:?}", full_path);
                            continue;
                        }
                    };
                    if let Err(e) = std::fs::create_dir_all(parent) {
                        warn!("Couldn't create parent directory for {:?}", full_path);
                        continue;
                    }

                    let file = match OpenOptions::new()
                        .write(true)
                        .truncate(true)
                        .create(true)
                        .open(&full_path)
                    {
                        Ok(f) => f,
                        Err(e) => {
                            warn!("Couldn't create file {:?}: {}", full_path, e);
                            continue;
                        }
                    };

                    match serde_json::to_writer_pretty(&file, &data) {
                        Ok(_) => (),
                        Err(e) => {
                            warn!("Couldn't serialize data for file {:?}: {}", full_path, e);
                            continue;
                        }
                    }
                }
                ext => warn!(
                    "Unrecognised file path extension for {:?} (got {:?})",
                    full_path, ext
                ),
            }
        } else {
            // If there's no data check if we should stop, if so exit and set the stop flag to
            // false to indicate were're done processing data
            if stop.load(std::sync::atomic::Ordering::Relaxed) {
                stop.store(false, std::sync::atomic::Ordering::Relaxed);
            }
        }
    }
}
