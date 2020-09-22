//! Generic logger utility functions

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External imports
use log::{self, info};
use fern;
use colored::{ColoredString, Colorize};
use thiserror::Error;

// Internal imports
use crate::session;

// Re-exports
pub use log::LevelFilter;

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Errors associated with initialising the logger.
#[derive(Debug, Error)]
pub enum LoggerInitError {
    #[error("Expected a log level less than `INFO`, found `{0}`")]
    InvalidMinLogLevel(log::LevelFilter),

    #[error("Error initialising the log file: {0}")]
    LogFileInitError(std::io::Error),

    #[error("An error occured while setting up the logger: {0}")]
    FernInitError(log::SetLoggerError)
}

// ---------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ---------------------------------------------------------------------------

/// Initialise the logger for this execution.
/// 
/// # Notes
/// 
/// - `min_level` must be greater than `log::Level::Info`.
/// 
/// # Safety
/// 
/// - This function must only be called once to prevent corrupting logs.
pub fn logger_init(
    min_level: self::LevelFilter, 
    session: &session::Session
) -> Result<(), LoggerInitError> {

    if min_level < log::Level::Info {
        return Err(LoggerInitError::InvalidMinLogLevel(min_level))
    }

    // Setup the logger using fern's builder pattern
    match fern::Dispatch::new()
        .format(|out, message, record| {

            // If debug or trace include the target, otherwise don't include it
            if record.level() > log::Level::Info {
                out.finish(format_args!(
                    "[{:10.6} {}] {}: {}",
                    session::get_elapsed_seconds(),
                    level_to_str(record.level()),
                    record.target(),
                    message
                ))
            }
            else {
                out.finish(format_args!(
                    "[{:10.6} {}] {}",
                    session::get_elapsed_seconds(),
                    level_to_str(record.level()),
                    message
                ))
            }

        })
        .level(min_level)
        .level_for("zmq", LevelFilter::Info)
        .chain(std::io::stdout())
        .chain(match fern::log_file(session.log_file_path.clone()) {
            Ok(f) => f,
            Err(e) => return Err(LoggerInitError::LogFileInitError(e))
        })
        .apply() {
            Ok(_) => (),
            Err(e) => return Err(LoggerInitError::FernInitError(e))
        };
    
    info!("Logging initialised");
    info!("    Session epoch: {}", session::get_epoch());
    info!("    Log level: {:?}", min_level);
    info!("    Log file path: {:?}", session.log_file_path);

    Ok(())
}

// ---------------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ---------------------------------------------------------------------------

/// Get the string representation of a log level
fn level_to_str(level: log::Level) -> ColoredString {
    match level {
        log::Level::Trace => "TRC".dimmed().italic(),
        log::Level::Debug => "DBG".dimmed(),
        log::Level::Info  => "INF".normal(),
        log::Level::Warn  => "WRN".yellow(),
        log::Level::Error => "ERR".red().bold()
    }
}
