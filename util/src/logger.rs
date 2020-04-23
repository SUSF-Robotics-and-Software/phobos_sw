//! Generic logger utility functions

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use log::{self, info};
use fern;
use chrono::prelude::*;
use conquer_once::spin::OnceCell;
use colored::{ColoredString, Colorize};

// Re-exports
pub use log::LevelFilter;

// ---------------------------------------------------------------------------
// STATICS
// ---------------------------------------------------------------------------

static INIT_TIME: OnceCell<DateTime<Utc>> = OnceCell::uninit();

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// Errors associated with initialising the logger.
#[derive(Debug)]
pub enum LoggerInitError {
    InvalidMinLogLevel,
    LogFileInitError(std::io::Error),
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
    log_file_path: &str
) -> Result<(), LoggerInitError> {

    if min_level < log::Level::Info {
        println!(
            "Cannot initialise logging with a level less than log::Level::Info");
        return Err(LoggerInitError::InvalidMinLogLevel)
    }

    // Initialise the start time of the logger
    INIT_TIME.try_init_once(|| {
        Utc::now()
    }).expect("Failed to initialise logging base time");

    // Setup the logger using fern's builder pattern
    match fern::Dispatch::new()
        .format(|out, message, record| {
            
            let elapsed: chrono::Duration = match INIT_TIME.get() {
                Some(&t0) => Utc::now() - t0,
                None => chrono::Duration::zero()
            };

            // If debug or trace include the target, otherwise don't include it
            if record.level() > log::Level::Info {
                out.finish(format_args!(
                    "[{} {}] {}: {}",
                    match crate::time::duration_to_nanos(elapsed) {
                        Some(s) => format!("{:10.6}", s),
                        None => String::from("ERR OVRFLOW")
                    },
                    level_to_str(record.level()),
                    record.target(),
                    message
                ))
            }
            else {
                out.finish(format_args!(
                    "[{} {}] {}",
                    match crate::time::duration_to_nanos(elapsed) {
                        Some(s) => format!("{:10.6}", s),
                        None => String::from("ERR OVRFLOW")
                    },
                    level_to_str(record.level()),
                    message
                ))
            }

        })
        .level(min_level)
        .chain(std::io::stdout())
        .chain(match fern::log_file(log_file_path) {
            Ok(f) => f,
            Err(e) => return Err(LoggerInitError::LogFileInitError(e))
        })
        .apply() {
            Ok(_) => (),
            Err(e) => return Err(LoggerInitError::FernInitError(e))
        };
    
    info!("Logging initialised");
    info!("    Log init time: {}", INIT_TIME.get().unwrap());
    info!("    Log level: {:?}", min_level);
    info!("    Log file path: {}", log_file_path);

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