//! Utility library for Phobos Rover Software

// ---------------------------------------------------------------------------
// MODULES
// ---------------------------------------------------------------------------

pub mod archive;
pub mod host;
#[macro_use]
pub mod logger;
pub mod maths;
pub mod module;
pub mod params;
pub mod session;
pub mod script_interpreter;
pub mod time;

// ---------------------------------------------------------------------------
// REEXPORTS
// ---------------------------------------------------------------------------

pub use comms_if;

// ---------------------------------------------------------------------------
// MACROS
// ---------------------------------------------------------------------------

/// Fire an unrecoverable error which will panic.
/// 
/// # Notes
/// = It is prefered to return a `Result<_,Error>` instead of raising an error
///   as this allows the application to potentially handle.
#[macro_export]
macro_rules! raise_error {
    () => ({
        log::error!("Explicit error raised.");
        std::panic!("Unrecoverable error");
    });
    ($fmt:expr) => ({
        log::error!("{}", $fmt);
        std::panic!("Unrecoverable error");
    });
    ($fmt:expr, $($arg:tt)*) => ({
        log::error!("{}", std::format_args!($fmt, $($arg)+));
        std::panic!("Unrecoverable error");
    });
}