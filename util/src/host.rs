//! Host platform (linux for example) utility functions

use uname;

/// Retrieve uname information.
pub fn get_uname() -> std::io::Result<uname::Info> {
    uname::uname()
}