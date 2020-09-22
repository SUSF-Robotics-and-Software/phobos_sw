//! Host platform (linux for example) utility functions

use std::path::PathBuf;

// use uname;

/// Retrieve uname information.
pub fn get_uname() -> std::io::Result<String> {
    // uname::uname()
    Ok("HOST INFO NOT YET AVAILABLE".to_string())
}

pub fn get_phobos_sw_root() -> Result<PathBuf, std::env::VarError> {
    match std::env::var("SUSF_PHOBOS_SW_ROOT") {
        Ok(s) => Ok(s.into()),
        Err(e) => Err(e)
    }
}