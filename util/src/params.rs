//! Generic parameters functions

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use serde::de::DeserializeOwned;
use std::fs::read_to_string;
use thiserror::Error;
use toml;

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// An error that occurs during loading of a parameter file.
#[derive(Debug, Error)]
pub enum LoadError {
    #[error("The software root environment variable (SUSF_PHOBOS_SW_ROOT) is not set")]
    SwRootNotSet,

    #[error("Cannot load the parmeter file: {0}")]
    FileLoadError(std::io::Error),

    #[error("Cannot read the parameter file: {0}")]
    DeserialiseError(toml::de::Error)
}

// ---------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ---------------------------------------------------------------------------

/// Load a parameter file
///
/// The file path is relative to the "phobos_sw/params" directory
pub fn load<P>(param_file_path: &str) -> Result<P, LoadError> 
where
    P: DeserializeOwned
{
    // Get the params dir
    let mut path = crate::host::get_phobos_sw_root()
        .map_err(|_| LoadError::SwRootNotSet)?;
    path.push("params");
    path.push(param_file_path);

    // Load the file into a string
    let params_str = match read_to_string(path) {
        Ok(s) => s,
        Err(e) => return Err(LoadError::FileLoadError(e))
    };

    // Parse the string into the parameter struct
    match toml::from_str(params_str.as_str()) {
        Ok(p) => Ok(p),
        Err(e) => Err(LoadError::DeserialiseError(e))
    }
}