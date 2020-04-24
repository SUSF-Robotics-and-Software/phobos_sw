//! Generic parameters functions

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use serde::de::DeserializeOwned;
use std::fs::read_to_string;
use toml;

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// An error that occurs during loading of a parameter file.
#[derive(Debug)]
pub enum LoadError {
    FileLoadError(std::io::Error),
    DeserialiseError(toml::de::Error)
}

// ---------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ---------------------------------------------------------------------------

/// Load a parameter file
pub fn load<P>(param_file_path: &str) -> Result<P, LoadError> 
where
    P: DeserializeOwned
{

    // Load the file into a string
    let params_str = match read_to_string(param_file_path) {
        Ok(s) => s,
        Err(e) => return Err(LoadError::FileLoadError(e))
    };

    // Parse the string into the parameter struct
    match toml::from_str(params_str.as_str()) {
        Ok(p) => Ok(p),
        Err(e) => Err(LoadError::DeserialiseError(e))
    }
}