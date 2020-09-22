//! Host platform (linux for example) utility functions

// use uname;

/// Retrieve uname information.
pub fn get_uname() -> std::io::Result<String> {
    // uname::uname()
    Ok("HOST INFO NOT YET AVAILABLE".to_string())
}