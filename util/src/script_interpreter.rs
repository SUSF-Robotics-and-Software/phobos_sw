//! # Phobos rover script interpreter module
//!
//! This module provides an interpreter for Phobos Rover Scripts, allowing 
//! telecommands to be executed from these scripts.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::fs;
use regex::RegexBuilder;
use thiserror::Error;

// Internal
use comms_if::tc::{Tc, TcParseError};
use crate::session::get_elapsed_seconds;

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// A command which is scripted to occur at a specific time.
pub struct Command {
    /// The time the command is supposed to execute at
    exec_time_s: f64,

    /// The Telecommand to run
    tc: Tc
}

/// A script interpreter.
///
/// After initialising with the path to the script to run use `.get_pending` to
/// acquire a list of telecommands that need executing.
pub struct ScriptInterpreter {
    _script_path: PathBuf,
    cmds: VecDeque<Command>
}

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

#[derive(Debug, Error)]
pub enum ScriptError {
    #[error("Could not find the script at {0}")]
    ScriptNotFound(String),

    #[error("Could not load the script: {0}")]
    ScriptLoadError(std::io::Error),

    #[error("The script is empty (or is so bad it can't be read)")]
    ScriptEmpty,

    #[error(
        "Script contains an invalid timestamp: {0}. \
        Should be a float (like 1.0)")]
    InvalidTimestamp(String),

    #[error("Script contains an invalid TC at {0} s: {1}")]
    InvalidTc(f64, TcParseError)
}

pub enum PendingTcs {
    None,
    Some(Vec<Tc>),
    EndOfScript
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl ScriptInterpreter {

    /// Create a new interpreter from the given script path.
    pub fn new<P: AsRef<Path>>(script_path: P) -> Result<Self, ScriptError> {

        // Get the path in a buffer
        let path = PathBuf::from(script_path.as_ref());
        
        // Check that the script file exists.
        if !path.exists() {
            return Err(
                ScriptError::ScriptNotFound(path.to_str().unwrap().to_string()));
        }

        // Load the script into a string
        let script = match fs::read_to_string(script_path) {
            Ok(s) => s,
            Err(e) => return Err(ScriptError::ScriptLoadError(e))
        };

        // Empty queue of commands
        let mut tc_queue: VecDeque<Command> = VecDeque::new();

        // Go through the script executing __the magic regex__.
        let re = RegexBuilder::
            new(r"^\s*(\d+(\.\d+)?)\s*:\s*([^;]*);")
            .multi_line(true)
            .build()
            .unwrap();

        let mut num_caps = 0;

        for cap in re.captures_iter(&script) {
            // Parse the exec time
            let exec_time_s: f64 = match cap.get(1).unwrap().as_str().parse() {
                Ok(t) => t,
                Err(e) => return Err(
                    ScriptError::InvalidTimestamp(format!("{}", e)))
            };

            // Parse the TC from the payload. The scripts contain JSON only.
            let tc = match Tc::from_json(
                cap.get(3).unwrap().as_str()) 
            {
                Ok(c) => c,
                Err(e) => return Err(ScriptError::InvalidTc(
                    exec_time_s, e
                ))
            };

            // Build command from the match
            tc_queue.push_back(Command {
                exec_time_s,
                tc
            });

            num_caps += 1;
        }

        if num_caps == 0 {
            return Err(ScriptError::ScriptEmpty)
        }

        Ok(ScriptInterpreter {
            _script_path: path,
            cmds: tc_queue
        })
    }

    /// Return a vector of pending TCs, or `None` if no TCs need executing now.
    pub fn get_pending_tcs(&mut self) -> PendingTcs {

        // If the queue is empty the script is over and we return the end of
        // script variant
        if self.cmds.len() == 0 {
            return PendingTcs::EndOfScript
        }

        let mut tc_vec: Vec<Tc> = vec![];

        let current_time_s = get_elapsed_seconds();

        // Peek items from the queue, if the head's exec time is lower than
        // the current time add it to the vector, and keep adding TCs until
        // the exec times are larger than the current time.
        while 
            self.cmds.len() > 0
            &&
            self.cmds.front().unwrap().exec_time_s < current_time_s
        {
            tc_vec.push(self.cmds.pop_front().unwrap().tc);
        }

        // If the vector is longer than 0 return Some, otherwise None
        if tc_vec.len() > 0 {
            PendingTcs::Some(tc_vec)
        }
        else {
            PendingTcs::None
        }
    }

    /// Get the number of TCs in the script
    pub fn get_num_tcs(&self) -> usize {
        self.cmds.len()
    }

    /// Get the length of the script in seconds
    pub fn get_duration(&self) -> f64 {
        match self.cmds.back() {
            Some(c) => c.exec_time_s,
            None => 0f64
        }
    }
}