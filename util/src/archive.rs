//! Struct archiving functionality
//!
//! To add archiving functionality to a struct implement the `Archive` trait.

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External imports
use std::path::Path;
use std::fs::{File, OpenOptions};
use csv::WriterBuilder;
pub use csv::Writer;
use serde::Serialize;

// Internal imports
use crate::session::{Session, get_elapsed_seconds};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

/// An object used to write CSV archive files.
#[derive(Default)]
pub struct Archiver {
    writer: Option<Writer<File>>
}

#[derive(Serialize)]
struct Record<T: Serialize> {
    time_s: f64,
    data: T
}

// ---------------------------------------------------------------------------
// TRAITS
// ---------------------------------------------------------------------------

/// A trait which enables a struct to be archived as a timestamped csv.
///
/// To implement this trait, the struct shall have an `Archiver` member which
/// shall be ignored by Serde using `#[serde(skip_serializing)]. The archiver
/// member shall be setup in the struct's `init` or `new` functions. 
pub trait Archived {
    /// Write the archives for this struct
    fn write(&mut self) -> Result<(), Box<dyn std::error::Error>>;
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl Archiver {
    /// Create a new archiver from a paricular path relative to the session's
    /// archive root.
    pub fn from_path<P: AsRef<Path>>(
        session: &Session, path: P
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut session_path = session.arch_root.clone();
        session_path.push(path);
        
        // Create the file if it does not exist
        std::fs::File::create(session_path.clone())?;

        // Open the file in append mode
        let file = match OpenOptions::new()
            .append(true).open(session_path)
        {
            Ok(f) => f,
            Err(e) => return Err(Box::new(e))
        };

        let w = WriterBuilder::new()
            .has_headers(true)
            .from_writer(file);

        Ok(Self {
            writer: Some(w)
        })
    }

    /// Serialise a record into the archive.
    pub fn serialise<T: serde::Serialize>(
        &mut self, record: T
    ) -> Result<(), Box<dyn std::error::Error>> {
        match self.writer {
            Some(ref mut w) => {
                // w.serialize(Record { 
                //     time_s: get_elapsed_seconds(),
                //     data: record
                // })?;
                w.serialize(record)?;
                w.flush()?
            },
            None => panic!("Cannot find an initialised writer!")
        }

        Ok(())
    }
}
