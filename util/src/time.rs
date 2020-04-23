//! General time utility functions

use chrono;

/// Number of nanoseconds in a second
pub const NANOS_PER_SECOND: i64 = 1_000_000_000;

/// Convert a duration into a number of nanoseconds, or `None` if overflow
pub fn duration_to_nanos(duration: chrono::Duration) -> Option<f64> {
    if let Some(ns) = duration.num_nanoseconds() {
        Some(ns as f64 / NANOS_PER_SECOND as f64)
    }
    else {
        None
    }
}