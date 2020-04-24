//! Utility maths functions

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

use num_traits::Float;

/// Map a value from one range into another.
pub fn lin_map<T>(source_range: (T, T), target_range: (T, T), value: T) -> T
where 
    T: Float 
{
    target_range.0 
        + ((value - source_range.0) 
        * (target_range.1 - target_range.0) 
        / (source_range.1 - source_range.0))
}
