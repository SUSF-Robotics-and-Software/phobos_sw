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

/// Return the euclidian norm (distance between) of two points.
///
/// If the points do not have the same number of dimentions then `None` is 
/// returned.
pub fn norm<T>(point_0: &[T], point_1: &[T]) -> Option<T> 
where
    T: Float + std::ops::AddAssign
{
    // Check that the dimentions match
    if point_0.len() != point_1.len() {
        return None;
    }

    // Sum all elements of the points
    let mut sum = T::from(0).unwrap();

    for i in 0..point_0.len() {
        sum += (point_0[i] - point_1[i]).powi(2);
    }

    // Return the squareroot of the sum
    Some(sum.sqrt())
}