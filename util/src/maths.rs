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

/// Apply polynomial coefficients to a value
pub fn poly_val<T>(value: &T, coeffs: &Vec<T>) -> T
where
    T: Float + std::ops::Mul + std::ops::Add + std::ops::AddAssign
{
    let mut res = T::from(0).unwrap();

    for i in 0..(coeffs.len() as i32) {
        res += value.powi(coeffs.len() as i32 - 1 - i) * coeffs[i as usize];
    }

    res
}

pub fn clamp<T>(value: &T, min: &T, max: &T) -> T 
where
    T: Float + std::ops::Mul + std::ops::Add + std::ops::AddAssign
{
    let mut ret = *value;

    if ret > *max {
        ret = *max
    }
    if ret < *min {
        ret = *min
    }

    ret
}

/// Get the signed angular distance between two angles in the range of [0, 2pi].
///
/// This function will return the shortest signed distance between a and b accounting for wrapping
/// between 0 and 2pi.
pub fn get_ang_dist_2pi<T>(a: T, b: T) -> T
where
    T: Float + std::ops::Mul + std::ops::Add + std::ops::Sub
{
    let tau_t: T = T::from(std::f64::consts::TAU).unwrap();
    
    let c = rem_euclid(a - b, tau_t);
    let d = rem_euclid(b - a, tau_t);

    if c < d {
        return -c
    }
    else {
        return d
    }

    // let mut c: T = a - b;
    
    // c = rem_euclid(c + pi_t, tau_t) - pi_t;

    // if c > pi_t {
    //     c = c - tau_t;
    // }
    // if c < -pi_t {
    //     c = c + tau_t;
    // }

    // c
}

/// Calculates the least nonnegative remainder of `lhs (mod rhs)`.
/// 
/// This function is taken from the std library as num is missing it.
///
/// In particular, the return value `r` satisfies `0.0 <= r < rhs.abs()` in
/// most cases. However, due to a floating point round-off error it can
/// result in `r == rhs.abs()`, violating the mathematical definition, if
/// `self` is much smaller than `rhs.abs()` in magnitude and `self < 0.0`.
/// This result is not an element of the function's codomain, but it is the
/// closest floating point number in the real numbers and thus fulfills the
/// property `self == self.div_euclid(rhs) * rhs + self.rem_euclid(rhs)`
/// approximatively.
pub fn rem_euclid<T>(lhs: T, rhs: T) -> T
where
    T: Float + std::ops::Mul + std::ops::Add + std::ops::Sub + std::ops::Rem
{
    let r = lhs % rhs;
    if r < T::from(0.0).unwrap() { r + rhs.abs() } else { r }
}

/// Map a value in the range [-pi, pi] to [0, 2pi]
pub fn map_pi_to_2pi<T>(value: T) -> T 
where
    T: Float
{
    let tau_t: T = T::from(std::f64::consts::TAU).unwrap();

    if value < T::from(0).unwrap() {
        return tau_t + value;
    }
    else {
        return value;
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_get_ang_dist_2pi() {
        const TAU: f64 = std::f64::consts::TAU;
        const PI: f64 = std::f64::consts::PI;

        assert_eq!(get_ang_dist_2pi(1f64, 2f64), 1f64);
        assert_eq!(get_ang_dist_2pi(2f64, 1f64), -1f64);
        assert_eq!(get_ang_dist_2pi(0f64, TAU), 0f64);
        assert_eq!(get_ang_dist_2pi(TAU, 0f64), 0f64);
        assert_eq!(get_ang_dist_2pi(1f64, TAU), -1f64);
        assert_eq!(get_ang_dist_2pi(0f64, TAU - 1f64), -1f64);
        assert_eq!(get_ang_dist_2pi(TAU - 1f64, 1f64), 2f64);
    }
}