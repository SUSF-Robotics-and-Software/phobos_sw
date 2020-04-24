//! Locomotion Configuration structure

// ---------------------------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------------------------

// External
use inner::inner;
use super::{NUM_DRV_AXES, NUM_STR_AXES};

// ---------------------------------------------------------------------------
// DATA STRUCTURES
// ---------------------------------------------------------------------------

// Stores a locomotion configuration - the positions of all steer axes and 
/// speed of all drive axes.
#[derive(Clone, Copy)]
pub struct LocoConfig {
    
    /// All possible steer axes
    pub str_axes: [Axis<AxisData>; NUM_STR_AXES],

    /// All possible drive axes
    pub drv_axes: [Axis<AxisData>; NUM_DRV_AXES]
}

/// Used for getting an iterator over LocoConfig axes
pub struct LocoConfigIntoIterator {
    cfg: LocoConfig,
    idx: usize
}

/// Data about a particular axis
#[derive(Clone, Copy)]
pub struct AxisData {
    /// Absolute position of the axis in radians relative to the axis's 
    /// predefined "zero" position. For STR this is straight ahead. For DRV it
    /// is the position of the axis when the system was initialised.
    /// 
    /// Units: radians
    pub abs_pos_rad: f64,

    /// Rate of the axis.
    /// 
    /// Units: radians/second for Absolute, N/A for normalised
    pub rate: AxisRate,

    /// The position of the axis's centre in the Rover body frame.
    /// 
    /// Units: meters,
    /// Frame: Rover body
    pub pos_m_rb: [f64; 3]
}

// ---------------------------------------------------------------------------
// IMPLEMENTATIONS
// ---------------------------------------------------------------------------

impl IntoIterator for LocoConfig {
    type Item = Axis<(AxisData, AxisData)>;
    type IntoIter = LocoConfigIntoIterator;

    fn into_iter(self) -> Self::IntoIter {
        LocoConfigIntoIterator {
            cfg: self,
            idx: 0
        }
    }
}

impl Iterator for LocoConfigIntoIterator {
    type Item = Axis<(AxisData, AxisData)>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.idx {
            0 => {
                let drv_data = inner!(
                    self.cfg.drv_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::LF(_)))
                        .nth(0).unwrap(), 
                    if Axis::LF);
                let str_data = inner!(
                    self.cfg.str_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::LF(_)))
                        .nth(0).unwrap(), 
                    if Axis::LF);
                Some(Axis::LF((drv_data.clone(), str_data.clone())))
            },
            1 => {
                let drv_data = inner!(
                    self.cfg.drv_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::LM(_)))
                        .nth(0).unwrap(), 
                    if Axis::LM);
                let str_data = inner!(
                    self.cfg.str_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::LM(_)))
                        .nth(0).unwrap(), 
                    if Axis::LM);
                Some(Axis::LM((drv_data.clone(), str_data.clone())))
            },
            2 => {
                let drv_data = inner!(
                    self.cfg.drv_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::LR(_)))
                        .nth(0).unwrap(), 
                    if Axis::LR);
                let str_data = inner!(
                    self.cfg.str_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::LR(_)))
                        .nth(0).unwrap(), 
                    if Axis::LR);
                Some(Axis::LR((drv_data.clone(), str_data.clone())))
            },
            3 => {
                let drv_data = inner!(
                    self.cfg.drv_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::RF(_)))
                        .nth(0).unwrap(), 
                    if Axis::RF);
                let str_data = inner!(
                    self.cfg.str_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::RF(_)))
                        .nth(0).unwrap(), 
                    if Axis::RF);
                Some(Axis::RF((drv_data.clone(), str_data.clone())))
            },
            4 => {
                let drv_data = inner!(
                    self.cfg.drv_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::RM(_)))
                        .nth(0).unwrap(), 
                    if Axis::RM);
                let str_data = inner!(
                    self.cfg.str_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::RM(_)))
                        .nth(0).unwrap(), 
                    if Axis::RM);
                Some(Axis::RM((drv_data.clone(), str_data.clone())))
            },
            5 => {
                let drv_data = inner!(
                    self.cfg.drv_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::RR(_)))
                        .nth(0).unwrap(), 
                    if Axis::RR);
                let str_data = inner!(
                    self.cfg.str_axes
                        .iter()
                        .filter(|a| matches!(a, Axis::RR(_)))
                        .nth(0).unwrap(), 
                    if Axis::RR);
                Some(Axis::RR((drv_data.clone(), str_data.clone())))
            },
            _ => None
        }
    }
}

// ---------------------------------------------------------------------------
// ENUMERATIONS
// ---------------------------------------------------------------------------

/// An enumeration over the possible axis positions.
#[derive(Clone, Copy, PartialOrd, Ord, PartialEq, Eq)]
pub enum Axis<T> {

    /// Left front
    LF(T),
    /// Left middle
    LM(T),
    /// Left rear
    LR(T),

    /// Right front
    RF(T),
    /// Right middle
    RM(T),
    /// Right rear
    RR(T)
}

impl<T> Axis<T> 
where T: Default
{
    /// Return an array of default values of `T` for all Axes.
    pub fn default_array() -> [Self; 6] {
        [
            Axis::LF(T::default()),
            Axis::LM(T::default()),
            Axis::LR(T::default()),
            Axis::RF(T::default()),
            Axis::RM(T::default()),
            Axis::RR(T::default()),
        ]
    }
}

impl<T> Axis<T>
where
    T: Copy,
{
    /// Create an array with the given value
    pub fn array(value: T) -> [Self; 6] {
        [
            Axis::LF(value),
            Axis::LM(value),
            Axis::LR(value),
            Axis::RF(value),
            Axis::RM(value),
            Axis::RR(value),
        ]
    }

    /// Make an axis array from a normal array
    pub fn into_axis_array(array: &[T; 6]) -> [Self; 6] {
        [
            Axis::LF(array[0]),
            Axis::LM(array[1]),
            Axis::LR(array[2]),
            Axis::RF(array[3]),
            Axis::RM(array[4]),
            Axis::RR(array[5]),
        ]
    }

    /// Unwrap the inner value of the axis enum.
    pub fn unwrap(&self) -> T {
        (match self {
            Axis::LF(v) => v,
            Axis::LM(v) => v,
            Axis::LR(v) => v,
            Axis::RF(v) => v,
            Axis::RM(v) => v,
            Axis::RR(v) => v
        }).clone()
    }
}

/// An enumeration which allows axis rates to be specifed as either Normalised
/// rates (-1 to +1) or absolute rates.
#[derive(Clone, Copy)]
pub enum AxisRate {
    Normalised(f64),
    Absolute(f64)
}

// ---------------------------------------------------------------------------
// MACROS
// ---------------------------------------------------------------------------

/// Useful macro in the `LocoConfig` iter implementation.
#[allow(unused_macros)]
macro_rules! matches(
    ($e:expr, $p:pat) => ({
        match $e {
            $p => true,
            _ => false
        }
    })
);