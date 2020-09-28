//! # Servo Controller Module
//!
//! This module provides a unified servo control interface which can abstract over different types
//! of servo driver boards.
//!
//! TODO: This module is still in progress and shouldn't be used right now

// ------------------------------------------------------------------------------------------------
// MODULES
// ------------------------------------------------------------------------------------------------

/// [`ServoDriver`] implementation for the Adafruit PCA9685 16 channel servo driver board.
pub mod pca9685;

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use std::{collections::HashMap, hash::Hash};
use embedded_hal::blocking::i2c;
use serde::{Serialize, Deserialize};

// ------------------------------------------------------------------------------------------------
// TRAITS
// ------------------------------------------------------------------------------------------------

/// Trait to provide a unified API for accessing servo driver boards.
pub trait ServoDriver {

    /// The type that the underlying driver uses for channel identification
    type Channel;

    // TODO
    // fn new();

    /// Set the duty cycle of a channel.
    ///
    /// ## Arguments
    /// - `id` - The channel ID to set the duty cycle for
    /// - `duty_cycle` - The duty cycle to set. Must be a value between 0.0 and 1.0. Values outside
    ///   this range will be rejected.
    fn set_duty_cycle(&mut self, channel: Self::Channel, duty_cycle: f64) -> Result<(), ServoError>;

}

// ------------------------------------------------------------------------------------------------
// STRUCTS
// ------------------------------------------------------------------------------------------------

pub struct ServoCtrl<D, S>
where
    D: ServoDriver,
    S: Eq + Hash
{
    drivers: Vec<D>,

    servo_config_map: HashMap<S, ServoConfig<D::Channel>>,
}
#[derive(Serialize, Deserialize)]
pub struct ControllerConfig<S, C>
where
    S: Eq + Hash
{
    pub num_boards: usize,

    pub board_addresses: Vec<u16>,

    pub servo_config: HashMap<S, ServoConfig<C>>
}

// ------------------------------------------------------------------------------------------------
// ENUMS
// ------------------------------------------------------------------------------------------------

#[derive(thiserror::Error, Debug)]
pub enum ServoError {
    #[error("An I2C error occured")]
    I2c,

    #[error("Duty cycle must be between 0.0 and 1.0")]
    InvalidDutyCycle
}

#[derive(Serialize, Deserialize, Debug)]
pub enum ServoConfig<C> {
    Positional {
        channel: (usize, C),
        min_angle_rad: f64,
        max_angle_rad: f64,
    },
    Continuous {
        channel: (usize, C),
        min_speed_rads: f64,
        max_speed_rads: f64
    }
}

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl<D, S> ServoCtrl<D, S> 
where 
    D: ServoDriver,
    S: Eq + Hash
{
    /// Create a new servo controller.
    ///
    /// ## Arguments
    /// - `drivers` - A vector of initialised [`ServoDriver`] boards
    /// - `config` - A configuration for the servos managed by this controller
    pub fn new(
        config: ControllerConfig<S, D::Channel>
    ) -> Result<Self, ServoError> {

        todo!("ServoCtrl not currently implemented");

        // Create drivers

        // TODO: Check the config is valid

        // Ok(Self {
        //     drivers,
        //     servo_config_map: config.servo_config
        // })
    }
}