//! [`ServoDriver`] implementation for the PCA9685 driver

// ------------------------------------------------------------------------------------------------
// IMPORTS
// ------------------------------------------------------------------------------------------------

use pwm_pca9685::{Channel, Pca9685};
use embedded_hal::blocking::i2c::{Write, WriteRead};

use super::{ServoDriver, ServoError};

// ------------------------------------------------------------------------------------------------
// CONSTANTS
// ------------------------------------------------------------------------------------------------

const MAX_PWM: u16 = 4096;

// ------------------------------------------------------------------------------------------------
// IMPLS
// ------------------------------------------------------------------------------------------------

impl<I2C, E> ServoDriver for Pca9685<I2C>
where 
    I2C: Write<Error = E> + WriteRead<Error = E>
{
    type Channel = Channel;

    fn set_duty_cycle(
        &mut self, 
        channel: Self::Channel, 
        duty_cycle: f64
    ) -> Result<(), ServoError> {

        // If the duty cycle is out of range return an error
        if duty_cycle < 0.0 || duty_cycle > 1.0 {
            return Err(ServoError::InvalidDutyCycle)
        }

        match self.set_channel_on(
            channel, 
            (duty_cycle*(MAX_PWM as f64)) as u16
        ) {
            Ok(_) => Ok(()),
            Err(pwm_pca9685::Error::I2C(_)) => Err(ServoError::I2c),
            Err(pwm_pca9685::Error::InvalidInputData) => Err(ServoError::InvalidDutyCycle)
        }
    }
}