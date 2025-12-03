// Bidirectional Pin Trait
//
// This module provides a trait for bidirectional GPIO pins that can switch
// between input and output modes. This is required for bit-banging SPI with
// single-wire bidirectional data lines (like PMW3610's SDIO).

/// Trait for a bidirectional GPIO pin (required for bit-banging SPI with PMW3610's SDIO line)
///
/// The PMW3610 uses a single bidirectional data line for SPI communication.
/// This trait abstracts over HAL-specific implementations.
pub trait BidirectionalPin {
    /// Set the pin as output mode
    fn set_as_output(&mut self);

    /// Set the pin as input mode
    fn set_as_input(&mut self);

    /// Set the pin high (only valid in output mode)
    fn set_high(&mut self);

    /// Set the pin low (only valid in output mode)
    fn set_low(&mut self);

    /// Read the pin state (only valid in input mode)
    fn is_high(&self) -> bool;

    /// Read the pin state (only valid in input mode)
    fn is_low(&self) -> bool {
        !self.is_high()
    }
}

// ============================================================================
// HAL-specific implementations
// ============================================================================

/// Embassy-nRF implementation of BidirectionalPin for Flex pin
#[cfg(feature = "embassy-nrf")]
impl<'d> BidirectionalPin for embassy_nrf::gpio::Flex<'d> {
    fn set_as_output(&mut self) {
        embassy_nrf::gpio::Flex::set_as_output(self, embassy_nrf::gpio::OutputDrive::Standard);
    }

    fn set_as_input(&mut self) {
        embassy_nrf::gpio::Flex::set_as_input(self, embassy_nrf::gpio::Pull::None);
    }

    fn set_high(&mut self) {
        embassy_nrf::gpio::Flex::set_high(self);
    }

    fn set_low(&mut self) {
        embassy_nrf::gpio::Flex::set_low(self);
    }

    fn is_high(&self) -> bool {
        embassy_nrf::gpio::Flex::is_high(self)
    }
}
