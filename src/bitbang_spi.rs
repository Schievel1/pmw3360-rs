// Bit-banging SPI Bus Implementation
//
// This module provides a software SPI implementation using GPIO bit-banging.
// It's designed for sensors that use a single bidirectional data line (half-duplex SPI).

use defmt::Format;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::ErrorType;
use embedded_hal_async::spi::SpiBus;

use crate::bidirectional_pin::BidirectionalPin;

/// Error type for bit-banging SPI
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum BitBangError {
    /// Generic SPI error (placeholder for compatibility)
    Bus,
}

impl embedded_hal::spi::Error for BitBangError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

/// Bit-banging SPI bus for half-duplex communication
///
/// This implements the `embedded_hal::spi::SpiBus` trait using GPIO bit-banging.
/// It's designed for sensors like PMW3610 that use a single bidirectional data line.
///
/// # Type Parameters
/// - `SCK`: SPI clock pin (output)
/// - `SDIO`: Bidirectional data pin (implements `BidirectionalPin`)
///
/// # Example
///
/// ```ignore
/// use pmw3610_rs::{BitBangSpiBus, Pmw3610, Pmw3610Config};
///
/// let sck = /* your clock pin */;
/// let sdio = /* your bidirectional data pin */;
/// let cs = /* your chip select pin */;
///
/// let spi_bus = BitBangSpiBus::new(sck, sdio);
/// let sensor = Pmw3610::new(spi_bus, cs, None, Pmw3610Config::default());
/// ```
pub struct BitBangSpiBus<SCK, SDIO>
where
    SCK: OutputPin,
    SDIO: BidirectionalPin,
{
    sck: SCK,
    sdio: SDIO,
}

impl<SCK, SDIO> BitBangSpiBus<SCK, SDIO>
where
    SCK: OutputPin,
    SDIO: BidirectionalPin,
{
    /// Create a new bit-banging SPI bus
    pub fn new(mut sck: SCK, sdio: SDIO) -> Self {
        let _ = sck.set_high();
        Self { sck, sdio }
    }

    #[inline(always)]
    fn spi_delay() {
        // Short busy-wait delay for SPI timing
        // This is approximately 32 cycles at typical clock speeds
        for _ in 0..32 {
            core::hint::spin_loop();
        }
    }

    /// Write a single byte over the bidirectional SPI (MSB first)
    fn write_byte(&mut self, byte: u8) {
        self.sdio.set_as_output();

        for i in (0..8).rev() {
            if (byte >> i) & 1 == 1 {
                self.sdio.set_high();
            } else {
                self.sdio.set_low();
            }
            Self::spi_delay();

            let _ = self.sck.set_low();
            Self::spi_delay();

            let _ = self.sck.set_high();
            Self::spi_delay();
        }
    }

    /// Read a single byte from the bidirectional SPI (MSB first)
    fn read_byte(&mut self) -> u8 {
        self.sdio.set_as_input();

        let mut byte = 0u8;

        for i in (0..8).rev() {
            let _ = self.sck.set_low();
            Self::spi_delay();

            let _ = self.sck.set_high();
            Self::spi_delay();

            if self.sdio.is_high() {
                byte |= 1 << i;
            }
        }

        byte
    }
}

impl<SCK, SDIO> ErrorType for BitBangSpiBus<SCK, SDIO>
where
    SCK: OutputPin,
    SDIO: BidirectionalPin,
{
    type Error = BitBangError;
}

impl<SCK, SDIO> SpiBus for BitBangSpiBus<SCK, SDIO>
where
    SCK: OutputPin,
    SDIO: BidirectionalPin,
{
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words.iter_mut() {
            *word = self.read_byte();
        }
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &word in words {
            self.write_byte(word);
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        // For half-duplex: write first, then read
        self.write(write).await?;
        self.read(read).await?;
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        // For half-duplex: read replaces written data
        for word in words.iter_mut() {
            self.write_byte(*word);
            *word = self.read_byte();
        }
        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
