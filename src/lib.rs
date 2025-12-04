// PMW3610 Low-Power Mouse Sensor Driver (Rust/Embassy/RMK port)
//
// Ported from the Zephyr driver implementation:
// https://github.com/zephyrproject-rtos/zephyr/blob/d31c6e95033fd6b3763389edba6a655245ae1328/drivers/input/input_pmw3610.C

#![no_std]

mod bidirectional_pin;
mod bitbang_spi;

pub use bidirectional_pin::BidirectionalPin;
pub use bitbang_spi::{BitBangError, BitBangSpiBus};

use defmt::{debug, error, info, warn, Format};
use embassy_time::{Duration, Timer};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::spi::SpiBus;

// ============================================================================
// Page 0 registers
// ============================================================================
const PMW3610_PROD_ID: u8 = 0x00;
#[allow(dead_code)]
const PMW3610_REV_ID: u8 = 0x01;
const PMW3610_MOTION: u8 = 0x02;
#[allow(dead_code)]
const PMW3610_DELTA_X_L: u8 = 0x03;
#[allow(dead_code)]
const PMW3610_DELTA_Y_L: u8 = 0x04;
const PMW3610_DELTA_XY_H: u8 = 0x05;
const PMW3610_PERFORMANCE: u8 = 0x11;
const PMW3610_BURST_READ: u8 = 0x12;
const PMW3610_RUN_DOWNSHIFT: u8 = 0x1b;
const PMW3610_REST1_RATE: u8 = 0x1c;
const PMW3610_REST1_DOWNSHIFT: u8 = 0x1d;
const PMW3610_OBSERVATION1: u8 = 0x2d;
const PMW3610_SMART_MODE: u8 = 0x32;
const PMW3610_POWER_UP_RESET: u8 = 0x3a;
#[allow(dead_code)]
const PMW3610_SHUTDOWN: u8 = 0x3b;
const PMW3610_SPI_CLK_ON_REQ: u8 = 0x41;
const PWM3610_SPI_PAGE0: u8 = 0x7f;

// ============================================================================
// Page 1 registers
// ============================================================================
const PMW3610_RES_STEP: u8 = 0x05;
const PWM3610_SPI_PAGE1: u8 = 0x7f;

// ============================================================================
// Burst register offsets
// ============================================================================
const BURST_MOTION: usize = 0;
const BURST_DELTA_X_L: usize = 1;
const BURST_DELTA_Y_L: usize = 2;
const BURST_DELTA_XY_H: usize = 3;
#[allow(dead_code)]
const BURST_SQUAL: usize = 4;
const BURST_SHUTTER_HI: usize = 5;
const BURST_SHUTTER_LO: usize = 6;

const BURST_DATA_LEN_NORMAL: usize = BURST_DELTA_XY_H + 1;
const BURST_DATA_LEN_SMART: usize = BURST_SHUTTER_LO + 1;
#[allow(dead_code)]
const BURST_DATA_LEN_MAX: usize = if BURST_DATA_LEN_NORMAL > BURST_DATA_LEN_SMART {
    BURST_DATA_LEN_NORMAL
} else {
    BURST_DATA_LEN_SMART
};

// ============================================================================
// Init sequence values
// ============================================================================
const OBSERVATION1_INIT_MASK: u8 = 0x0f;
const PERFORMANCE_INIT: u8 = 0x0d;
const RUN_DOWNSHIFT_INIT: u8 = 0x04;
const REST1_RATE_INIT: u8 = 0x04;
const REST1_DOWNSHIFT_INIT: u8 = 0x0f;

// ============================================================================
// Constants
// ============================================================================
const PRODUCT_ID_PMW3610: u8 = 0x3e;
const SPI_WRITE: u8 = 0x80; // BIT(7)
const MOTION_STATUS_MOTION: u8 = 0x80; // BIT(7)
const SPI_CLOCK_ON_REQ_ON: u8 = 0xba;
const SPI_CLOCK_ON_REQ_OFF: u8 = 0xb5;
#[allow(dead_code)]
const RES_STEP_SWAP_XY_BIT: u8 = 7;
const RES_STEP_INV_X_BIT: u8 = 6;
const RES_STEP_INV_Y_BIT: u8 = 5;
const RES_STEP_RES_MASK: u8 = 0x1f;
const PERFORMANCE_FMODE_MASK: u8 = 0x0f << 4;
const PERFORMANCE_FMODE_NORMAL: u8 = 0x00 << 4;
const PERFORMANCE_FMODE_FORCE_AWAKE: u8 = 0x0f << 4;
const POWER_UP_RESET_VAL: u8 = 0x5a;
#[allow(dead_code)]
const POWER_UP_WAKEUP: u8 = 0x96;
#[allow(dead_code)]
const SHUTDOWN_ENABLE: u8 = 0xe7;
const SPI_PAGE0_1: u8 = 0xff;
const SPI_PAGE1_0: u8 = 0x00;
const SHUTTER_SMART_THRESHOLD: u16 = 45;
const SMART_MODE_ENABLE: u8 = 0x00;
const SMART_MODE_DISABLE: u8 = 0x80;

const PMW3610_DATA_SIZE_BITS: usize = 12;

// Timing constants
const RESET_DELAY_MS: u64 = 10;
const INIT_OBSERVATION_DELAY_MS: u64 = 10;
const CLOCK_ON_DELAY_US: u64 = 300;

// SPI timing constants (from PMW3610 datasheet)
const T_NCS_SCLK_US: u64 = 1;
const T_SRAD_US: u64 = 5;
const T_SRX_US: u64 = 2;
const T_SWX_US: u64 = 35;
const T_SCLK_NCS_WR_US: u64 = 20;
const T_BEXIT_US: u64 = 2;

// Resolution constants
const RES_STEP: u16 = 200;
const RES_MIN: u16 = 200;
const RES_MAX: u16 = 3200;

/// PMW3610 configuration
#[derive(Clone)]
pub struct Pmw3610Config {
    /// CPI resolution (200-3200, step 200). Set to -1 to use default.
    pub res_cpi: i16,
    /// Invert X axis
    pub invert_x: bool,
    /// Invert Y axis
    pub invert_y: bool,
    /// Swap X and Y axes
    pub swap_xy: bool,
    /// Force awake mode (disable power saving)
    pub force_awake: bool,
    /// Enable smart mode for better tracking on shiny surfaces
    pub smart_mode: bool,
}

impl Default for Pmw3610Config {
    fn default() -> Self {
        Self {
            res_cpi: -1, // Use default
            invert_x: false,
            invert_y: false,
            swap_xy: false,
            force_awake: false,
            smart_mode: false,
        }
    }
}

/// PMW3610 error types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
#[allow(dead_code)]
pub enum Pmw3610Error {
    /// SPI communication error
    Spi,
    /// Invalid product ID detected
    InvalidProductId(u8),
    /// Initialization failed
    InitFailed,
    /// Invalid CPI value
    InvalidCpi,
}

/// Motion data from the sensor
#[derive(Debug, Clone, Copy, Default)]
pub struct MotionData {
    pub dx: i16,
    pub dy: i16,
}

/// PMW3610 driver using embedded-hal SPI traits
///
/// This driver accepts any SPI bus implementation via the `embedded_hal::spi::SpiBus` trait.
/// For the PMW3610's half-duplex SPI communication, you can use the provided `BitBangSpiBus`
/// or any compatible hardware SPI peripheral.
///
/// # Type Parameters
/// - `SPI`: SPI bus implementing `embedded_hal::spi::SpiBus`
/// - `CS`: Chip select pin (active low)
/// - `MOTION`: Optional motion interrupt pin (active low)
///
/// # Example with bit-banging
///
/// ```ignore
/// use pmw3610_rs::{BitBangSpiBus, Pmw3610, Pmw3610Config};
///
/// let spi_bus = BitBangSpiBus::new(sck_pin, sdio_pin);
/// let mut sensor = Pmw3610::new(spi_bus, cs_pin, Some(motion_pin), Pmw3610Config::default());
/// sensor.init().await?;
/// ```
///
/// # Example with hardware SPI (if supported)
///
/// ```ignore
/// use pmw3610_rs::{Pmw3610, Pmw3610Config};
///
/// let spi_bus = /* your hardware SPI bus */;
/// let mut sensor = Pmw3610::new(spi_bus, cs_pin, Some(motion_pin), Pmw3610Config::default());
/// sensor.init().await?;
/// ```
pub struct Pmw3610<SPI, CS, MOTION>
where
    SPI: SpiBus,
    CS: OutputPin,
    MOTION: InputPin,
{
    spi: SPI,
    cs: CS,
    motion_gpio: Option<MOTION>,
    config: Pmw3610Config,
    smart_flag: bool,
}

impl<SPI, CS, MOTION> Pmw3610<SPI, CS, MOTION>
where
    SPI: SpiBus,
    CS: OutputPin,
    MOTION: InputPin,
{
    /// Create a new PMW3610 driver instance
    ///
    /// # Arguments
    /// - `spi`: SPI bus (use `BitBangSpiBus` for half-duplex bit-banging without hardware support)
    /// - `cs`: Chip select pin (active low)
    /// - `motion_gpio`: Optional motion interrupt pin (active low)
    /// - `config`: Sensor configuration
    pub fn new(spi: SPI, cs: CS, motion_gpio: Option<MOTION>, config: Pmw3610Config) -> Self {
        Self {
            spi,
            cs,
            motion_gpio,
            config,
            smart_flag: false,
        }
    }

    /// Check if motion is pending (motion GPIO is active low)
    pub fn motion_pending(&mut self) -> bool {
        match &mut self.motion_gpio {
            Some(gpio) => gpio.is_low().unwrap_or(true),
            None => true,
        }
    }

    // ========================================================================
    // Low-level timing helpers
    // ========================================================================

    #[inline(always)]
    fn short_delay() {
        for _ in 0..64 {
            core::hint::spin_loop();
        }
    }

    // ========================================================================
    // Register access functions
    // ========================================================================

    /// Read a single byte from a register
    async fn read_reg(&mut self, addr: u8) -> Result<u8, Pmw3610Error> {
        let _ = self.cs.set_low();
        Timer::after(Duration::from_micros(T_NCS_SCLK_US)).await;

        // Send address with read bit (bit 7 = 0)
        self.spi
            .write(&[addr & 0x7f])
            .await
            .map_err(|_| Pmw3610Error::Spi)?;

        Timer::after(Duration::from_micros(T_SRAD_US)).await;

        let mut value = [0u8];
        self.spi
            .read(&mut value)
            .await
            .map_err(|_| Pmw3610Error::Spi)?;

        Self::short_delay();
        let _ = self.cs.set_high();

        Timer::after(Duration::from_micros(T_SRX_US)).await;

        Ok(value[0])
    }

    /// Read multiple bytes using burst read
    async fn read_burst(&mut self, addr: u8, data: &mut [u8]) -> Result<(), Pmw3610Error> {
        let _ = self.cs.set_low();
        Timer::after(Duration::from_micros(T_NCS_SCLK_US)).await;

        // Send address with read bit (bit 7 = 0)
        self.spi
            .write(&[addr & 0x7f])
            .await
            .map_err(|_| Pmw3610Error::Spi)?;

        Timer::after(Duration::from_micros(T_SRAD_US)).await;

        self.spi.read(data).await.map_err(|_| Pmw3610Error::Spi)?;

        Self::short_delay();
        let _ = self.cs.set_high();

        Timer::after(Duration::from_micros(T_BEXIT_US)).await;

        Ok(())
    }

    /// Write a single byte to a register
    async fn write_reg(&mut self, addr: u8, value: u8) -> Result<(), Pmw3610Error> {
        let _ = self.cs.set_low();
        Timer::after(Duration::from_micros(T_NCS_SCLK_US)).await;

        // Send address with write bit (bit 7 = 1)
        self.spi
            .write(&[addr | SPI_WRITE, value])
            .await
            .map_err(|_| Pmw3610Error::Spi)?;

        Timer::after(Duration::from_micros(T_SCLK_NCS_WR_US)).await;
        let _ = self.cs.set_high();

        Timer::after(Duration::from_micros(T_SWX_US)).await;

        Ok(())
    }

    // ========================================================================
    // SPI clock control
    // ========================================================================

    async fn spi_clk_on(&mut self) -> Result<(), Pmw3610Error> {
        self.write_reg(PMW3610_SPI_CLK_ON_REQ, SPI_CLOCK_ON_REQ_ON)
            .await?;
        Timer::after(Duration::from_micros(CLOCK_ON_DELAY_US)).await;
        Ok(())
    }

    async fn spi_clk_off(&mut self) -> Result<(), Pmw3610Error> {
        self.write_reg(PMW3610_SPI_CLK_ON_REQ, SPI_CLOCK_ON_REQ_OFF)
            .await
    }

    // ========================================================================
    // Sensor configuration
    // ========================================================================

    /// Set sensor resolution in CPI (200-3200, step 200)
    pub async fn set_resolution(&mut self, cpi: u16) -> Result<(), Pmw3610Error> {
        if !(RES_MIN..=RES_MAX).contains(&cpi) {
            return Err(Pmw3610Error::InvalidCpi);
        }

        self.spi_clk_on().await?;

        self.write_reg(PWM3610_SPI_PAGE0, SPI_PAGE0_1).await?;

        let mut val = self.read_reg(PMW3610_RES_STEP).await?;
        val &= !RES_STEP_RES_MASK;
        val |= (cpi / RES_STEP) as u8;

        self.write_reg(PMW3610_RES_STEP, val).await?;
        self.write_reg(PWM3610_SPI_PAGE1, SPI_PAGE1_0).await?;

        self.spi_clk_off().await?;

        debug!("PMW3610: Resolution set to {} CPI", cpi);
        Ok(())
    }

    /// Set force awake mode
    pub async fn force_awake(&mut self, enable: bool) -> Result<(), Pmw3610Error> {
        let mut val = self.read_reg(PMW3610_PERFORMANCE).await?;
        val &= !PERFORMANCE_FMODE_MASK;
        if enable {
            val |= PERFORMANCE_FMODE_FORCE_AWAKE;
        } else {
            val |= PERFORMANCE_FMODE_NORMAL;
        }

        self.spi_clk_on().await?;
        self.write_reg(PMW3610_PERFORMANCE, val).await?;
        self.spi_clk_off().await?;

        Ok(())
    }

    // ========================================================================
    // Initialization
    // ========================================================================

    /// Configure and initialize the sensor
    async fn configure(&mut self) -> Result<(), Pmw3610Error> {
        // Power-up reset
        self.write_reg(PMW3610_POWER_UP_RESET, POWER_UP_RESET_VAL)
            .await?;
        Timer::after(Duration::from_millis(RESET_DELAY_MS)).await;

        // Verify product ID
        let val = self.read_reg(PMW3610_PROD_ID).await?;
        if val != PRODUCT_ID_PMW3610 {
            error!("Invalid product id: {:#02x}", val);
            return Err(Pmw3610Error::InvalidProductId(val));
        }
        info!("PMW3610 detected, product ID: {:#02x}", val);

        // Power-up init sequence
        self.spi_clk_on().await?;

        self.write_reg(PMW3610_OBSERVATION1, 0).await?;
        Timer::after(Duration::from_millis(INIT_OBSERVATION_DELAY_MS)).await;

        let val = self.read_reg(PMW3610_OBSERVATION1).await?;
        if (val & OBSERVATION1_INIT_MASK) != OBSERVATION1_INIT_MASK {
            error!("Unexpected OBSERVATION1 value: {:#02x}", val);
            return Err(Pmw3610Error::InitFailed);
        }

        // Read motion registers to clear them
        for reg in PMW3610_MOTION..=PMW3610_DELTA_XY_H {
            self.read_reg(reg).await?;
        }

        self.write_reg(PMW3610_PERFORMANCE, PERFORMANCE_INIT).await?;
        self.write_reg(PMW3610_RUN_DOWNSHIFT, RUN_DOWNSHIFT_INIT)
            .await?;
        self.write_reg(PMW3610_REST1_RATE, REST1_RATE_INIT).await?;
        self.write_reg(PMW3610_REST1_DOWNSHIFT, REST1_DOWNSHIFT_INIT)
            .await?;

        // Configuration: axis swap and inversion
        self.write_reg(PWM3610_SPI_PAGE0, SPI_PAGE0_1).await?;

        let mut res_step_val = self.read_reg(PMW3610_RES_STEP).await?;

        if self.config.swap_xy {
            res_step_val |= 1 << RES_STEP_SWAP_XY_BIT;
        } else {
            res_step_val &= !(1 << RES_STEP_SWAP_XY_BIT);
        }

        if self.config.invert_x {
            res_step_val |= 1 << RES_STEP_INV_X_BIT;
        } else {
            res_step_val &= !(1 << RES_STEP_INV_X_BIT);
        }

        if self.config.invert_y {
            res_step_val |= 1 << RES_STEP_INV_Y_BIT;
        } else {
            res_step_val &= !(1 << RES_STEP_INV_Y_BIT);
        }

        self.write_reg(PMW3610_RES_STEP, res_step_val).await?;
        self.write_reg(PWM3610_SPI_PAGE1, SPI_PAGE1_0).await?;

        self.spi_clk_off().await?;

        // The remaining functions call spi_clk_on/off independently.

        if self.config.res_cpi > 0 {
            self.set_resolution(self.config.res_cpi as u16).await?;
        }

        self.force_awake(self.config.force_awake).await?;

        info!("PMW3610 initialized successfully");
        Ok(())
    }

    /// Initialize the sensor (public API)
    pub async fn init(&mut self) -> Result<(), Pmw3610Error> {
        // Set initial pin states
        let _ = self.cs.set_high();
        Timer::after(Duration::from_millis(1)).await;

        self.configure().await
    }

    // ========================================================================
    // Motion reading
    // ========================================================================

    /// Read motion data from the sensor (motion work handler)
    pub async fn read_motion(&mut self) -> Result<MotionData, Pmw3610Error> {
        let burst_data_len = if self.config.smart_mode {
            BURST_DATA_LEN_SMART
        } else {
            BURST_DATA_LEN_NORMAL
        };

        let mut burst_data = [0u8; BURST_DATA_LEN_SMART];
        self.read_burst(PMW3610_BURST_READ, &mut burst_data[..burst_data_len])
            .await?;

        if (burst_data[BURST_MOTION] & MOTION_STATUS_MOTION) == 0x00 {
            return Ok(MotionData::default());
        }

        // Extract 12-bit signed motion values
        let x = ((burst_data[BURST_DELTA_XY_H] as u16) << 4) & 0xf00
            | (burst_data[BURST_DELTA_X_L] as u16);
        let y = ((burst_data[BURST_DELTA_XY_H] as u16) << 8) & 0xf00
            | (burst_data[BURST_DELTA_Y_L] as u16);

        let dx = Self::sign_extend(x, PMW3610_DATA_SIZE_BITS - 1);
        let dy = Self::sign_extend(y, PMW3610_DATA_SIZE_BITS - 1);

        // Smart mode handling
        if self.config.smart_mode {
            let shutter_val = ((burst_data[BURST_SHUTTER_HI] as u16) << 8)
                | (burst_data[BURST_SHUTTER_LO] as u16);

            if self.smart_flag && shutter_val < SHUTTER_SMART_THRESHOLD {
                self.spi_clk_on().await?;
                self.write_reg(PMW3610_SMART_MODE, SMART_MODE_ENABLE)
                    .await?;
                self.spi_clk_off().await?;
                self.smart_flag = false;
            } else if !self.smart_flag && shutter_val > SHUTTER_SMART_THRESHOLD {
                self.spi_clk_on().await?;
                self.write_reg(PMW3610_SMART_MODE, SMART_MODE_DISABLE)
                    .await?;
                self.spi_clk_off().await?;
                self.smart_flag = true;
            }
        }

        Ok(MotionData { dx, dy })
    }

    /// Sign extend a value (equivalent to Zephyr's sign_extend)
    fn sign_extend(value: u16, bits: usize) -> i16 {
        let sign_bit = 1 << bits;
        if value & sign_bit != 0 {
            (value | !((1 << (bits + 1)) - 1)) as i16
        } else {
            value as i16
        }
    }
}

// ============================================================================
// RMK InputDevice implementation (optional, enabled with "rmk" feature)
// ============================================================================

#[cfg(feature = "rmk")]
mod rmk_integration {
    use super::*;
    use rmk::event::{Axis, AxisEvent, AxisValType, Event};
    use rmk::input_device::InputDevice;

    /// Initialization state for the device
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum InitState {
        Pending,
        Initializing(u8),
        Ready,
        Failed,
    }

    /// PMW3610 as an InputDevice for RMK
    ///
    /// This device returns `Event::Joystick` events with relative X/Y movement.
    /// Use an `InputProcessor` (e.g., `JoystickProcessor` or `ScrollLayerProcessor`)
    /// to convert these events into `MouseReport` and send to the host.
    pub struct Pmw3610Device<SPI, CS, MOTION>
    where
        SPI: SpiBus,
        CS: OutputPin,
        MOTION: InputPin,
    {
        sensor: Pmw3610<SPI, CS, MOTION>,
        init_state: InitState,
        poll_interval: Duration,
    }

    impl<SPI, CS, MOTION> Pmw3610Device<SPI, CS, MOTION>
    where
        SPI: SpiBus,
        CS: OutputPin,
        MOTION: InputPin,
    {
        const MAX_INIT_RETRIES: u8 = 3;

        /// Create a new PMW3610 device for RMK
        ///
        /// # Arguments
        /// - `spi`: SPI bus (use `BitBangSpiBus` for half-duplex bit-banging without hardware support)
        /// - `cs`: Chip select pin (active low)
        /// - `motion_gpio`: Optional motion interrupt pin (active low)
        /// - `config`: Sensor configuration
        pub fn new(spi: SPI, cs: CS, motion_gpio: Option<MOTION>, config: Pmw3610Config) -> Self {
            Self {
                sensor: Pmw3610::new(spi, cs, motion_gpio, config),
                init_state: InitState::Pending,
                poll_interval: Duration::from_micros(500),
            }
        }

        async fn try_init(&mut self) -> bool {
            match self.init_state {
                InitState::Ready => return true,
                InitState::Failed => return false,
                InitState::Pending => {
                    self.init_state = InitState::Initializing(0);
                }
                InitState::Initializing(_) => {}
            }

            if let InitState::Initializing(retry_count) = self.init_state {
                info!(
                    "PMW3610: Initializing sensor (attempt {})",
                    retry_count + 1
                );

                match self.sensor.init().await {
                    Ok(()) => {
                        info!("PMW3610: Sensor initialized successfully");
                        self.init_state = InitState::Ready;
                        return true;
                    }
                    Err(e) => {
                        error!("PMW3610: Init failed: {:?}", e);
                        if retry_count + 1 >= Self::MAX_INIT_RETRIES {
                            error!("PMW3610: Max retries reached, giving up");
                            self.init_state = InitState::Failed;
                            return false;
                        }
                        self.init_state = InitState::Initializing(retry_count + 1);
                        Timer::after(Duration::from_millis(100)).await;
                        return false;
                    }
                }
            }

            false
        }
    }

    impl<SPI, CS, MOTION> InputDevice for Pmw3610Device<SPI, CS, MOTION>
    where
        SPI: SpiBus,
        CS: OutputPin,
        MOTION: InputPin,
    {
        async fn read_event(&mut self) -> Event {
            loop {
                Timer::after(self.poll_interval).await;

                if self.init_state != InitState::Ready {
                    if !self.try_init().await {
                        continue;
                    }
                }

                if !self.sensor.motion_pending() {
                    continue;
                }

                match self.sensor.read_motion().await {
                    Ok(motion) => {
                        if motion.dx != 0 || motion.dy != 0 {
                            return Event::Joystick([
                                AxisEvent {
                                    typ: AxisValType::Rel,
                                    axis: Axis::X,
                                    value: motion.dx,
                                },
                                AxisEvent {
                                    typ: AxisValType::Rel,
                                    axis: Axis::Y,
                                    value: motion.dy,
                                },
                                AxisEvent {
                                    typ: AxisValType::Rel,
                                    axis: Axis::Z,
                                    value: 0,
                                },
                            ]);
                        }
                    }
                    Err(e) => {
                        warn!("PMW3610 read error: {:?}", e);
                    }
                }
            }
        }
    }
}

#[cfg(feature = "rmk")]
pub use rmk_integration::Pmw3610Device;
