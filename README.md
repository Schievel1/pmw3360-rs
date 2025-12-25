# pmw3360-rs

PixArt PMW3360 Mouse Sensor Driver for Rust/Embassy/RMK.

Ported from the Zephyr driver implementation:
https://github.com/zephyrproject-rtos/zephyr/blob/d31c6e95033fd6b3763389edba6a655245ae1328/drivers/input/input_pmw3610.c

## Usage with RMK

### 1. Add to your Cargo.toml dependencies

```toml
[dependencies]
pmw3360-rs = { git = "https://github.com/Schievel1/pmw3360-rs", branch = "main", features = ["rmk"] }
```

#### Features

- `rmk`: Enables `Pmw3360Device` with `InputDevice` trait for RMK.

### 2. Initialize the sensor

```rust
use embassy_rp::gpio::{Input, Output, Level, Pull}; // rp2040 hal as example
use embassy_rp::spi::{Spi, Config, Polarity, Phase};
use pmw3360_rs::{Pmw3360Config, Pmw3360Device};

let p = embassy_rp::init(Default::default());

let mut spi_cfg = Config::default();
// MODE_3 = Polarity::IdleHigh + Phase::CaptureOnSecondTransition
spi_cfg.polarity = Polarity::IdleHigh;
spi_cfg.phase = Phase::CaptureOnSecondTransition;
spi_cfg.frequency = 2_000_000;

 // Create GPIO pins
 let pmw3360_sck = p.PIN_18;
 let pmw3360_mosi = p.PIN_19;
 let pmw3360_miso = p.PIN_16;
 let pmw3360_cs = Output::new(p.PIN_17, Level::High);
 let pmw3360_irq = Input::new(p.PIN_20, Pull::Up);

// Create the SPI bus
let pmw3360_spi = Spi::new(p.SPI0, pmw3360_sck,pmw3360_mosi,pmw3360_miso, p.DMA_CH2, p.DMA_CH3, spi_cfg); // SPI0, DMA_CH2, DMA_CH3 used in this example

// Initialize PMW3360 mouse sensor
let pmw3360_config = Pmw3360Config {
    res_cpi: 1600,
    rot_trans_angle: -15,
    liftoff_dist: 0x08,
    swap_xy: false,
    invert_x: true,
    invert_y: false,
    ..Default::default()
};

 // Create the sensor device
 let mut pmw3360_device = Pmw3360Device::new(
     pmw3360_spi, pmw3360_cs, Some(pmw3360_irq), pmw3360_config
 );

// Add to the run_devices! macro
run_devices! (
    (matrix, pmw3310_device) => EVENT_CHANNEL,
),
```

### 3. Add an InputProcessor to handle the events

`Pmw3360Device` returns `Event::Joystick` events. RMK provides a `Pmw3610Processor` to convert the joystick events to `MouseReport`.

```rust
use rmk::input_device::pmw3610::Pmw3610Processor;

let mut pmw3360_processor = Pmw3610Processor::new(&keymap);

// Add processor to the chain
    run_processor_chain! {
        EVENT_CHANNEL => [pmw3360_processor],
    },
```

# License

Apache-2.0 (derived from Zephyr driver)
