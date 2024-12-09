//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_graphics::{
    mono_font::{
        ascii::{FONT_6X9, FONT_7X14, FONT_8X13, FONT_9X18},
        MonoTextStyle,
    },
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use embedded_hal::digital::OutputPin;
use export::display;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{
    self as bsp,
    hal::gpio::{FunctionI2C, Pin},
};
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use ssd1306::{
    mode::DisplayConfig, prelude::DisplayRotation, size::DisplaySize128x64, I2CDisplayInterface,
    Ssd1306,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // SPI interface pins
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio2.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio3.reconfigure();

    // Initializing display interface, display & text style etc.
    let i2c = rp_pico::hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        1.MHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let mut led_pin = pins.led.into_push_pull_output();

    let text_style = MonoTextStyle::new(&FONT_9X18, BinaryColor::On);

    let hello_world = Text::new(
        "Hello World!\nHello World!\nHello World!",
        Point::new(0, 10),
        text_style,
    );
    let hey_jude = Text::new("Hey Jude!\nNa Na na na", Point::new(0, 10), text_style);

    loop {
        info!("on!");
        let _ = display.clear(BinaryColor::Off);
        led_pin.set_high().unwrap();
        let _ = hello_world.draw(&mut display);
        let _ = display.flush();
        delay.delay_ms(1000);

        let _ = display.clear(BinaryColor::Off);
        info!("off!");
        led_pin.set_low().unwrap();
        let _ = hey_jude.draw(&mut display);
        let _ = display.flush();
        delay.delay_ms(1000);
    }
}

// End of file
