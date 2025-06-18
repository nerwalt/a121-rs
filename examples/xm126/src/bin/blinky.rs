#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // The LED is active low, so initialize the output high to start with the LED off
    let mut led = Output::new(p.P0_04, Level::High, OutputDrive::Standard);
    
    defmt::info!("blinky");

    loop {
        defmt::info!("blink");
        led.set_low();
        Timer::after_millis(100).await;
        led.set_high();
        Timer::after_millis(100).await;

        Timer::after_secs(1).await;
    }
}

