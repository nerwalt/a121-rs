#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut led = Output::new(p.P0_04, Level::Low, OutputDrive::Standard);
    
    defmt::info!("blinky");

    loop {
        led.set_high();
        defmt::info!("on");
        Timer::after_millis(300).await;
        led.set_low();
        defmt::info!("off");
        Timer::after_millis(300).await;
    }
}

