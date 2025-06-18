#![no_std]
#![no_main]
#![allow(unused)]

extern crate alloc;

use alloc::vec;
use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Input, Output, Pull, OutputDrive};
use embassy_nrf::spim;
use embassy_time::{Delay, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embassy_nrf::{peripherals, bind_interrupts};
use embassy_futures::select::{select, Either, select3, Either3};

use defmt::{panic, debug, info, trace, warn};
use {defmt_rtt as _, panic_probe as _};

use a121_rs::detector::distance::config::{
    RadarDistanceConfig,
    MaxStepLenght,
    PeakSortingMethod,
    ThresholdMethod,
    ReflectorShape,
};
use a121_rs::config::frame_rate::FrameRate;
use a121_rs::config::profile::RadarProfile;
use a121_rs::detector::distance::RadarDistanceDetector;
use a121_rs::radar::{self, Radar, version::rss_version};

use xm126::adapter::SpiAdapter;
use xm126::*;

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => spim::InterruptHandler<peripherals::TWISPI0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let enable = Output::new(p.P0_15, Level::Low, OutputDrive::Standard); 
    let cs_pin = Output::new(p.P1_07, Level::Low, OutputDrive::Standard);
    let interrupt = Input::new(p.P0_08, Pull::Up);
    info!("GPIO initialized.");

    let spi = spim::Spim::new(
        p.TWISPI0,
        Irqs,
        p.P0_27, // SCK
        p.P0_05, // MISO
        p.P1_08, // MOSI
        spim::Config::default(),
    );
    let exclusive_device = ExclusiveDevice::new(spi, cs_pin, Delay);

    unsafe { SPI_DEVICE = Some(RefCell::new(SpiAdapter::new(exclusive_device))) };
    let spi_mut_ref = unsafe { SPI_DEVICE.as_mut().unwrap() };

    debug!("RSS Version: {}", rss_version());

    info!("Enabling radar.");
    let mut radar = Radar::new(1, spi_mut_ref.get_mut(), interrupt, enable, Delay).await;
    info!("Radar enabled.");

    info!("Calibrating.");
    let mut calibration = radar.calibrate().await.unwrap();
    info!("Calibration complete.");

    let mut foo = radar.prepare_sensor(&mut calibration).unwrap();
    foo.config.set_frame_rate(FrameRate::Limited(1.0));

    info!("Frame Rate: {:?}", foo.config.frame_rate().value());

    let mut dist_config = RadarDistanceConfig::balanced();
    dist_config.set_interval(0.5..=3.0);
    // dist_config.set_close_range_leakage_cancelation(true);
    // dist_config.set_max_step_length(MaxStepLenght::Manual(24));
    // dist_config.set_max_profile(RadarProfile::AccProfile5);
    // dist_config.set_reflector_shape(ReflectorShape::Planar);
    // dist_config.set_peak_sorting_method(PeakSortingMethod::Strength);
    // dist_config.set_threshold_method(ThresholdMethod::Cfar);
    // dist_config.set_threshold_sensitivity(0.0);
    // dist_config.set_signal_quality(-10.0);

    let mut distance = RadarDistanceDetector::with_config(&mut foo, dist_config);
    let mut buffer = vec![0u8; distance.get_distance_buffer_size()];
    let mut static_cal_result = vec![0u8; distance.get_static_result_buffer_size()];

    // trace!("Calibrating detector...");
    // let mut dynamic_cal_result = distance
    //     .calibrate_detector(&calibration, &mut buffer, &mut static_cal_result)
    //     .await
    //     .unwrap();

    // // let mut dynamic_cal_result = match select(
    // //     distance.calibrate_detector(&calibration, &mut buffer, &mut static_cal_result),
    // //     Timer::after_secs(120),
    // // ).await {
    // //     Either::First(result) => result.unwrap(),
    // //     Either::Second(_) => panic!("TImeout calibrating detector"),
    // // };

    // loop {
    //     distance
    //         .prepare_detector(&calibration, &mut buffer)
    //         .unwrap();
    //     distance.measure(&mut buffer).await.unwrap();

    //     match distance.process_data(&mut buffer, &mut static_cal_result, &mut dynamic_cal_result) {
    //         Ok(res) => {
    //             if res.num_distances() > 0 {
    //                 info!("Distances: {} {:?}", res.num_distances(), res.distances());
    //             }
    //             if res.calibration_needed() {
    //                 info!("Calibration needed.");
    //                 let calibration = distance.calibrate().await.unwrap();
    //                 dynamic_cal_result = distance
    //                     .update_calibration(&calibration, &mut buffer)
    //                     .await
    //                     .unwrap();
    //             }
    //         }
    //         Err(_) => {}//warn!("Failed to process data."),
    //     }

    //     Timer::after_millis(3000).await;
    // }
}
