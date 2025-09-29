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
use embassy_futures::select::{select, Either, select3, Either3};

use defmt::{panic, error, debug, info, trace, warn};
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
use xm126::{init_heap, Irqs, peripherals, RADAR_SPI};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    init_heap();

    let p = embassy_nrf::init(Default::default());

    debug!("Acconeer RSS Version: {}", rss_version());

    info!("Initialize SPI");
    let sck = p.P0_27;
    let miso = p.P0_05;
    let mosi = p.P1_08;
    let spi = spim::Spim::new(p.TWISPI0, Irqs, sck, miso, mosi, spim::Config::default());
    // Pick any unsed pin for CS (SS) as the chip (slave) select line on the xm126 is tied to
    // ground, which makes the a121 an hard-wired exclusive device. The CS pin is thus needed only
    // to make the driver happy.
    let cs = Output::new(p.P1_07, Level::High, OutputDrive::Standard);
    let spi = ExclusiveDevice::new(spi, cs, Delay).unwrap();
    let spi = SpiAdapter::new(spi);
    let spi = RADAR_SPI.init(spi);

    info!("Initialize radar");
    let enable = Output::new(p.P0_15, Level::Low, OutputDrive::Standard); 
    let interrupt = Input::new(p.P0_08, Pull::Up);
    let mut radar = match Radar::new(1, spi, interrupt, enable, Delay).await {
        Ok(radar) => {
            info!("Radar enabled.");
            radar
        },
        Err(err) => {
            error!("Error Instantiating the radar driver: {:?}", err);
            return;
        }
    };

    // Check radar connectivity before calibration
    if !radar.is_connected() {
        defmt::error!("Radar is not connected or not responding");
        return;
    }
    info!("Radar connected");

    // Check radar status. Useful for debugging
    // radar.check_status();

    // Calibrate before use
    info!("Calibrating radar");
    let mut calibration = loop {
        match radar.calibrate().await {
            Ok(calibration) => match calibration.validate_calibration() {
                Ok(()) => {
                    info!("Calibration complete and validated.");
                    break calibration;
                }
                Err(_) => {
                    warn!("Calibration invalid, retrying...");
                    Timer::after_millis(100).await;
                }
            },
            Err(e) => {
                warn!("Calibration failed: {:?}, retrying...", e);
                Timer::after_millis(100).await;
            }
        }
    };

    info!("Calibration complete.");

    info!("Preparing radar");
    if let Err(err) = radar.prepare_sensor(&mut calibration) {
        error!("Error preparing radar: {:?}", err);
    }

    let mut dist_config = RadarDistanceConfig::balanced();
    // If the start interval is <= 6 cm, then close_range_leakage_cancellation needs to be enabled.
    // If this is the case, a known calibraiton environment is required. Avoid this unless strictly
    // needed.
    dist_config.set_interval(0.25..=3.0);
    // dist_config.set_start_interval(0.10);
    // dist_config.set_end_interval(2.0);
    dist_config.set_close_range_leakage_cancelation(false);
    // The step length and profile are both automatically selected to yield a good trade-off
    // between SNR and power consumption. The SNR can be improved by reducing step length through
    // the parameter max_step_length , with a penalty on power consumption. The radial resolution
    // can be increased by limiting the max profile used through the parameter max_profile , with a
    // penalty on SNR
    dist_config.set_max_step_length(MaxStepLenght::ProfileBased);
    dist_config.set_max_profile(RadarProfile::AccProfile5);
    // The reflector_shape should be set to PLANAR when measuring a planar surface. In all other
    // cases, it should be set to GENERIC .
    dist_config.set_reflector_shape(ReflectorShape::Generic);
    // // Peak sorting determines the sort order of the detected objects. Whether to use CLOSEST
    // (Amplitude) or STRONGEST (Strength) depends on the use case.  Note, regardless of the
    // selected peak sorting method, all detected distances are returned by the application.
    dist_config.set_peak_sorting_method(PeakSortingMethod::Strength);
    // There are four threshold methods available. Which one to use is use case dependent. More
    // information can be found under the section Thresholds.
    // – CFAR - Suitable when the use case involve clear peaks such as a level measurement
    // application. The method is robust over temperatures and does not required any consideration
    // to the surroundings when calibrating.
    // – FIXED_STRENGTH - Applies a threshold to the estimated strengths. This threshold is
    // suitable when estimating the distance to a strong reflector in a cluttered environment.
    // – FIXED - Applies a threshold to the sweep amplitude. This threshold detects objects based
    // on their measured amplitudes.  Note, for a given object, the amplitude reduce with distance
    // as less energy is reflected back to the sensor, resulting in missed detections.
    // – RECORDED - This threshold records the background clutter and is thereafter applied to the sweep as a
    // threshold.  The threshold is suitable when the environment consists of a several reflecting
    // objects that should not be detected (clutter).  The threshold has a built in temperature
    // compensation, based internal temperature sensor, adjusting the threshold to keep a constant
    // false positive rate.  Note, the threshold is only valid as long as the background is static.
    // A change in the clutter can result in undesired objects being detected.
    //
    // Generally, the CFAR or RECORDED are preferred when the ambient temperature is expected to
    // change. The FIXED_STRENGTH and FIXED are fixed and has no temperature compensations built
    // in.
    dist_config.set_threshold_method(ThresholdMethod::Cfar);
    // Threshold_sensitivity controls the false positive rate for the CFAR and recorded threshold.
    // The parameters should be tuned for each use case to achieve the desirable performance.
    dist_config.set_threshold_sensitivity(0.5);
    // Signal_quality should be set so that desirable detection rate is achieved. A higher value
    // corresponds to higher HWAAS and SNR, but also higher power consumption.
    dist_config.set_signal_quality(15.0);

    // Initialize the distance detector
    let mut distance = match RadarDistanceDetector::with_config(&mut radar, dist_config) {
        Ok(distance) => distance,
        Err(err) => {
            error!("Error creating distance detector: {:?}", err);
            return
        }
    };

    // Initialize results and calibration buffers
    let mut buffer = vec![0u8; distance.get_distance_buffer_size()];
    let mut static_cal_result = vec![0u8; distance.get_static_result_buffer_size()];

    trace!("Calibrating detector...");
    let mut dynamic_cal_result = match distance.calibrate_detector(&calibration, &mut buffer, &mut static_cal_result).await {
        Ok(res) => res,
        Err(err) => {
            error!("Error calibrating distance detector: {:?}", err);
            return
        }
    };

    // 
    // Main sampling loop
    //
    loop {
        // Prepare the detector for a measurement. This must be called before measure()
        if let Err(err) = distance.prepare_detector(&calibration, &mut buffer) {
            error!("Error preparing measurement: {:?}", err);
            return
        }

        // Trigger a measurement... waits for the interrupt pin to go high, signaling a measurement
        // has been captured into the resultsbuffer
        if let Err(err) = distance.measure(&mut buffer).await {
            error!("Error getting measurement: {:?}", err);
            return
        }

        match distance.process_data(&mut buffer, &mut static_cal_result, &mut dynamic_cal_result) {
            Ok(res) => {
                if res.num_distances() > 0 {
                    info!("{} Distances found:\n{:?}", res.num_distances(), res.distances());
                }
                if res.calibration_needed() {
                    info!("Calibration needed.");
                    match distance.calibrate().await {
                        Ok(cal) => {
                            match distance.update_calibration(&cal, &mut buffer).await {
                                Ok(res) => {
                                    dynamic_cal_result = res;
                                }
                                Err(err) => {
                                    warn!("Error updating calibration result: {:?}", err);
                                    warn!("Continuing ...");
                                }
                            };
                        }
                        Err(err) => {
                            warn!("Error re-calibrating radar: {:?}", err);
                            warn!("Continuing ...");
                        }
                    };
                }
            }
            Err(_) => warn!("Failed to process data."),
        }

        // Timer::after_secs(1).await;
    }
}
