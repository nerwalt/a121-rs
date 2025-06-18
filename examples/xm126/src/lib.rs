#![no_std]

extern crate alloc;

use core::cell::RefCell;

use embassy_nrf::gpio::Output;
use embassy_nrf::peripherals;
use embassy_nrf::spim::Spim;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use talc::{ClaimOnOom, Span, Talc, Talck};
use tinyrlibc as _;
use {defmt_rtt as _, panic_probe as _};

use crate::adapter::SpiAdapter;

pub mod adapter;

static mut ARENA: [u8; 16384] = [0u8; 16384];

#[global_allocator]
static ALLOCATOR: Talck<spin::Mutex<()>, ClaimOnOom> = Talc::new(unsafe {
    // if we're in a hosted environment, the Rust runtime may allocate before
    // main() is called, so we need to initialize the arena automatically
    ClaimOnOom::new(Span::from_const_array(core::ptr::addr_of!(ARENA)))
})
.lock();

// bind_interrupts!(struct Irqs {
//     SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => twim::InterruptHandler<peripherals::TWISPI0>;
// });

// The chip select line on the A121 on the XM126 is grounded and thus no used. Here, we give
// ExclusiveDevice an un-used pin to make it happy.
pub type SpiDeviceMutex =
    ExclusiveDevice<Spim<'static, peripherals::TWISPI0>, Output<'static, peripherals::P1_07>, Delay>;

pub static mut SPI_DEVICE: Option<RefCell<SpiAdapter<SpiDeviceMutex>>> = None;
