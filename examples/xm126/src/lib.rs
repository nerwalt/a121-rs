#![no_std]

use core::mem::MaybeUninit;
use embassy_nrf::bind_interrupts;
use embassy_nrf::gpio::Output;
use embassy_nrf::spim::{self, Spim};
use embassy_time::Delay;
use embedded_alloc::LlffHeap as Heap;
use embedded_hal_bus::spi::ExclusiveDevice;
use tinyrlibc as _;
use static_cell::StaticCell;

pub mod adapter;
use adapter::SpiAdapter;

//
// Heap
//
pub const HEAP_SIZE: usize = 32 * 1024;

#[global_allocator]
pub static HEAP: Heap = Heap::empty();

pub fn init_heap() {
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    #[allow(static_mut_refs)]
    unsafe {
        HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE)
    }
}


//
// Hardware peripherals on the XM126
//
pub mod peripherals {
    use embassy_nrf::peripherals;

    pub type A121Enable = peripherals::P0_15;
    pub type A121Interrupt = peripherals::P0_08;

    pub type Spi = peripherals::TWISPI0;
    pub type SpiSck = peripherals::P0_27;
    pub type SpiMiso = peripherals::P0_05;
    pub type SpiMos = peripherals::P1_08;

    // The SPI chip/slave-select (CS/SS) on the XM126 is tied to ground to make it the single,
    // exclusive SPI slave device on the bus. We still need to provide a pin to the driver, so pick
    // any unused pin for the CS pin.
    pub type SpiCs = peripherals::P1_07; 
}

//
// SPI 
//

bind_interrupts!(pub struct Irqs {
    TWISPI0 => spim::InterruptHandler<peripherals::Spi>;
});

type RawSpi = Spim<'static>;
type ExclusiveRawSpi = ExclusiveDevice<RawSpi, Output<'static>, Delay>;
pub type RadarSpi = SpiAdapter<ExclusiveRawSpi>;

pub static RADAR_SPI: StaticCell<RadarSpi> = StaticCell::new();

