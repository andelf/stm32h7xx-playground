#![no_std]
#![no_main]
#![allow(dead_code)] // Allow dead code as not all commands are used in the example

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::qspi::enums::{AddressSize, ChipSelectHighTime, FIFOThresholdLevel, MemorySize, *};
use embassy_stm32::qspi::{self, Config as QspiCfg, Instance, Qspi, TransferConfig};
use embassy_stm32::{pac, Config as StmCfg};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());
    info!("Embassy initialized");

    let config = QspiCfg {
        memory_size: MemorySize::_8MiB,
        address_size: AddressSize::_24bit,
        prescaler: 2,
        cs_high_time: ChipSelectHighTime::_1Cycle,
        fifo_threshold: FIFOThresholdLevel::_32Bytes,
    };
    /*
    - IO3: PD13
    - IO2: PE2
    - IO1: PD12
    - IO0: PD11
    - CLK: PB2
    - CS: PB6
             */
    let driver = Qspi::new_blocking_bank1(p.QUADSPI, p.PD11, p.PD12, p.PE2, p.PD13, p.PB2, p.PB6, config);
    //let flash = FlashMemory::new(driver);
    // core::mem::forget(flash);

    let qspi = pac::QUADSPI;

    qspi.cr().modify(|w| {
        w.set_en(false);
        w.set_tcen(false);
        w.set_dmaen(false);
    });

    while qspi.sr().read().busy() {}

    qspi.ccr().modify(|w| {
        w.set_fmode(0b11); // 0b11 = memory-mapped mode

        w.set_instruction(0xEB); // W25Qxx_CMD_FastReadQuad_IO 0xEB for 1-4-4 mode
                                 // 0x32  	// 1-1-4 mode

        w.set_abmode(0b11); // 4 lines
        w.set_absize(0); // 8bytes

        w.set_dcyc(6);
        w.set_sioo(true);

        w.set_adsize(0b10); // 24-bit address

        // QspiWidth
        w.set_imode(0x01); // SING
        w.set_admode(0b11); // QUAD
        w.set_dmode(0b11); // QUAD
    });
    qspi.abr().modify(|w| w.set_alternate(0xEF));
    qspi.ar().write(|w| w.set_address(0x00));

    qspi.cr().modify(|w| w.set_en(true));

    while qspi.sr().read().busy() {}

    defmt::info!("Start reading memory");

    let mut led = Output::new(p.PE3, Level::High, Speed::Low);

    loop {
        defmt::println!("FOO func address: 0x{:08x}", foo as *const () as u32);

        foo();

        led.toggle();

        Timer::after_millis(1000).await;
    }
}

#[link_section = ".spiflash"]
fn foo() {
    defmt::info!("in function foo");
}
