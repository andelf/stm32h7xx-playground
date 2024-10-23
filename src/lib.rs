#![no_std]

use embassy_stm32 as hal;
use hal::{pac, peripherals};

pub unsafe fn configure_qspi_flash() {
    use hal::qspi::enums::*;
    use hal::qspi::*;
    let config = hal::qspi::Config {
        memory_size: MemorySize::_8MiB,
        address_size: AddressSize::_24bit,
        prescaler: 1,
        cs_high_time: ChipSelectHighTime::_1Cycle,
        fifo_threshold: FIFOThresholdLevel::_32Bytes,
    };
    /*
                d0: impl Peripheral<P = impl BK1D0Pin<T>> + 'd,
            d1: impl Peripheral<P = impl BK1D1Pin<T>> + 'd,
            d2: impl Peripheral<P = impl BK1D2Pin<T>> + 'd,
            d3: impl Peripheral<P = impl BK1D3Pin<T>> + 'd,
            sck: impl Peripheral<P = impl SckPin<T>> + 'd,
            nss: impl Peripheral<P = impl BK1NSSPin<T>> + 'd,
            dma: impl Peripheral<P = impl QuadDma<T>> + 'd,
            - IO3: PD13
    - IO2: PE2
    - IO1: PD12
    - IO0: PD11
    - CLK: PB2
    - CS: PB6
             */

    unsafe {
        let quadspi = peripherals::QUADSPI::steal();
        let pd11 = peripherals::PD11::steal();
        let pd12 = peripherals::PD12::steal();
        let pe2 = peripherals::PE2::steal();
        let pd13 = peripherals::PD13::steal();
        let pb2 = peripherals::PB2::steal();
        let pb6 = peripherals::PB6::steal();

        let driver = Qspi::new_blocking_bank1(quadspi, pd11, pd12, pe2, pd13, pb2, pb6, config);
        core::mem::forget(driver);
    }

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

        w.set_dcyc(4);
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
    defmt::info!("QSPI ready");
}
