#![no_std]
#![no_main]

use assign_resources::assign_resources;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};
use embedded_graphics::framebuffer::Framebuffer;
use embedded_graphics::mono_font::ascii::FONT_7X13;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::raw::BigEndian;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::{Point, RgbColor, *};
use embedded_graphics::text::Text;
use {defmt_rtt as _, embassy_stm32 as hal, panic_probe as _};

assign_resources! {
    leds: Led {
        blue: PE3
    },
    tftlcd: TftLcdResource {
        spi: SPI4,
        cs: PE11,
        dc: PE13,
        mosi: PE14,
        sck: PE12,
        bl: PE10,
        dma_ch: DMA1_CH2
    }
}

#[embassy_executor::task]
async fn lcd_task(r: TftLcdResource) {
    let mut config = hal::spi::Config::default();
    config.frequency = Hertz::mhz(30);
    let spi = hal::spi::Spi::new_txonly(r.spi, r.sck, r.mosi, r.dma_ch, config);
    let cs = Output::new(r.cs, Level::High, Speed::Low);

    //let spi_bus = AtomicCell::new(spi);
    //let mut spi_dev = embedded_hal_bus::spi::AtomicDevice::new(&spi_bus, cs, Delay).unwrap();

    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(spi);
    let spi_dev = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi_bus, cs);

    let dc = Output::new(r.dc, Level::High, Speed::Low);

    let mut bl = Output::new(r.bl, Level::High, Speed::Low);
    bl.set_low();

    let mut display: edrv_st7735::ST7735<edrv_st7735::Display160x80Type2, _, _> = edrv_st7735::ST7735::new(spi_dev, dc);
    display.init(&mut Delay).await.unwrap();

    display.clear(Rgb565::WHITE).await.unwrap();

    let mut fb = Framebuffer::<
        Rgb565,
        _,
        BigEndian,
        160,
        80,
        { embedded_graphics::framebuffer::buffer_size::<Rgb565>(160, 80) },
    >::new();

    let character_style = MonoTextStyleBuilder::new()
        //.background_color(BinaryColor::Off)
        .text_color(Rgb565::RED)
        .font(&FONT_7X13)
        .build();

    loop {
        Text::new("Hello Ruast!", Point::new(10, 30), character_style)
            .draw(&mut fb)
            .unwrap();

        display.write_framebuffer(fb.data()).await.unwrap();

        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let r = split_resources!(p);

    let mut led = Output::new(r.leds.blue, Level::High, Speed::Low);

    spawner.must_spawn(lcd_task(r.tftlcd));

    loop {
        info!("high");
        led.set_high();
        Timer::after_millis(500).await;

        info!("low");
        led.set_low();
        Timer::after_millis(500).await;
    }
}
