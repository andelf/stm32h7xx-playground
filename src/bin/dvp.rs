#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::fmt::Write as _;

use assign_resources::assign_resources;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dcmi::{Dcmi, HSyncDataInvalidLevel, PixelClockPolarity, VSyncDataInvalidLevel};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::rcc::Mco;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};
use embedded_graphics::framebuffer::Framebuffer;
use embedded_graphics::image::ImageRaw;
use embedded_graphics::mono_font::ascii::FONT_7X13;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::raw::{BigEndian, LittleEndian};
use embedded_graphics::pixelcolor::{Bgr555, Rgb565};
use embedded_graphics::prelude::{Point, RgbColor, *};
use embedded_graphics::text::Text;
use hal::{dcmi, i2c};
use ov7725::Ov7725;
use {defmt_rtt as _, embassy_stm32 as hal, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    DCMI => dcmi::InterruptHandler<peripherals::DCMI>;
});

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
        dma_ch: DMA1_CH3,
    }
    /*
    DCMI DO -> PC6
DCMI D1 -> PC7
DCMI D2 ->PEO
DCMI D3 > PE1
DCMI D4 -> PE4
DCMI D5 -> PD3
DCMI D6 -> PE5
DCMI D7 -> PE6
DCMI VSYNC -> PB7
DCMI HSYNC ->PA4
DCMI PIXCLK ->PA6
RCC MCO 1 ->PA8
I2C1 SCL-> PB8
I2C1 SDA -> PB9
 */
    ov7725: CameraResource {
        dcmi: DCMI,
        d0: PC6,
        d1: PC7,
        d2: PE0,
        d3: PE1,
        d4: PE4,
        d5: PD3,
        d6: PE5,
        d7: PE6,
        vsync: PB7,
        hsync: PA4,
        pixclk: PA6,
        dma_ch: DMA1_CH0,
        i2c1: I2C1,
        i2c1_tx_dma: DMA1_CH1,
        i2c1_rx_dma: DMA1_CH2,
        scl: PB8,
        sda: PB9,
        mco1: MCO1,
        mco_pin: PA8
    }
}

const WIDTH: usize = 160;
const HEIGHT: usize = 120;

static mut FRAME: [u32; WIDTH * HEIGHT / 2] = [0u32; WIDTH * HEIGHT / 2];

#[embassy_executor::task]
async fn camera_task(r: CameraResource) {
    use hal::rcc::{Mco, Mco1Source, McoPrescaler};
    let mco = Mco::new(r.mco1, r.mco_pin, Mco1Source::HSI, McoPrescaler::DIV2);

    let cam_i2c = I2c::new(
        r.i2c1,
        r.scl,
        r.sda,
        Irqs,
        r.i2c1_tx_dma,
        r.i2c1_rx_dma,
        Hertz::khz(100),
        Default::default(),
    );

    let mut camera = Ov7725::new(cam_i2c, mco);

    defmt::unwrap!(camera.init().await);

    let manufacturer_id = defmt::unwrap!(camera.read_manufacturer_id().await);
    let camera_id = defmt::unwrap!(camera.read_product_id().await);

    //  manufacturer: 0x7fa2, pid: 0x7721
    defmt::info!("manufacturer: 0x{:x}, pid: 0x{:x}", manufacturer_id, camera_id);

    let mut config = dcmi::Config::default();
    config.pixclk_polarity = PixelClockPolarity::FallingEdge;
    // config.hsync_level = HSyncDataInvalidLevel::High;
    //config.vsync_level = VSyncDataInvalidLevel::Low;
    let mut dcmi = Dcmi::new_8bit(
        r.dcmi, r.dma_ch, Irqs, r.d0, r.d1, r.d2, r.d3, r.d4, r.d5, r.d6, r.d7, r.vsync, r.hsync, r.pixclk, config,
    );

    defmt::info!("attempting capture");
    defmt::unwrap!(dcmi.capture(unsafe { &mut *core::ptr::addr_of_mut!(FRAME) }).await);

    defmt::info!("captured frame: {:x}", unsafe { &*core::ptr::addr_of!(FRAME) });

    loop {
        defmt::unwrap!(dcmi.capture(unsafe { &mut *core::ptr::addr_of_mut!(FRAME) }).await);
        defmt::info!("captured frame");
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

    let mut buf = heapless::String::<128>::new();

    let mut i = 0;
    loop {
        buf.clear();
        fb.clear(Rgb565::BLACK).unwrap();

        core::write!(buf, "Hello Ruast! {}", i).unwrap();
        i += 1;

        Text::new(buf.as_str(), Point::new(10, 30), character_style)
            .draw(&mut fb)
            .unwrap();

        // display.write_framebuffer(fb.data()).await.unwrap();

        let raw = unsafe { &*core::ptr::addr_of!(FRAME) };
        let snap = *raw;

        println!("fuck {:08x}", &raw[..20]);
        println!("fuck {:08x}", &raw[300..310]);

        let buf = unsafe { core::slice::from_raw_parts(snap.as_ptr() as *const u8, WIDTH * HEIGHT * 2) };

        //let snap = *raw;

        let label_img1: ImageRaw<Rgb565, BigEndian> = ImageRaw::new(buf, 160);

        label_img1.draw(&mut fb).unwrap();

        display.write_framebuffer(fb.data()).await.unwrap();

        //        display.write_framebuffer(buf).await.unwrap();

        Timer::after_millis(100).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // let p = embassy_stm32::init(Default::default());
    let mut config = hal::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8), // 100mhz
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }
    let p = embassy_stm32::init(config);
    info!("Hello World!");

    let r = split_resources!(p);

    let mut led = Output::new(r.leds.blue, Level::High, Speed::Low);

    spawner.must_spawn(lcd_task(r.tftlcd));

    spawner.must_spawn(camera_task(r.ov7725));

    loop {
        led.set_high();
        Timer::after_millis(500).await;

        led.set_low();
        Timer::after_millis(500).await;
    }
}

mod ov7725 {
    use core::marker::PhantomData;

    use defmt::Format;
    use embassy_stm32::rcc::{Mco, McoInstance};
    use embassy_time::Timer;
    use embedded_hal_async::i2c::I2c;

    #[repr(u8)]
    pub enum RgbFormat {
        Gbr422 = 0,
        RGB565 = 1,
        RGB555 = 2,
        RGB444 = 3,
    }
    pub enum PixelFormat {
        Yuv,
        ProcessedRawBayer,
        Rgb(RgbFormat),
        RawBayer,
    }

    impl From<PixelFormat> for u8 {
        fn from(raw: PixelFormat) -> Self {
            match raw {
                PixelFormat::Yuv => 0,
                PixelFormat::ProcessedRawBayer => 1,
                PixelFormat::Rgb(mode) => 2 | ((mode as u8) << 2),
                PixelFormat::RawBayer => 3,
            }
        }
    }

    #[derive(Clone, Copy)]
    #[repr(u8)]
    #[allow(unused)]
    pub enum Register {
        Gain = 0x00,
        Blue = 0x01,
        Red = 0x02,
        Green = 0x03,
        BAvg = 0x05,
        GAvg = 0x06,
        RAvg = 0x07,
        Aech = 0x08,
        Com2 = 0x09,
        PId = 0x0a,
        Ver = 0x0b,
        Com3 = 0x0c,
        Com4 = 0x0d,
        Com5 = 0x0e,
        Com6 = 0x0f,
        Aec = 0x10,
        ClkRc = 0x11,
        Com7 = 0x12,
        Com8 = 0x13,
        Com9 = 0x14,
        Com10 = 0x15,
        Reg16 = 0x16,
        HStart = 0x17,
        HSize = 0x18,
        VStart = 0x19,
        VSize = 0x1a,
        PShift = 0x1b,
        MidH = 0x1c,
        MidL = 0x1d,
        Laec = 0x1f,
        Com11 = 0x20,
        BdBase = 0x22,
        BdMStep = 0x23,
        Aew = 0x24,
        Aeb = 0x25,
        Vpt = 0x26,
        Reg28 = 0x28,
        HOutSize = 0x29,
        EXHCH = 0x2a,
        EXHCL = 0x2b,
        VOutSize = 0x2c,
        Advfl = 0x2d,
        Advfh = 0x2e,
        Yave = 0x2f,
        LumHTh = 0x30,
        LumLTh = 0x31,
        HRef = 0x32,
        DspCtrl4 = 0x67,
        DspAuto = 0xac,
    }

    const CAM_ADDR: u8 = 0x21;

    #[derive(Format, PartialEq, Eq)]
    pub enum Error<I2cError: Format> {
        I2c(I2cError),
    }

    pub struct Ov7725<'d, Bus: I2c> {
        phantom: PhantomData<&'d ()>,
        bus: Bus,
    }

    impl<'d, Bus> Ov7725<'d, Bus>
    where
        Bus: I2c,
        Bus::Error: Format,
    {
        pub fn new<T>(bus: Bus, _mco: Mco<T>) -> Self
        where
            T: McoInstance,
        {
            Self {
                phantom: PhantomData,
                bus,
            }
        }

        pub async fn init(&mut self) -> Result<(), Error<Bus::Error>> {
            Timer::after_millis(500).await;
            self.reset_regs().await?;
            Timer::after_millis(500).await;
            self.set_pixformat().await?;
            self.set_resolution().await?;
            Ok(())
        }

        pub async fn read_manufacturer_id(&mut self) -> Result<u16, Error<Bus::Error>> {
            Ok(u16::from_le_bytes([
                self.read(Register::MidL).await?,
                self.read(Register::MidH).await?,
            ]))
        }

        pub async fn read_product_id(&mut self) -> Result<u16, Error<Bus::Error>> {
            Ok(u16::from_le_bytes([
                self.read(Register::Ver).await?,
                self.read(Register::PId).await?,
            ]))
        }

        async fn reset_regs(&mut self) -> Result<(), Error<Bus::Error>> {
            self.write(Register::Com7, 0x80).await
        }

        async fn set_pixformat(&mut self) -> Result<(), Error<Bus::Error>> {
            self.write(Register::DspCtrl4, 0).await?;
            let mut com7 = self.read(Register::Com7).await?;
            com7 |= u8::from(PixelFormat::Rgb(RgbFormat::RGB565));
            self.write(Register::Com7, com7).await?;
            Ok(())
        }

        async fn set_resolution(&mut self) -> Result<(), Error<Bus::Error>> {
            let horizontal: u16 = super::WIDTH as u16;
            let vertical: u16 = super::HEIGHT as u16;

            let h_high = (horizontal >> 2) as u8;
            let v_high = (vertical >> 1) as u8;
            let h_low = (horizontal & 0x03) as u8;
            let v_low = (vertical & 0x01) as u8;

            self.write(Register::HOutSize, h_high).await?;
            self.write(Register::VOutSize, v_high).await?;
            self.write(Register::EXHCH, h_low | (v_low << 2)).await?;

            self.write(Register::Com3, 0xd1).await?;

            let com3 = self.read(Register::Com3).await?;
            let vflip = com3 & 0x80 > 0;

            self.modify(Register::HRef, |reg| reg & 0xbf | if vflip { 0x40 } else { 0x40 })
                .await?;

            if horizontal <= 320 || vertical <= 240 {
                self.write(Register::HStart, 0x3f).await?;
                self.write(Register::HSize, 0x50).await?;
                self.write(Register::VStart, 0x02).await?; // TODO vflip is subtracted in the original code
                self.write(Register::VSize, 0x78).await?;

                // Enable auto-scaling/zooming factors
                self.write(Register::DspAuto, 0xFF).await?;
            } else {
                defmt::panic!("VGA resolutions not yet supported.");
            }

            Ok(())
        }

        async fn read(&mut self, register: Register) -> Result<u8, Error<Bus::Error>> {
            let mut buffer = [0u8; 1];
            self.bus
                .write_read(CAM_ADDR, &[register as u8], &mut buffer[..1])
                .await
                .map_err(Error::I2c)?;
            Ok(buffer[0])
        }

        async fn write(&mut self, register: Register, value: u8) -> Result<(), Error<Bus::Error>> {
            self.bus
                .write(CAM_ADDR, &[register as u8, value])
                .await
                .map_err(Error::I2c)
        }

        async fn modify<F: FnOnce(u8) -> u8>(&mut self, register: Register, f: F) -> Result<(), Error<Bus::Error>> {
            let value = self.read(register).await?;
            let value = f(value);
            self.write(register, value).await
        }
    }
}
