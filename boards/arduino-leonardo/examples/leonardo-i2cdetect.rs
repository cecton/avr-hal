#![allow(unused_macros, dead_code)]
#![feature(slice_fill)]
#![no_std]
#![no_main]

extern crate panic_halt;
use arduino_leonardo::prelude::*;

/*
const FRAMES: &[&[u8]] = &[
    // TODO there seems to be an X offset in xbm
    /*
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-1.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-2.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-3.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-4.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-5.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-6.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-7.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-8.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-9.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-10.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-11.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-12.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-13.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-14.xbm"),
    xbm::include_xbm!("boards/arduino-leonardo/examples/F501-15.xbm"),
    */
    include_bytes!("F501-1.raw"),
    include_bytes!("F501-2.raw"),
    include_bytes!("F501-3.raw"),
    include_bytes!("F501-4.raw"),
    include_bytes!("F501-5.raw"),
    include_bytes!("F501-6.raw"),
    include_bytes!("F501-7.raw"),
    include_bytes!("F501-8.raw"),
    include_bytes!("F501-9.raw"),
    include_bytes!("F501-10.raw"),
    include_bytes!("F501-11.raw"),
    include_bytes!("F501-12.raw"),
    include_bytes!("F501-13.raw"),
    include_bytes!("F501-14.raw"),
    include_bytes!("F501-15.raw"),
];
*/
const FRAME_1: &[u8] = include_bytes!("F501-1.raw");
const FRAME_2: &[u8] = include_bytes!("F501-2.raw");
const FRAME_3: &[u8] = include_bytes!("F501-3.raw");
const FRAME_4: &[u8] = include_bytes!("F501-4.raw");
const FRAME_5: &[u8] = include_bytes!("F501-5.raw");
const FRAME_6: &[u8] = include_bytes!("F501-6.raw");
const FRAME_7: &[u8] = include_bytes!("F501-7.raw");
const FRAME_8: &[u8] = include_bytes!("F501-8.raw");
const FRAME_9: &[u8] = include_bytes!("F501-9.raw");
const FRAME_10: &[u8] = include_bytes!("F501-10.raw");
const FRAME_11: &[u8] = include_bytes!("F501-11.raw");
const FRAME_12: &[u8] = include_bytes!("F501-12.raw");
const FRAME_13: &[u8] = include_bytes!("F501-13.raw");
const FRAME_14: &[u8] = include_bytes!("F501-14.raw");
const FRAME_15: &[u8] = include_bytes!("F501-15.raw");

static mut BUFFER: [u8; 1024 + 1] = [0; 1024 + 1];

#[arduino_leonardo::entry]
fn main() -> ! {
    let dp = arduino_leonardo::Peripherals::take().unwrap();

    let mut delay = arduino_leonardo::Delay::new();
    let mut pins = arduino_leonardo::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD, dp.PORTE);
    let mut led_rx = pins.led_rx.into_output(&mut pins.ddr);
    let mut serial = arduino_leonardo::Serial::new(
        dp.USART1,
        pins.d0,
        pins.d1.into_output(&mut pins.ddr),
        57600,
    );
    let mut i2c = arduino_leonardo::I2c::new(
        dp.TWI,
        pins.d2.into_pull_up_input(&mut pins.ddr),
        pins.d3.into_pull_up_input(&mut pins.ddr),
        400000,
    );

    /*
    ufmt::uwriteln!(&mut serial, "Write direction test:\r").void_unwrap();
    i2c.i2cdetect(&mut serial, arduino_leonardo::hal::i2c::Direction::Write)
        .void_unwrap();
    ufmt::uwriteln!(&mut serial, "\r\nRead direction test:\r").void_unwrap();
    i2c.i2cdetect(&mut serial, arduino_leonardo::hal::i2c::Direction::Read)
        .void_unwrap();
    rx.set_high().void_unwrap();
    */

    let address = 0b0111100;

    macro_rules! write {
        ($($byte:expr),+) => {{
            if let Err(err) = i2c.write(address, &[0b00000000, $($byte),+]) {
                ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
            };
        }};
    }

    macro_rules! write_data {
        ($($byte:expr),+) => {{
            if let Err(err) = i2c.write(address, &[0b01000000, $($byte),+]) {
                ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
            };
        }};
    }

    macro_rules! write_raw {
        ($arr:expr) => {{
            if let Err(err) = i2c.write(address, $arr) {
                ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
            };
        }};
    }

    let mut screen = unsafe { Screen::new(&mut BUFFER, address, 128, 128) };
    if let Err(err) = screen.init(&mut i2c, &mut delay) {
        ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
    }

    if let Err(err) = screen.fill(&mut i2c, 0x00) {
        ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
    }

    macro_rules! draw_frame {
        ($frame:expr) => {{
            screen.draw(&mut i2c, $frame, 40, 42, &mut serial);
            delay.delay_ms(1000u16);
            led_rx.toggle().void_unwrap();
        }};
    }

    write!(0x15, 0, 40 / 2 - 1);
    write!(0x75, 0, 42 - 1);

    let (sph, spl) = sph_spl();
    ufmt::uwriteln!(&mut serial, "SPH={} SPL={}\r", sph, spl).void_unwrap();

    loop {
        //for i in 0..15 {
            /*
            if let Err(err) = screen.fill(&mut i2c, 0x00) {
                ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
            }
            */

            //ufmt::uwriteln!(&mut serial, "{}\r", i).void_unwrap();
            /*
            if let Err(err) = screen.draw_rect(&mut i2c, i * 0xff, 32, 64, 40, 42, &mut serial) {
                ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
            }
            */
            draw_frame!(FRAME_1);
            draw_frame!(FRAME_2);
            draw_frame!(FRAME_3);
            draw_frame!(FRAME_4);
            draw_frame!(FRAME_5);
            draw_frame!(FRAME_6);
            draw_frame!(FRAME_7);
            draw_frame!(FRAME_8);
            draw_frame!(FRAME_9);
            draw_frame!(FRAME_10);
            draw_frame!(FRAME_11);
            draw_frame!(FRAME_12);
            draw_frame!(FRAME_13);
            draw_frame!(FRAME_14);
            draw_frame!(FRAME_15);

        //}
    }

    loop {}

    /*
    let mut led_rx_state = false;
    let mut toggle_led_rx = move || if led_rx_state {
        led_rx.set_low().void_unwrap();
        led_rx_state = !led_rx_state;
    } else {
        led_rx.set_high().void_unwrap();
        led_rx_state = !led_rx_state;
    };

    let step = 64;
    let mut counter: u8 = 255 - step + 1;
    let mut direction = true;
    loop {
        //ufmt::uwrite!(&mut serial, "{}", counter).void_unwrap();
        let _ = nb::block!(serial.read()).void_unwrap();

        match i2c.ping_slave(address, arduino_leonardo::hal::i2c::Direction::Write) {
            Ok(true) => {
                toggle_led_rx();
                if let Err(err) = screen.fill(&mut i2c, counter) {
                    ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
                }
                toggle_led_rx();

                if direction {
                    //counter = counter.wrapping_sub(step);
                    counter = 0x00;
                } else {
                    //counter = counter.wrapping_add(step);
                    counter = 0xff;
                }

                if counter == 0 || counter == 255 - step + 1 || true {
                    direction = !direction;
                }
            }
            Ok(false) => {}
            Err(err) => {
                ufmt::uwriteln!(&mut serial, "Error: {:?}\r", err).void_unwrap();
            }
        }
        //delay.delay_ms(1000u16);
    }
    */
}

struct Screen {
    address: u8,
    width: u8,
    height: u8,
    buffer: &'static mut [u8; 1024 + 1],
}

impl Screen {
    fn new(buffer: &'static mut [u8; 1024 + 1], address: u8, width: u8, height: u8) -> Screen {
        Self {
            address,
            width,
            height,
            buffer,
        }
    }

    fn init<W: _embedded_hal_blocking_i2c_Write, D: _embedded_hal_blocking_delay_DelayMs<u16>>(
        &self,
        i2c: &mut W,
        delay: &mut D,
    ) -> Result<(), W::Error> {
        macro_rules! write {
            ($($byte:expr),+) => {{
                i2c.write(self.address, &[0b00000000, $($byte),+])?;
            }};
        }

        /*
        write!(0xae); //--turn off oled panel

        write!(0x15); //set column addresses
        write!(0x00); //start column  0
        write!(0x7f); //end column  127

        write!(0x75); //set row addresses
        write!(0x00); //start row  0
        write!(0x7f); //end row  127

        write!(0x81); //set contrast control
        write!(0x80); //50% (128/255)

        write!(0xa0); //gment remap
        write!(0x51); //51 (To my understanding, this is orientation

        write!(0xa1); //start line
        write!(0x00);

        write!(0xa2); //display offset
        write!(0x00);

        write!(0xa4); //rmal display
        write!(0xa8); //set multiplex ratio
        write!(0x7f);

        write!(0xb1); //set phase leghth
        write!(0xf1);

        write!(0xb3); //set dclk
        write!(0x00); //80Hz:0xc1 90Hz:0xe1  100Hz:0x00  110Hz:0x30 120Hz:0x50  130Hz:0x70   01

        write!(0xab); //Enable vReg
        write!(0x01);

        write!(0xb6); //set phase leghth
        write!(0x0f);

        write!(0xbe); //Set vcomh voltage
        write!(0x0f);

        write!(0xbc); //set pre-charge voltage
        write!(0x08);

        write!(0xd5); //second precharge period
        write!(0x62);

        write!(0xfd); //Unlock commands
        write!(0x12);
        */

        write!(0xAF);
        write!(0xa0, 0x51);
        //delay.delay_ms(300u16);

        Ok(())
    }

    fn fill<W: _embedded_hal_blocking_i2c_Write>(
        &mut self,
        i2c: &mut W,
        c: u8,
    ) -> Result<(), W::Error> {
        let x = c / 16;
        let x = x * 16 + x;
        for i in 1..=1024 {
            self.buffer[i] = x;
        }

        macro_rules! write {
            ($($byte:expr),+) => {{
                i2c.write(self.address, &[0b00000000, $($byte),+])?;
            }};
        }

        write!(0x15, 0, self.width / 2 - 1);
        write!(0x75, 0, self.height - 1);
        let mut pixels: usize = self.width as usize * self.height as usize;
        while pixels >= 2 * 1024 {
            i2c.write(self.address, self.buffer)?;
            pixels -= 2 * 1024;
        }
        if pixels > 0 {
            i2c.write(self.address, &self.buffer[..=pixels])?;
        }

        Ok(())
    }

    #[inline(never)]
    fn draw<W: _embedded_hal_blocking_i2c_Write>(
        &mut self,
        i2c: &mut W,
        image: &[u8],
        width: u8,
        height: u8,
        mut writer: &mut impl ufmt::uWrite,
    ) -> Result<(), W::Error> {
        macro_rules! write {
            ($($byte:expr),+) => {{
                i2c.write(self.address, &[0b00000000, $($byte),+])?;
            }};
        }

        /*
            for i in 0..42 {
                let _ = ufmt::uwriteln!(&mut writer, "line={} {:?}\r", i, self.buffer[(i*20+1)..((i+1)*20+1)]);
            }
        */

        //use crc::crc16;
        //let _ = ufmt::uwriteln!(&mut writer, "crc={}\r", crc16::checksum_x25(image));
        //self.buffer.fill(0b01010101);
        //self.buffer[0] = 0b01000000;
        //let _ = ufmt::uwriteln!(&mut writer, "checksum={}\r", checksum(image));
        //write!(0x15, 0, width / 2 - 1);
        //write!(0x75, 0, height - 1);
        let mut pixels: usize = width as usize * height as usize;
        let mut chunks = image.iter().map(#[inline(never)] |x| {
            [
                (x & 0b10000000).count_ones() as u8 * 0b11110000
                + (x & 0b01000000).count_ones() as u8 * 0b00001111,
                (x & 0b00100000).count_ones() as u8 * 0b11110000
                + (x & 0b00010000).count_ones() as u8 * 0b00001111,
                (x & 0b00001000).count_ones() as u8 * 0b11110000
                + (x & 0b00000100).count_ones() as u8 * 0b00001111,
                (x & 0b00000010).count_ones() as u8 * 0b11110000
                + (x & 0b00000001).count_ones() as u8 * 0b00001111,
            ]
        });
        /*
        while pixels >= 2 * 1024 {
            for i in (1..=1024).step_by(4) {
                self.buffer[i..(i+4)].copy_from_slice(&chunks.next().unwrap());
            }

            i2c.write(self.address, &self.buffer)?;
            pixels -= 2 * 1024;
        }
        */
        if pixels > 0 {
            let mut i = 0;
            while let Some(chunk) = chunks.next() {
                self.buffer[i..(i+4)].copy_from_slice(&chunk);
                /*
                if i > 1 && pixels % 40 == 0 {
                    let _ = ufmt::uwriteln!(&mut writer, "line={} {:?}\r", 42 - pixels / 40, self.buffer[(i+4-20)..(i+4)]);
                }
                */
                i += 4;
                pixels -= 8;
            }
            self.buffer[0] = 0b01000000;

            /*
            for i in 0..42 {
                let _ = ufmt::uwriteln!(&mut writer, "line={} {:?}\r", i, self.buffer[(i*20+1)..((i+1)*20+1)]);
            }
            */

            //let _ = ufmt::uwriteln!(&mut writer, "pixels={} i={}\r", pixels, i);
            i2c.write(self.address, &self.buffer[..=i])?;
            /*
            let (sph, spl) = sph_spl();
            let _ = ufmt::uwriteln!(&mut writer, "SPH={} SPL={}\r", sph, spl);
            */
            //let _ = ufmt::uwriteln!(&mut writer, "crc={}\r", crc16::checksum_x25(image));
        }

        Ok(())
    }

    fn draw_rect<W: _embedded_hal_blocking_i2c_Write>(
        &mut self,
        i2c: &mut W,
        c: u8,
        x: u8,
        y: u8,
        width: u8,
        height: u8,
        mut writer: &mut impl ufmt::uWrite,
    ) -> Result<(), W::Error> {
        {
            let x = c / 16;
            let x = x * 16 + x;
            for i in 1..=1024 {
                self.buffer[i] = x & 0b11000011;
            }
        }

        macro_rules! write {
            ($($byte:expr),+) => {{
                i2c.write(self.address, &[0b00000000, $($byte),+])?;
            }};
        }

        write!(0x15, x / 2, x / 2 + width / 2 - 1);
        write!(0x75, y, y + height - 1);
        let mut pixels: usize = width as usize * height as usize;
        /*
        let mut chunks = image.iter().map(|x| [
            (x & 0b10000000).count_ones() as u8 * 0b11110000
            + (x & 0b01000000).count_ones() as u8 * 0b00001111,
            (x & 0b00100000).count_ones() as u8 * 0b11110000
            + (x & 0b00010000).count_ones() as u8 * 0b00001111,
            (x & 0b00001000).count_ones() as u8 * 0b11110000
            + (x & 0b00000100).count_ones() as u8 * 0b00001111,
            (x & 0b00000010).count_ones() as u8 * 0b11110000
            + (x & 0b00000001).count_ones() as u8 * 0b00001111,
        ]);
        */
        while pixels >= 2 * 1024 {
            /*
            for i in (1..=1024).step_by(4) {
                self.buffer[i..(i+4)].copy_from_slice(&chunks.next().unwrap());
            }
            */

            i2c.write(self.address, self.buffer)?;
            pixels -= 2 * 1024;
        }
        if pixels > 0 {
            /*
            let mut i = 1;
            while let Some(chunk) = chunks.next() {
                self.buffer[i..(i+4)].copy_from_slice(&chunk);
                let _ = ufmt::uwriteln!(&mut writer, "{}: {:?}\r", i, chunk);
                i += 4;
            }
            */

            i2c.write(self.address, &self.buffer[..=pixels])?;
        }

        Ok(())
    }
}

fn checksum(a: &[u8]) -> usize {
    let mut sum: usize = 0;
    for (i, x) in a.iter().enumerate() {
        sum = sum.wrapping_add(i * x.count_ones() as usize);
    }
    sum
}

fn sph_spl() -> (u8, u8) {
    // https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/ATMega32U4.pdf page 410
    let p = 0x5e as *const u8;
    let sph = unsafe { *p };
    let p = 0x5d as *const u8;
    let spl = unsafe { *p };
    (sph, spl)
}
