#![allow(unused_macros, dead_code)]
#![no_std]
#![no_main]

extern crate panic_halt;
use arduino_leonardo::prelude::*;

#[arduino_leonardo::entry]
fn main() -> ! {
    let dp = arduino_leonardo::Peripherals::take().unwrap();

    let mut delay = arduino_leonardo::Delay::new();
    let mut pins = arduino_leonardo::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD, dp.PORTE);
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

    ufmt::uwrite!(&mut serial, ".").void_unwrap();

    let mut screen = Screen::new(address, 128, 128);
    if let Err(err) = screen.init(&mut i2c, &mut delay) {
        ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
    }

    ufmt::uwrite!(&mut serial, ".").void_unwrap();

    let step = 64;
    let mut counter: u8 = 255 - step + 1;
    let mut direction = true;
    loop {
        ufmt::uwrite!(&mut serial, ".").void_unwrap();
        match i2c.ping_slave(address, arduino_leonardo::hal::i2c::Direction::Write) {
            Ok(true) => {
                ufmt::uwrite!(&mut serial, ".").void_unwrap();
                if let Err(err) = screen.fill(&mut i2c, counter) {
                    ufmt::uwrite!(&mut serial, ".").void_unwrap();
                    ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
                }

                if direction {
                    counter = counter.wrapping_sub(step);
                } else {
                    counter = counter.wrapping_add(step);
                }

                if counter == 0 || counter == 255 - step + 1 {
                    direction = !direction;
                }
            }
            Ok(false) => {}
            Err(err) => {
                ufmt::uwriteln!(&mut serial, "Error: {:?}\r", err).void_unwrap();
            }
        }
        ufmt::uwrite!(&mut serial, "{}", counter).void_unwrap();
        let i = nb::block!(serial.read()).void_unwrap();
        if i == 114 {
            ufmt::uwrite!(&mut serial, "BOOTLOADER").void_unwrap();
        }
    }
}

struct Screen {
    address: u8,
    width: u8,
    height: u8,
    buffer: [u8; 2049],
}

impl Screen {
    fn new(address: u8, width: u8, height: u8) -> Screen {
        Self {
            address,
            width,
            height,
            buffer: [0b01000000; 2049],
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

        write!(0xAF);
        delay.delay_ms(300u16);

        Ok(())
    }

    fn fill<W: _embedded_hal_blocking_i2c_Write>(
        &mut self,
        i2c: &mut W,
        c: u8,
    ) -> Result<(), W::Error> {
        let x = c / 16;
        let x = x * 16 + x;
        for i in 1..2049 {
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
        while pixels >= 2 * 2048 {
            i2c.write(self.address, &self.buffer)?;
            pixels -= 2 * 2048;
        }
        if pixels > 0 {
            i2c.write(self.address, &self.buffer[..=pixels])?;
        }

        Ok(())
    }
}
