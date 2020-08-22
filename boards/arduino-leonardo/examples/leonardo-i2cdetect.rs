#![no_std]
#![no_main]

extern crate panic_halt;
use arduino_leonardo::prelude::*;

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
        50000,
    );

    let address = 0b0111100; // replace this by the address of your device

    // a small macro to help us send commands without repeating ourselves too much
    macro_rules! write_cmd {
        ($($bytes:expr),+) => {{
            if let Err(err) = i2c.write(address, &[0b00000000, $($bytes),+]) {
                ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
            }
        }};
    }

    // turn on the screen
    write_cmd!(0xaf);
    write_cmd!(0xa0, 0x51);

    // fill the screen
    // our screen is 128 pixels long but we divide by 2 because there are 2 pixels per byte
    write_cmd!(0x15, 0, 63);
    // our screen is 128 pixels height
    write_cmd!(0x75, 0, 127);
    // we initialize an array of 64 + 1 bytes because 128 pixels / 2 + 1 byte for the control byte
    let mut data = [0x00; 2049];
    data[0] = 0b01000000; // the control byte
    for _ in 0..4 {
        if let Err(err) = i2c.write(address, &data) {
            ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap();
        }
    }
    // we should free the memory as it is quite limited
    drop(data);

    loop {
        match i2c.ping_slave(address, arduino_leonardo::hal::i2c::Direction::Write) {
            Ok(true) => led_rx.set_low().void_unwrap(),
            Ok(false) => led_rx.set_high().void_unwrap(),
            Err(err) => ufmt::uwriteln!(&mut serial, "Error: {:?}", err).void_unwrap(),
        }
        delay.delay_ms(1000u16);
    }
}
