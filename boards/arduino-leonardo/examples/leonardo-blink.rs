#![no_std]
#![no_main]

extern crate panic_halt;
use arduino_leonardo::prelude::*;
use ssd1306::{prelude::*, Builder, I2CDIBuilder, displaysize::*};
use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::BinaryColor,
    prelude::*,
};

#[arduino_leonardo::entry]
fn main() -> ! {
    let dp = arduino_leonardo::Peripherals::take().unwrap();

    let mut delay = arduino_leonardo::Delay::new();
    let mut pins = arduino_leonardo::Pins::new(
        dp.PORTB,
        dp.PORTC,
        dp.PORTD,
        dp.PORTE,
    );
    let mut i2c = arduino_leonardo::I2c::new(
        dp.TWI,
        pins.d2.into_pull_up_input(&mut pins.ddr),
        pins.d3.into_pull_up_input(&mut pins.ddr),
        400000,
    );

    /*
    let mut led0 = pins.led_rx.into_output(&mut pins.ddr);
    let mut led1 = pins.led_tx.into_output(&mut pins.ddr);

    loop {
        led0.set_low().void_unwrap();
        led1.set_high().void_unwrap();
        delay.delay_ms(1000u16);
        led0.set_high().void_unwrap();
        led1.set_low().void_unwrap();
        delay.delay_ms(1000u16);
    }
    */

    let address = 0b0111100;
    let interface = I2CDIBuilder::new().with_i2c_addr(address).init(i2c);
    //::<DisplaySize128x128>
    let mut disp: GraphicsMode<_> = Builder::new().connect(interface).into();
    //let mut disp = Builder::new().connect(interface);
    disp.init().unwrap();
    //disp.display_on(true);

    let raw: ImageRaw<BinaryColor> = ImageRaw::new(include_bytes!("./rust.raw"), 64, 64);

    let im = Image::new(&raw, Point::new(32, 0));

    im.draw(&mut disp).unwrap();

    disp.flush().unwrap();

    loop {}
}
