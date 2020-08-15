#![no_std]
#![no_main]

extern crate panic_halt;
use arduino_leonardo::prelude::*;

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
}
