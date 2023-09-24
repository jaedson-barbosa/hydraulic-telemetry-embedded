#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

// example https://github.com/eric-wood/rust-attiny85-example/blob/main/src/main.rs
// hal https://rahix.github.io/avr-hal/attiny_hal/index.html
// i2c https://github.com/Rahix/avr-hal/issues/301
// sleep https://www.gadgetronicx.com/attiny85-sleep-modes-tutorial/
// i2c intro https://www.gadgetronicx.com/attiny85-i2c-protocol-tutorial/
// ads1x1x https://github.com/eldruin/ads1x1x-rs/tree/master

mod i2c;
use core::fmt::Write;
mod i2c_adc;
use attiny_hal as hal;
use hal::{port::Pin, prelude::*};
use i2c::I2C;
use i2c_adc::I2CADC;
use panic_halt as _;

// fn enable_interrupt(dp: &hal::Peripherals) {}

// fn enable_sleep(dp: &hal::Peripherals) {
//     dp.CPU.mcucr.write(|w| w.se().set_bit().sm().pdown());
// }

// fn disable_pwdown(dp: &hal::Peripherals) {
//     dp.CPU.mcucr.write(|w| w.sm().idle());
// }

avr_hal_generic::renamed_pins! {
    type Pin = Pin;

    pub struct Pins from hal::Pins {
        pub pb1: hal::port::PB1 = pb1,
        pub random: hal::port::PB3 = pb3,
        pub i2c_sda: hal::port::PB0 = pb0,
        pub i2c_scl: hal::port::PB2 = pb2,
        pub led: hal::port::PB4 = pb4,
    }
}

pub trait NumberWrite<const N: usize> {
    fn write_str_number(&mut self, n: u16) -> Result<(), ()>;
}

impl<const N: usize> NumberWrite<N> for heapless::Vec<u8, N> {
    fn write_str_number(&mut self, mut n: u16) -> Result<(), ()> {
        if n == 0 {
            return self.push(b'0').map_err(|_| ());
        }
        let l = self.len();
        while n > 0 {
            let v = (n % 10) as u8 + b'0';
            self.insert(l, v).map_err(|_| ())?;
            n /= 10;
        }
        Ok(())
    }
}

#[attiny_hal::entry]
fn main() -> ! {
    let dp = hal::Peripherals::take().unwrap();
    // enable_sleep(&dp);

    let pins = Pins::with_mcu_pins(hal::pins!(dp));
    let i2c_peripheral = dp.USI;
    let mut i2c: I2C = I2C::with_external_pullup(i2c_peripheral, pins.i2c_sda, pins.i2c_scl);

    let mut delay = hal::delay::Delay::<hal::clock::MHz1>::new();
    delay.delay_ms(1000u16);
    let mut x = 0u16;
    let mut message = heapless::Vec::<u8, 50>::new();

    loop {
        let read = {
            let mut i2c_adc = I2CADC::new(i2c);
            let read = i2c_adc.read_all_mv();
            i2c = i2c_adc.destroy();
            read
        };

        message.clear();
        let _ = message.extend_from_slice(b"values ");
        let _ = message.write_str_number(x);
        let _ = message.extend_from_slice(b" and ");
        let _ = message.write_str_number(read.a0);
        let _ = message.push(b',');
        let _ = message.write_str_number(read.a1);
        let _ = message.push(b',');
        let _ = message.write_str_number(read.a2);
        let _ = message.push(b',');
        let _ = message.write_str_number(read.a3);
        let _ = message.push(b'\n');
        let _ = i2c.write(8, &message);
        x += 1;
        delay.delay_ms(1000u16);
        // avr_device::asm::sleep();
    }
}
