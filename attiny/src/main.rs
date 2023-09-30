#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(error_in_core)]

// example https://github.com/eric-wood/rust-attiny85-example/blob/main/src/main.rs
// hal https://rahix.github.io/avr-hal/attiny_hal/index.html
// i2c https://github.com/Rahix/avr-hal/issues/301
// sleep https://www.gadgetronicx.com/attiny85-sleep-modes-tutorial/
// i2c intro https://www.gadgetronicx.com/attiny85-i2c-protocol-tutorial/
// ads1x1x https://github.com/eldruin/ads1x1x-rs/tree/master

mod wire;
mod i2c_adc;
use attiny_hal as hal;
use hal::{port::Pin, prelude::*};
use i2c_adc::I2CADC;
use panic_halt as _;
use wire::{IoPin, SlavePollEvent, TwoWire};

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
        pub led1: hal::port::PB3 = pb3,
        pub i2c_sda: hal::port::PB0 = pb0,
        pub i2c_scl: hal::port::PB2 = pb2,
        pub led2: hal::port::PB4 = pb4,
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
    let mut i2c = TwoWire {
        address: None,
        delay: hal::delay::Delay::<hal::clock::MHz1>::new(),
        fast_mode: false,
        usi: i2c_peripheral,
        scl: IoPin::Input(pins.i2c_scl.forget_imode()),
        sda: IoPin::Input(pins.i2c_sda.forget_imode())
    };
    let mut led = pins.led1.into_output_high();

    let mut delay = hal::delay::Delay::<hal::clock::MHz1>::new();
    delay.delay_ms(1000u16);
    // let mut x = 0u8;
    // let mut message = heapless::Vec::<u8, 50>::new();
    i2c.begin(Some(8));

    loop {
        // let read = {
        //     let mut i2c_adc = I2CADC::new(i2c);
        //     let read = i2c_adc.read_all_mv();
        //     i2c = i2c_adc.destroy();
        //     read
        // };

        // message.clear();
        // let _ = message.extend_from_slice(b"x is ");
        // let _ = message.write_str_number(x);
        // let _ = message.extend_from_slice(b" and ");
        // let _ = message.write_str_number(read.a0);
        // let _ = message.push(b',');
        // let _ = message.write_str_number(read.a1);
        // let _ = message.push(b',');
        // let _ = message.write_str_number(read.a2);
        // let _ = message.push(b',');
        // let _ = message.write_str_number(read.a3);
        // let _ = message.push(b'\n');
        match i2c.slave_poll() {
            SlavePollEvent::StartRead => loop {
                let data = i2c.slave_read();
                match data {
                    wire::SlaveReadResult::Stop(v) => match v {
                        Some(v) => {
                            if v > 0 {
                                led.set_high();
                            } else {
                                led.set_low();
                            }
                        },
                        None => {
                            break;
                        }
                    }
                    wire::SlaveReadResult::Continue(v) => {
                        if v > 0 {
                            led.set_high();
                        } else {
                            led.set_low();
                        }
                    }
                }
            },
            SlavePollEvent::StartWrite => loop {
                let result = i2c.slave_write(Some(33));
                // x += 1;
                if result == wire::SlaveWriteResult::Stop {
                    break;
                }
            },
            SlavePollEvent::None => {
                
            }
        }
        // let mut message = [33u8; 6];
        // let mut message = b"x is \n";
        // message[..2].copy_from_slice(b"OK");
        // message[message.len() - 1] = b'\n';
        // let _ = i2c.write(8, &message);
        // delay.delay_ms(1000u16);
        // avr_device::asm::sleep();
    }
}
