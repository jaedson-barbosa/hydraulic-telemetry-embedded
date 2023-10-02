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
use desse::Desse;
use hal::prelude::*;
use panic_halt as _;
use wire::{IoPin, SlavePollEvent, TwoWire};
use shared::{AttinyRequest,AttinyResponse};

// fn enable_interrupt(dp: &hal::Peripherals) {}

// fn enable_sleep(dp: &hal::Peripherals) {
//     dp.CPU.mcucr.write(|w| w.se().set_bit().sm().pdown());
// }

// fn disable_pwdown(dp: &hal::Peripherals) {
//     dp.CPU.mcucr.write(|w| w.sm().idle());
// }

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

    let pins = hal::pins!(dp);
    let i2c_peripheral = dp.USI;
    let mut i2c = TwoWire {
        address: None,
        delay: hal::delay::Delay::<hal::clock::MHz1>::new(),
        fast_mode: false,
        usi: i2c_peripheral,
        scl: IoPin::Input(pins.pb2.forget_imode()),
        sda: IoPin::Input(pins.pb0.forget_imode())
    };
    let mut charger_en_pin = pins.pb3.into_output();

    let mut delay = hal::delay::Delay::<hal::clock::MHz1>::new();
    delay.delay_ms(1000u16);
    i2c.begin(Some(8));

    'main_loop: loop {
        // let read = {
        //     let mut i2c_adc = I2CADC::new(i2c);
        //     let read = i2c_adc.read_all_mv();
        //     i2c = i2c_adc.destroy();
        //     read
        // };
        match i2c.slave_poll() {
            SlavePollEvent::StartRead => {
                let mut buffer = [0u8; 2];
                // TODO prepare code for invalid request sizes
                for byte in buffer.iter_mut() {
                    match i2c.slave_read() {
                        wire::SlaveReadResult::Stop(v) => match v {
                            Some(v) => *byte = v,
                            None => continue 'main_loop
                        },
                        wire::SlaveReadResult::Continue(v) => *byte = v
                    }
                }
                let request = AttinyRequest::deserialize_from(&buffer);
                match request {
                    AttinyRequest::UpdateChargerEn(v) => if v { charger_en_pin.set_high() } else { charger_en_pin.set_low() }
                }
            },
            SlavePollEvent::StartWrite =>  {
                let state_buffer = AttinyResponse { charger_en: charger_en_pin.is_set_high() }.serialize();
                for byte in state_buffer {
                    match i2c.slave_write(Some(byte)) {
                        wire::SlaveWriteResult::Stop => continue 'main_loop,
                        _ => {}
                    }
                }
                while i2c.slave_write(None) == wire::SlaveWriteResult::Continue {}
            },
            SlavePollEvent::None => {}
        }
    }
}
