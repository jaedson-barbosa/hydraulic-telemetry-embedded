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
use attiny_hal as hal;
use desse::Desse;
use hal::prelude::*;
use panic_halt as _;
use shared::{AttinyRequest, AttinyResponse};
use wire::{IoPin, SlavePollEvent, SlaveReadResult, TwoWire};

static mut N_PULSES: u8 = 0;

fn enable_interrupt(dp: &hal::Peripherals) {
    dp.EXINT.gimsk.write(|w| w.pcie().set_bit());
    const PCINT1: u8 = 0b10;
    dp.EXINT.pcmsk.write(|w| w.bits(PCINT1));
    unsafe { avr_device::interrupt::enable() };
}

#[avr_device::interrupt(attiny85)]
fn PCINT0() {
    unsafe {
        if N_PULSES > 127 {
            N_PULSES = 0;
        } else {
            N_PULSES += 1
        }
    };
}

// fn enable_sleep(dp: &hal::Peripherals) {
//     dp.CPU.mcucr.write(|w| w.se().set_bit().sm().pdown());
// }

// fn disable_pwdown(dp: &hal::Peripherals) {
//     dp.CPU.mcucr.write(|w| w.sm().idle());
// }

#[attiny_hal::entry]
fn main() -> ! {
    let dp = hal::Peripherals::take().unwrap();
    // enable_sleep(&dp);
    enable_interrupt(&dp);

    let pins = hal::pins!(dp);
    let i2c_peripheral = dp.USI;
    let mut i2c = TwoWire {
        address: None,
        delay: hal::delay::Delay::<hal::clock::MHz1>::new(),
        fast_mode: false,
        usi: i2c_peripheral,
        scl: IoPin::Input(pins.pb2.forget_imode()),
        sda: IoPin::Input(pins.pb0.forget_imode()),
    };

    let mut adc = hal::Adc::<hal::clock::MHz1>::new(
        dp.ADC,
        hal::adc::AdcSettings {
            ref_voltage: hal::adc::ReferenceVoltage::Internal1_1,
            ..Default::default()
        },
    );
    let input = &pins.pb3.into_analog_input(&mut adc);

    let mut delay = hal::delay::Delay::<hal::clock::MHz1>::new();
    delay.delay_ms(1000u16);
    i2c.begin(Some(8));

    'main_loop: loop {
        match i2c.slave_poll() {
            SlavePollEvent::StartRead => {
                let mut buffer = [0u8; 1];
                let mut already_stopped = false;
                for byte in buffer.iter_mut() {
                    match i2c.slave_read() {
                        SlaveReadResult::Stop(v) => match v {
                            Some(v) => {
                                *byte = v;
                                already_stopped = true;
                            }
                            None => continue 'main_loop, // invalid none
                        },
                        SlaveReadResult::Continue(v) => *byte = v,
                    }
                }
                if !already_stopped {
                    match i2c.slave_read() {
                        SlaveReadResult::Stop(v) => match v {
                            Some(_) => continue 'main_loop, // invalid stop,
                            None => {}
                        },
                        SlaveReadResult::Continue(_) => continue 'main_loop, // invalid continue,
                    }
                }
                let request = AttinyRequest::deserialize_from(&buffer);
                match request {
                    AttinyRequest::None => {}
                }
            }
            SlavePollEvent::StartWrite => {
                let n_pulses = unsafe { N_PULSES };
                let generator_mv = input.analog_read(&mut adc) * 12; // 11 * 1.1 = 12
                let state_buffer = AttinyResponse {
                    n_pulses,
                    generator_mv,
                }
                .serialize();
                for byte in state_buffer {
                    match i2c.slave_write(Some(byte)) {
                        wire::SlaveWriteResult::Stop => continue 'main_loop,
                        _ => {}
                    }
                }
                unsafe { N_PULSES -= n_pulses };
                while i2c.slave_write(None) == wire::SlaveWriteResult::Continue {}
            }
            SlavePollEvent::None => {}
        }
    }
}
