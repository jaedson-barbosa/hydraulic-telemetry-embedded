#![no_std]
#![no_main]

use attiny_hal as hal;
use avr_hal_generic::simple_pwm::IntoPwmPin;
use hal::simple_pwm::*;
use panic_halt as _;

#[hal::entry]
fn main() -> ! {
    let dp = hal::Peripherals::take().unwrap();
    let pins = hal::pins!(dp);

    let timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Direct);
    let mut adc = hal::Adc::<hal::clock::MHz1>::new(dp.ADC, Default::default());
    let input = &pins.pb3.into_analog_input(&mut adc);

    let mut dc = 128;
    let target = 3300;

    let mut pwm = pins.pb1.into_output().into_pwm(&timer0);
    pwm.enable();
    loop {
        let a0 = input.analog_read(&mut adc) * 5; // in mV
        if a0 > target && dc > 0 {
            dc -= 1;
        } else if a0 < target && dc < 255 {
            dc += 1;
        }
        pwm.set_duty(dc);
    }
}
