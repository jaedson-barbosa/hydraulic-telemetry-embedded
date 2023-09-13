#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use attiny_hal as hal;
use panic_halt as _;

fn enable_interrupt(dp: &hal::Peripherals) {

}

fn enable_sleep(dp: &hal::Peripherals) {
    dp.CPU.mcucr.write(|w| w.se().set_bit().sm().pdown());
}

fn disable_pwdown(dp: &hal::Peripherals) {
    dp.CPU.mcucr.write(|w| w.sm().idle());
}

#[attiny_hal::entry]
fn main() -> ! {
    let dp = hal::Peripherals::take().unwrap();
    // let pins = hal::pins!(dp);
    // let mut led = pins.pb3.into_output();
    enable_sleep(&dp);

    // let mut delay = hal::delay::Delay::<hal::clock::MHz1>::new();
    loop {
        avr_device::asm::sleep();
        // led.toggle();
        // delay.delay_ms(1000u16);
        // led.toggle();
        // delay.delay_ms(5000u16);
    }
}
