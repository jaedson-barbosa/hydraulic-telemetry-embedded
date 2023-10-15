use core::sync::atomic::{AtomicU16, Ordering};
use embassy_executor::Spawner;
use embassy_time::{Timer, Duration};
use hal::{gpio::{GpioPin, Input, PullDown, Unknown}, prelude::_embedded_hal_async_digital_Wait};

static N_PULSES: AtomicU16 = AtomicU16::new(0);

pub fn start_pulse_counter(spawner: &Spawner, pulse_pin: GpioPin<Unknown, 15>) {
    let pulse_pin = pulse_pin.into_pull_down_input();
    spawner.spawn(pulse_counter(pulse_pin)).ok();
}

pub fn get_n_pulses() -> u16 {
    N_PULSES.load(Ordering::Acquire)
}

#[embassy_executor::task]
async fn pulse_counter(mut pin: GpioPin<Input<PullDown>, 15>) {
    loop {
        pin.wait_for_rising_edge().await.unwrap();
        Timer::after(Duration::from_millis(10)).await;
        N_PULSES.fetch_add(1, Ordering::Release);
    }
}
