use core::sync::atomic::{AtomicU16, Ordering};
use hal::{gpio::{GpioPin, Unknown}, prelude::_embedded_hal_async_digital_Wait};

static N_PULSES: AtomicU16 = AtomicU16::new(0);

pub fn get_n_pulses() -> u16 {
    N_PULSES.swap(0, Ordering::Acquire)
}

#[embassy_executor::task]
pub async fn pulse_counter(pulse_pin: GpioPin<Unknown, 21>) {
    let mut pin = pulse_pin.into_pull_down_input();
    loop {
        pin.wait_for_rising_edge().await.unwrap();
        N_PULSES.fetch_add(1, Ordering::Release);
    }
}
