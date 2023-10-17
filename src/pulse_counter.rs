use core::sync::atomic::{AtomicU16, Ordering};
use embassy_time::{Timer, Duration};
use hal::{gpio::{GpioPin, Unknown}, prelude::_embedded_hal_async_digital_Wait};

static N_PULSES: AtomicU16 = AtomicU16::new(0);

pub fn get_n_pulses() -> u16 {
    N_PULSES.fetch_min(0, Ordering::Acquire)
}

#[embassy_executor::task]
pub async fn pulse_counter(pulse_pin: GpioPin<Unknown, 15>) {
    let mut pin = pulse_pin.into_pull_down_input();
    loop {
        esp_println::println!("Detected pulse");
        pin.wait_for_rising_edge().await.unwrap();
        Timer::after(Duration::from_millis(10)).await;
        N_PULSES.fetch_add(1, Ordering::Release);
    }
}
