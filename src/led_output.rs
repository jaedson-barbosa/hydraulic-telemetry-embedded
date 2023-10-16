use core::sync::atomic::{AtomicU8, Ordering};
use embassy_time::{Duration, Timer};
use hal::{
    gpio::{GpioPin, Output, PushPull, Unknown},
    prelude::{_embedded_hal_digital_v2_OutputPin, _embedded_hal_digital_v2_ToggleableOutputPin},
};
use strum::FromRepr;

static WIFI_STATE: AtomicU8 = AtomicU8::new(0);

#[derive(serde::Serialize, Clone, Copy, Debug, FromRepr)]
#[repr(u8)]
pub enum WiFiState {
    Disabled,
    Connecting,
    Connected
}

impl WiFiState {
    pub fn get() -> Self {
        WiFiState::from_repr(WIFI_STATE.load(Ordering::Acquire)).unwrap_or(WiFiState::Disabled)
    }

    pub fn set(val: Self) {
        WIFI_STATE.store(val as u8, Ordering::Release);
    }
}

pub struct PulseLed(GpioPin<Output<PushPull>, 26>);

impl PulseLed {
    pub fn init(pulse_led_pin: GpioPin<Unknown, 26>) -> Self {
        PulseLed(pulse_led_pin.into_push_pull_output())
    }

    pub fn pulse_begin(&mut self) {
        self.0.set_high().unwrap();
    }

    pub fn pulse_end(&mut self) {
        self.0.set_low().unwrap();
    }
}

#[embassy_executor::task]
pub async fn wifi_led_state_task(wifi_led_pin: GpioPin<Unknown, 33>) {
    let mut pin = wifi_led_pin.into_push_pull_output();
    loop {
        match WiFiState::get() {
            WiFiState::Disabled => pin.set_low().unwrap(),
            WiFiState::Connecting => pin.toggle().unwrap(),
            WiFiState::Connected => pin.set_high().unwrap(),
        };
        Timer::after(Duration::from_millis(500)).await;
    }
}
