use core::sync::atomic::{AtomicU8, Ordering};
use embassy_executor::Spawner;
use embassy_time::{Ticker, Duration};
use hal::{gpio::{GpioPin, Output, PushPull, Unknown}, prelude::{_embedded_hal_digital_v2_OutputPin, _embedded_hal_digital_v2_ToggleableOutputPin}};
use strum::FromRepr;

static WIFI_STATE: AtomicU8 = AtomicU8::new(0);

pub struct LedOutputController {
    // active_pin: GpioPin<Output<PushPull>, 2>,
    pulse_pin: GpioPin<Output<PushPull>, 26>
}

#[derive(FromRepr)]
#[repr(u8)]
pub enum WiFiState {
    Disabled,
    Connecting,
    Connected,
    Transmiting,
    Error
}

impl LedOutputController {
    pub fn new(spawner: Spawner, internal_led_pin: GpioPin<Unknown, 2>, wifi_led_pin: GpioPin<Unknown, 33>, pulse_led_pin: GpioPin<Unknown, 26>) -> Self {
        let mut active_pin = internal_led_pin.into_push_pull_output();
        active_pin.set_high().unwrap();
        let wifi_pin = wifi_led_pin.into_push_pull_output();
        spawner.spawn(wifi_state_task(wifi_pin)).ok();
        Self {
            // active_pin: internal_led_pin.into_push_pull_output(),
            pulse_pin: pulse_led_pin.into_push_pull_output()
        }
    }

    pub fn set_wifi_state(&mut self, val: WiFiState) {
        WIFI_STATE.store(val as u8, Ordering::Release);
    }

    pub fn pulse_begin(&mut self) {
        self.pulse_pin.set_high();
    }

    pub fn pulse_end(&mut self) {
        self.pulse_pin.set_low();
    }
}

#[embassy_executor::task]
async fn wifi_state_task(mut pin: GpioPin<Output<PushPull>, 33>) {
    loop {
        match WiFiState::from_repr(WIFI_STATE.load(Ordering::Acquire)).unwrap_or(WiFiState::Error) {
            WiFiState::Disabled => pin.set_low().unwrap(),
            WiFiState::Connecting => pin.toggle().unwrap(),
            WiFiState::Connected => pin.set_high().unwrap(),
            WiFiState::Transmiting => {
                pin.toggle();
                embassy_time::Timer::after(Duration::from_millis(100)).await;
                continue;
            },
            WiFiState::Error => {
                pin.toggle();
                embassy_time::Timer::after(Duration::from_millis(1000)).await;
            }
        };
        embassy_time::Timer::after(Duration::from_millis(1000)).await;
    }
}
