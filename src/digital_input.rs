use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::Spawner;
use esp_hal_common::gpio::PullDown;
use hal::{
    gpio::{GpioPin, Input, Unknown},
    prelude::{_embedded_hal_async_digital_Wait, _esp_hal_gpio_InputPin},
};

static WIFI_EN: AtomicBool = AtomicBool::new(false);
static CHARGER_EN: AtomicBool = AtomicBool::new(false);
static PRESSURE_EN: AtomicBool = AtomicBool::new(false);
static HIGH_FREQ_EN: AtomicBool = AtomicBool::new(false);

#[derive(serde::Serialize, Clone, Copy, Debug)]
pub struct DigitalInputState {
    pub wifi_en: bool,
    pub pressure_en: bool,
    pub charger_en: bool,
    pub high_freq_en: bool
}

impl DigitalInputState {
    pub fn get() -> Self {
        Self {
            wifi_en: Self::get_wifi_en(),
            pressure_en: Self::get_pressure_en(),
            charger_en: Self::get_charger_en(),
            high_freq_en: Self::get_high_freq_en()
        }
    }

    pub fn get_wifi_en() -> bool {
        WIFI_EN.load(Ordering::Acquire)
    }

    pub fn get_charger_en() -> bool {
        CHARGER_EN.load(Ordering::Acquire)
    }

    pub fn get_pressure_en() -> bool {
        PRESSURE_EN.load(Ordering::Acquire)
    }

    pub fn get_high_freq_en() -> bool {
        HIGH_FREQ_EN.load(Ordering::Acquire)
    }
}

pub fn start_digital_input_monitor_tasks(
    spawner: &Spawner,
    wifi_en_pin: GpioPin<Unknown, 2>,
    charger_en_pin: GpioPin<Unknown, 4>,
    pressure_en_pin: GpioPin<Unknown, 16>,
    high_freq_en_pin: GpioPin<Unknown, 17>,
) {
    spawner
        .spawn(monitor_wifi_en(wifi_en_pin.into_pull_down_input()))
        .ok();
    spawner
        .spawn(monitor_charger_en(charger_en_pin.into_pull_down_input()))
        .ok();
    spawner
        .spawn(monitor_pressure_en(pressure_en_pin.into_pull_down_input()))
        .ok();
    spawner
        .spawn(monitor_high_freq_en(high_freq_en_pin.into_pull_down_input()))
        .ok();
}

#[embassy_executor::task]
async fn monitor_wifi_en(mut pin: GpioPin<Input<PullDown>, 2>) {
    loop {
        pin.wait_for_any_edge().await.unwrap();
        WIFI_EN.store(pin.is_input_high(), Ordering::Release);
    }
}

#[embassy_executor::task]
async fn monitor_charger_en(mut pin: GpioPin<Input<PullDown>, 4>) {
    loop {
        pin.wait_for_any_edge().await.unwrap();
        CHARGER_EN.store(pin.is_input_high(), Ordering::Release);
    }
}

#[embassy_executor::task]
async fn monitor_pressure_en(mut pin: GpioPin<Input<PullDown>, 16>) {
    loop {
        pin.wait_for_any_edge().await.unwrap();
        PRESSURE_EN.store(pin.is_input_high(), Ordering::Release);
    }
}

#[embassy_executor::task]
async fn monitor_high_freq_en(mut pin: GpioPin<Input<PullDown>, 17>) {
    loop {
        pin.wait_for_any_edge().await.unwrap();
        HIGH_FREQ_EN.store(pin.is_input_high(), Ordering::Release);
    }
}
