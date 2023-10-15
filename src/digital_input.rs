use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::Spawner;
use hal::{
    gpio::{GpioPin, Input, PullUp, Unknown},
    prelude::{_embedded_hal_async_digital_Wait, _esp_hal_gpio_InputPin},
};

static WIFI_EN: AtomicBool = AtomicBool::new(false);
static CHARGER_EN: AtomicBool = AtomicBool::new(false);
static INTERVAL_PIN_0: AtomicBool = AtomicBool::new(false);
static INTERVAL_PIN_1: AtomicBool = AtomicBool::new(false);

pub enum SamplingInterval {
    Sec1,
    Sec10,
    Min1,
    Min10,
}

pub struct DigitalInputRead {
    wifi_en: bool,
    charger_en: bool,
    sampling_interval: SamplingInterval
}

impl DigitalInputRead {
    pub fn get() -> Self {
        Self {
            wifi_en: Self::get_wifi_en(),
            charger_en: Self::get_charger_en(),
            sampling_interval: Self::get_sampling_interval()
        }
    }

    pub fn get_wifi_en() -> bool {
        WIFI_EN.load(Ordering::Acquire)
    }

    pub fn get_charger_en() -> bool {
        CHARGER_EN.load(Ordering::Acquire)
    }

    pub fn get_sampling_interval() -> SamplingInterval {
        let pin_0_high = INTERVAL_PIN_0.load(Ordering::Acquire);
        let pin_1_high = INTERVAL_PIN_1.load(Ordering::Acquire);
        match (pin_0_high, pin_1_high) {
            (false, false) => SamplingInterval::Sec1,
            (true, false) => SamplingInterval::Min1,
            (false, true) => SamplingInterval::Sec10,
            (true, true) => SamplingInterval::Min10,
        }
    }
}

pub fn start_digital_input_monitor_tasks(
    spawner: &Spawner,
    wifi_en_pin: GpioPin<Unknown, 2>,
    charger_en_pin: GpioPin<Unknown, 4>,
    interval_pin_0: GpioPin<Unknown, 16>,
    interval_pin_1: GpioPin<Unknown, 17>,
) {
    spawner
        .spawn(monitor_wifi_en(wifi_en_pin.into_pull_up_input()))
        .ok();
    spawner
        .spawn(monitor_charger_en(charger_en_pin.into_pull_up_input()))
        .ok();
    spawner
        .spawn(monitor_interval_0(interval_pin_0.into_pull_up_input()))
        .ok();
    spawner
        .spawn(monitor_interval_1(interval_pin_1.into_pull_up_input()))
        .ok();
}

#[embassy_executor::task]
async fn monitor_wifi_en(mut pin: GpioPin<Input<PullUp>, 2>) {
    loop {
        pin.wait_for_any_edge().await.unwrap();
        WIFI_EN.store(pin.is_input_high(), Ordering::Release);
    }
}

#[embassy_executor::task]
async fn monitor_charger_en(mut pin: GpioPin<Input<PullUp>, 4>) {
    loop {
        pin.wait_for_any_edge().await.unwrap();
        CHARGER_EN.store(pin.is_input_high(), Ordering::Release);
    }
}

#[embassy_executor::task]
async fn monitor_interval_0(mut pin: GpioPin<Input<PullUp>, 16>) {
    loop {
        pin.wait_for_any_edge().await.unwrap();
        INTERVAL_PIN_0.store(pin.is_input_high(), Ordering::Release);
    }
}

#[embassy_executor::task]
async fn monitor_interval_1(mut pin: GpioPin<Input<PullUp>, 17>) {
    loop {
        pin.wait_for_any_edge().await.unwrap();
        INTERVAL_PIN_1.store(pin.is_input_high(), Ordering::Release);
    }
}
