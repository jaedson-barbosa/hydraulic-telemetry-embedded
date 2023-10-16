use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer, Instant};

use crate::{
    charger::ChargerState, digital_input::DigitalInputState, i2c_adc::I2CADCRead,
    int_adc::{IntADC, IntADCState}, led_output::WiFiState, pressure_boost::PressureController,
    pulse_counter::get_n_pulses
};

#[derive(serde::Serialize, Clone, Copy, Debug)]
pub struct DeviceState {
    pub charger_state: ChargerState,
    pub digital_state: DigitalInputState,
    pub i2c_adc_state: I2CADCRead,
    pub int_adc_state: IntADCState,
    pub wifi_state: WiFiState,
    pub n_pulses: u16,
    pub register_time_ms: u64,
    pub transmission_time_ms: u64
}

static STATE_CHANNEL: Channel<CriticalSectionRawMutex, DeviceState, 256> = Channel::new();

pub async fn receive_state() -> DeviceState {
    STATE_CHANNEL.receive().await
}

#[embassy_executor::task]
pub async fn state_sampling_task(
    mut int_adc: IntADC,
    mut pressure_controller: PressureController,
) {
    loop {
        let charger_state = ChargerState::receive().await;
        let digital_state = DigitalInputState::get();
        let i2c_adc_state = I2CADCRead::get();
        let int_adc_state = int_adc.get();
        let wifi_state = WiFiState::get();
        let n_pulses = get_n_pulses();
        let register_time_ms = Instant::now().as_millis();

        let device_state = DeviceState {
            charger_state,
            digital_state,
            i2c_adc_state,
            int_adc_state,
            wifi_state,
            n_pulses,
            register_time_ms,
            transmission_time_ms: 0
        };
        STATE_CHANNEL.send(device_state).await; // stop after channel is full

        let mut interval = if digital_state.high_freq_en { 1 } else { 10 };
        if !digital_state.wifi_en {
            interval *= 60; // convert from secs to minutes if WiFi is disabled
        }
        if interval > 1 {
            pressure_controller.set_enable(false);
            Timer::after(Duration::from_secs(interval - 1)).await;
            pressure_controller.set_enable(true);
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}
