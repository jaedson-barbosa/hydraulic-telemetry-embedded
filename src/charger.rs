use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Duration;
use hal::{
    gpio::{GpioPin, Output, PushPull, Unknown},
    ledc::{
        channel::{self, Channel},
        timer::{self, config::Duty, Timer},
        LowSpeed, LEDC,
    },
    prelude::{
        _esp_hal_ledc_channel_ChannelHW, _esp_hal_ledc_channel_ChannelIFace,
        _esp_hal_ledc_timer_TimerIFace, _fugit_RateExtU32,
    },
};

use crate::i2c_adc::I2CADCRead;

const DUTY: Duty = Duty::Duty10Bit;
const TARGET_MV: u16 = 4100;
const TARGET_MA: i16 = 50;

#[derive(serde::Serialize, Clone, Copy, Debug)]
pub struct ChargerState {
    pwm_dc: u32,
    max_dc: u32,
}

impl ChargerState {
    pub async fn receive() -> ChargerState {
        CHARGER_STATE.wait().await
    }
}

static CHARGER_STATE: Signal<CriticalSectionRawMutex, ChargerState> = Signal::new();

#[embassy_executor::task]
pub async fn charger_control_task(ledc: &'static LEDC<'static>, pwm_pin: GpioPin<Unknown, 23>) {
    let mut lstimer1: Timer<'_, LowSpeed> = ledc.get_timer::<LowSpeed>(timer::Number::Timer1);
    lstimer1
        .configure(timer::config::Config {
            duty: DUTY,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 64u32.kHz(),
        })
        .unwrap(); // fix invalid divisor

    let pwm_pin = pwm_pin.into_push_pull_output();
    let mut pwm_channel: Channel<'_, LowSpeed, GpioPin<Output<PushPull>, 23>> =
        ledc.get_channel(channel::Number::Channel1, pwm_pin);
    pwm_channel
        .configure(channel::config::Config {
            timer: &lstimer1,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let mut state = ChargerState {
        pwm_dc: 0,
        max_dc: 2u32.pow(DUTY as u32 - 1), //limit do 50%
    };

    loop {
        let battery_mv = I2CADCRead::get_battery_mv();
        let battery_ma = I2CADCRead::get_battery_ma();
        if battery_mv > TARGET_MV && state.pwm_dc > 0 {
            state.pwm_dc -= 1;
        } else if battery_ma < TARGET_MA && state.pwm_dc < state.max_dc {
            state.pwm_dc += 1;
        } else if battery_ma > TARGET_MA && state.pwm_dc > 0 {
            state.pwm_dc -= 1;
        }
        pwm_channel.set_duty_hw(state.pwm_dc);
        CHARGER_STATE.signal(state);

        embassy_time::Timer::after(Duration::from_millis(100)).await;
    }
}
