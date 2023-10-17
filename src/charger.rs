use core::sync::atomic::{AtomicU8, Ordering};
use embassy_time::Duration;
use hal::{
    gpio::{GpioPin, Unknown},
    ledc::{
        channel,
        timer::{self, config::Duty, Timer},
        LowSpeed, LEDC,
    },
    prelude::{
        _esp_hal_ledc_channel_ChannelHW, _esp_hal_ledc_channel_ChannelIFace,
        _esp_hal_ledc_timer_TimerIFace, _fugit_RateExtU32,
    },
};

use crate::i2c_adc::I2CADCRead;

const DUTY: Duty = Duty::Duty6Bit;
const TARGET_MV: u16 = 4200;
const TARGET_MA: i16 = 25;

static PWM_PCT: AtomicU8 = AtomicU8::new(0);

pub fn get_pwm_pct() -> u8 {
    PWM_PCT.load(Ordering::Acquire)
}

#[embassy_executor::task]
pub async fn charger_control_task(
    ledc: &'static LEDC<'static>,
    buck_pwm_pin: GpioPin<Unknown, 23>,
    boost_pwm_pin: GpioPin<Unknown, 19>,
) {
    let mut lstimer1: Timer<'_, LowSpeed> = ledc.get_timer::<LowSpeed>(timer::Number::Timer1);
    lstimer1
        .configure(timer::config::Config {
            duty: DUTY,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 250u32.kHz(),
        })
        .unwrap(); // fix invalid divisor

    let mut buck_pwm_channel = ledc.get_channel(
        channel::Number::Channel1,
        buck_pwm_pin.into_push_pull_output(),
    );
    buck_pwm_channel
        .configure(channel::config::Config {
            timer: &lstimer1,
            duty_pct: 100,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let mut boost_pwm_channel = ledc.get_channel(
        channel::Number::Channel2,
        boost_pwm_pin.into_push_pull_output(),
    );
    boost_pwm_channel
        .configure(channel::config::Config {
            timer: &lstimer1,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let single_pwm_limit = 2u32.pow(DUTY as u32);
    let max_dc = 3 * single_pwm_limit / 2 - 1; // 100% buck + 50% boost = 0 - 2 * Vin
    let mut pwm_dc = 0;

    loop {
        let battery_mv = I2CADCRead::get_battery_mv();
        let battery_ma = I2CADCRead::get_battery_ma();
        if battery_mv > TARGET_MV && pwm_dc > 0 {
            pwm_dc -= 1;
        } else if battery_ma < TARGET_MA && pwm_dc < max_dc {
            pwm_dc += 1;
        } else if battery_ma > TARGET_MA && pwm_dc > 0 {
            pwm_dc -= 1;
        }
        if pwm_dc > single_pwm_limit {
            buck_pwm_channel.set_duty_hw(single_pwm_limit - 1);
            boost_pwm_channel.set_duty_hw(pwm_dc - single_pwm_limit);
        } else {
            buck_pwm_channel.set_duty_hw(pwm_dc);
            boost_pwm_channel.set_duty_hw(0);
        }
        let new_pct = (pwm_dc * 100 / max_dc) as u8;
        PWM_PCT.store(new_pct, Ordering::Release);
        embassy_time::Timer::after(Duration::from_millis(100)).await;
    }
}
