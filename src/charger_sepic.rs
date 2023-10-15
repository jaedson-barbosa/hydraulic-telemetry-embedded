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

pub struct ChargerSEPIC {
    target_mv: u16,
    pwm_dc: u32,
    max_dc: u32,
    pwm_channel: Channel<'static, LowSpeed, GpioPin<Output<PushPull>, 23>>,
}

impl ChargerSEPIC {
    pub fn get_timer(ledc: &'static LEDC<'static>) -> Timer<'_, LowSpeed> {
        let mut lstimer1: Timer<'_, LowSpeed> =
            ledc.get_timer::<LowSpeed>(timer::Number::Timer1);
        lstimer1
            .configure(timer::config::Config {
                duty: DUTY,
                clock_source: timer::LSClockSource::APBClk,
                frequency: 100u32.kHz(),
            })
            .unwrap();
        lstimer1
    }

    pub fn new(
        ledc: &'static LEDC<'static>,
        pwm_pin: GpioPin<Unknown, 23>,
        lstimer1: &'static Timer<'_, LowSpeed>,
    ) -> Self {
        let pwm_pin = pwm_pin.into_push_pull_output();

        let mut pwm_channel: Channel<'_, LowSpeed, GpioPin<Output<PushPull>, 23>> =
            ledc.get_channel(channel::Number::Channel1, pwm_pin);
        pwm_channel
            .configure(channel::config::Config {
                timer: lstimer1,
                duty_pct: 0,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .unwrap();

        Self {
            target_mv: 0,
            pwm_dc: 0,
            max_dc: 2u32.pow(DUTY as u32 - 1) - 1, //limit do 50%
            pwm_channel,
        }
    }

    pub fn update_pwm_dc(&mut self) {
        let battery_mv = I2CADCRead::get_battery_mv();
        if battery_mv > self.target_mv && self.pwm_dc > 0 {
            self.pwm_dc -= 1;
        } else if battery_mv < self.target_mv && self.pwm_dc < self.max_dc {
            self.pwm_dc += 1;
        } else {
            return;
        }
        self.pwm_channel.set_duty_hw(self.pwm_dc);
    }
}
