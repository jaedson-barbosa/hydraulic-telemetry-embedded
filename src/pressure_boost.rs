use hal::{
    gpio::{GpioPin, Output, PushPull, Unknown},
    ledc::{channel, timer, LowSpeed, LEDC},
    prelude::{
        _embedded_hal_digital_v2_OutputPin, _esp_hal_ledc_channel_ChannelIFace,
        _esp_hal_ledc_timer_TimerIFace, _fugit_RateExtU32,
    },
};

pub struct PressureController {
    enable_pin: GpioPin<Output<PushPull>, 13>,
}

impl PressureController {
    pub fn new(
        ledc: &'static LEDC<'static>,
        pwm_pin: GpioPin<Unknown, 12>,
        enable_pin: GpioPin<Unknown, 13>,
    ) -> Self {
        let pwm_pin = pwm_pin.into_push_pull_output();

        let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer2);
        lstimer0
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty5Bit,
                clock_source: timer::LSClockSource::APBClk,
                frequency: 500u32.kHz(),
            })
            .unwrap();

        let mut channel0 = ledc.get_channel(channel::Number::Channel0, pwm_pin);
        channel0
            .configure(channel::config::Config {
                timer: &lstimer0,
                duty_pct: 30,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .unwrap();

        let enable_pin = enable_pin.into_push_pull_output();
        Self { enable_pin }
    }

    pub fn set_enable(&mut self, value: bool) {
        match value {
            true => self.enable_pin.set_low().unwrap(),
            false => self.enable_pin.set_high().unwrap(),
        }
    }
}
