use hal::{
    gpio::{GpioPin, Output, PushPull, Unknown},
    // ledc::{
    //     channel::{self, ChannelIFace},
    //     timer::{self, TimerIFace}, LowSpeed, LEDC,
    // },
    prelude::*,
};

pub struct OutControl {
    charger_en_pin: GpioPin<Output<PushPull>, 6>,
    pressure_en_pin: GpioPin<Output<PushPull>, 7>,
    // channel0: channel::Channel<'a, LowSpeed, GpioPin<Output<PushPull>, 18>>,
}

impl OutControl {
    pub fn new(
        charger_pin: GpioPin<Unknown, 6>,
        pressure_en_pin: GpioPin<Unknown, 7>,
        // pressure_pwm_pin: GpioPin<Unknown, 18>,
        // lstimer0: &'a mut timer::Timer<'a, LowSpeed>,
        // ledc: &'a LEDC,
    ) -> Self {
        // lstimer0
        //     .configure(timer::config::Config {
        //         duty: timer::config::Duty::Duty5Bit,
        //         clock_source: timer::LSClockSource::APBClk,
        //         frequency: 500u32.kHz(),
        //     })
        //     .unwrap();

        // let pressure_pwm_pin = pressure_pwm_pin.into_push_pull_output();
        // let mut channel0: channel::Channel<'a, LowSpeed, GpioPin<Output<PushPull>, 18>> =
        //     ledc.get_channel(channel::Number::Channel0, pressure_pwm_pin);
        // channel0
        //     .configure(channel::config::Config {
        //         timer: lstimer0,
        //         duty_pct: 6,
        //         pin_config: channel::config::PinConfig::PushPull,
        //     })
        //     .unwrap();

        Self {
            charger_en_pin: charger_pin.into_push_pull_output(),
            pressure_en_pin: pressure_en_pin.into_push_pull_output(),
            // channel0
        }
    }

    pub fn charger_en_set(&mut self, state: bool) {
        if state {
            self.charger_en_pin.set_high().unwrap();
        } else {
            self.charger_en_pin.set_low().unwrap();
        }
    }

    pub fn pressure_en_set(&mut self, state: bool) {
        if state {
            self.pressure_en_pin.set_low().unwrap();
        } else {
            self.pressure_en_pin.set_high().unwrap();
        }
    }
}
