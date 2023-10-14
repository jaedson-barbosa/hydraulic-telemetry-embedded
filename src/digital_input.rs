use hal::{gpio::{GpioPin, Input, PullUp, Unknown}, prelude::_esp_hal_gpio_InputPin};

pub struct WiFiEnInput {
    pin: GpioPin<Input<PullUp>, 2>
}

impl WiFiEnInput {
    pub fn new(gpio2: GpioPin<Unknown, 2>) -> Self {
        Self { pin: gpio2.into_pull_up_input() }
    }

    pub fn get_wifi_en(&self) -> bool {
        self.pin.is_input_high()
    }
}

pub struct ChargerEnInput {
    pin: GpioPin<Input<PullUp>, 4>
}

impl ChargerEnInput {
    pub fn new(gpio4: GpioPin<Unknown, 4>) -> Self {
        Self { pin: gpio4.into_pull_up_input() }
    }

    pub fn get_charger_en(&self) -> bool {
        self.pin.is_input_high()
    }
}

pub enum TransmissionInterval {
    Sec1,
    Sec10,
    Min1,
    Min10
}

pub struct TransmissionIntervalInput {
    pin_0: GpioPin<Input<PullUp>, 16>,
    pin_1: GpioPin<Input<PullUp>, 17>,
}

impl TransmissionIntervalInput {
    pub fn new(gpio16: GpioPin<Unknown, 16>, gpio17: GpioPin<Unknown, 17>) -> Self {
        Self {
            pin_0: gpio16.into_pull_up_input(),
            pin_1: gpio17.into_pull_up_input()
        }
    }

    pub fn get_transmission_interval(&self) -> TransmissionInterval {
        let pin_0_high = self.pin_0.is_input_high();
        let pin_1_high = self.pin_1.is_input_high();
        match (pin_0_high, pin_1_high) {
            (false, false) => TransmissionInterval::Sec1,
            (true, false) => TransmissionInterval::Min1,
            (false, true) => TransmissionInterval::Sec10,
            (true, true) => TransmissionInterval::Min10
        }
    }
}
