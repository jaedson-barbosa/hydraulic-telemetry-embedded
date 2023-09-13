use hal::{
    adc::{AdcCalCurve, AdcPin, Attenuation, ADC, ADC1},
    gpio::{Analog, GpioPin},
    prelude::*,
};
use strum::{Display, EnumIter};

pub const ATTENUATION: Attenuation = Attenuation::Attenuation11dB;

#[derive(Display, EnumIter)]
pub enum IntADCInput {
    GPIO2,
    GPIO4
}

pub struct IntADC {
    pub analog1: AdcPin<GpioPin<Analog, 2>, ADC1, AdcCalCurve<ADC1>>,
    pub analog2: AdcPin<GpioPin<Analog, 4>, ADC1, AdcCalCurve<ADC1>>,
    pub adc1: ADC<'static, ADC1>,
}

impl IntADC {
    pub fn read_mv(&mut self, pin: &IntADCInput) -> u32 {
        let value: u32 = match pin {
            IntADCInput::GPIO2 => nb::block!(self.adc1.read(&mut self.analog1)).unwrap(),
            IntADCInput::GPIO4 => nb::block!(self.adc1.read(&mut self.analog2)).unwrap(),
        };
        value * ATTENUATION.ref_mv() as u32 / 4096
    }
}
