use hal::{
    adc::{AdcCalCurve, AdcPin, Attenuation, ADC, ADC1},
    gpio::{Analog, GpioPin},
    prelude::*,
};
use strum::{Display, EnumIter};

pub const ATTENUATION: Attenuation = Attenuation::Attenuation11dB;

#[derive(Display, EnumIter)]
enum IntADCInput {
    GPIO2,
    GPIO4,
}

#[derive(Debug)]
pub struct IntADCRead {
    pub gpio2: u16,
    pub gpio4: u16,
}

pub struct IntADC {
    pub analog1: AdcPin<GpioPin<Analog, 2>, ADC1, AdcCalCurve<ADC1>>,
    pub analog2: AdcPin<GpioPin<Analog, 4>, ADC1, AdcCalCurve<ADC1>>,
    pub adc1: ADC<'static, ADC1>,
}

impl IntADC {
    fn read_pin_mv(&mut self, pin: &IntADCInput) -> u32 {
        let value: u32 = match pin {
            IntADCInput::GPIO2 => {
                (0..4)
                    .map(|_| -> u32 { nb::block!(self.adc1.read(&mut self.analog1)).unwrap() })
                    .sum::<u32>() / 4
            }
            IntADCInput::GPIO4 => {
                (0..4)
                    .map(|_| -> u32 { nb::block!(self.adc1.read(&mut self.analog2)).unwrap() })
                    .sum::<u32>() / 4
            }
        };
        value * ATTENUATION.ref_mv() as u32 / 4096
    }

    pub fn read_mv(&mut self) -> IntADCRead {
        IntADCRead {
            gpio2: self.read_pin_mv(&IntADCInput::GPIO2) as u16,
            gpio4: self.read_pin_mv(&IntADCInput::GPIO4) as u16,
        }
    }
}
