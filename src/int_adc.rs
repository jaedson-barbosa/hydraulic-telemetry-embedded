use hal::{
    adc::{AdcPin, Attenuation, ADC, ADC1},
    gpio::{Analog, GpioPin},
    prelude::*,
};

pub const ATTENUATION: Attenuation = Attenuation::Attenuation11dB;

#[derive(Debug)]
pub struct IntADCRead {
    pub gpio2: u16,
    pub gpio4: u16,
}

pub struct IntADC {
    pub pin36: AdcPin<GpioPin<Analog, 36>, ADC1>,
    pub pin39: AdcPin<GpioPin<Analog, 39>, ADC1>,
    pub adc1: ADC<'static, ADC1>,
}

impl IntADC {
    fn read_pin_mv<PIN>(adc1: &mut ADC<'static, ADC1>, pin: &mut AdcPin<PIN, ADC1>) -> u32
    where
        PIN: embedded_hal::adc::Channel<ADC1, ID = u8>,
    {
        (0..4)
            .map(|_| -> u32 { nb::block!(adc1.read(pin)).unwrap() })
            .sum::<u32>()
            / 4
    }

    pub fn read_mv(&mut self) -> IntADCRead {
        let gpio2 = Self::read_pin_mv(&mut self.adc1, &mut self.pin36) as u16;
        let gpio4 = Self::read_pin_mv(&mut self.adc1, &mut self.pin39) as u16;
        IntADCRead {
            gpio2,
            gpio4,
        }
    }
}
