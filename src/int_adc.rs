use hal::{
    adc::{AdcConfig, AdcPin, Attenuation, ADC, ADC1},
    gpio::{Analog, GpioPin, Unknown},
    prelude::*,
};

const N_READS: u32 = 4;

#[derive(Debug)]
pub struct IntADCRead {
    pub gpio32: u16,
    pub gpio34: u16,
    pub gpio35: u16,
    pub gpio36: u16,
    pub gpio39: u16,
}

pub struct IntADC {
    pub pin32: AdcPin<GpioPin<Analog, 32>, ADC1>,
    pub pin34: AdcPin<GpioPin<Analog, 34>, ADC1>,
    pub pin35: AdcPin<GpioPin<Analog, 35>, ADC1>,
    pub pin36: AdcPin<GpioPin<Analog, 36>, ADC1>,
    pub pin39: AdcPin<GpioPin<Analog, 39>, ADC1>,
    pub adc1: ADC<'static, ADC1>,
}

impl IntADC {
    pub fn new(
        analog: ADC1,
        gpio32: GpioPin<Unknown, 32>,
        gpio34: GpioPin<Unknown, 34>,
        gpio35: GpioPin<Unknown, 35>,
        gpio36: GpioPin<Unknown, 36>,
        gpio39: GpioPin<Unknown, 39>,
    ) -> Self {
        const ATTENUATION: Attenuation = Attenuation::Attenuation11dB;
        let mut adc1_config = AdcConfig::new();
        let pin32 = adc1_config.enable_pin(gpio32.into_analog(), ATTENUATION);
        let pin34 = adc1_config.enable_pin(gpio34.into_analog(), ATTENUATION);
        let pin39 = adc1_config.enable_pin(gpio39.into_analog(), ATTENUATION);
        let pin35 = adc1_config.enable_pin(gpio35.into_analog(), ATTENUATION);
        let pin36 = adc1_config.enable_pin(gpio36.into_analog(), ATTENUATION);
        let adc1 = ADC::<ADC1>::adc(analog, adc1_config).unwrap();
        Self {
            pin32,
            pin34,
            pin35,
            pin36,
            pin39,
            adc1,
        }
    }

    fn read_pin_mv<PIN>(adc1: &mut ADC<'static, ADC1>, pin: &mut AdcPin<PIN, ADC1>) -> u32
    where
        PIN: embedded_hal::adc::Channel<ADC1, ID = u8>,
    {
        (0..N_READS)
            .map(|_| -> u32 { nb::block!(adc1.read(pin)).unwrap() })
            .sum::<u32>()
            / N_READS
    }

    pub fn read_mv(&mut self) -> IntADCRead {
        let gpio32 = Self::read_pin_mv(&mut self.adc1, &mut self.pin32) as u16;
        let gpio34 = Self::read_pin_mv(&mut self.adc1, &mut self.pin34) as u16;
        let gpio35 = Self::read_pin_mv(&mut self.adc1, &mut self.pin35) as u16;
        let gpio36 = Self::read_pin_mv(&mut self.adc1, &mut self.pin36) as u16;
        let gpio39 = Self::read_pin_mv(&mut self.adc1, &mut self.pin39) as u16;

        IntADCRead {
            gpio32,
            gpio34,
            gpio35,
            gpio36,
            gpio39,
        }
    }
}
