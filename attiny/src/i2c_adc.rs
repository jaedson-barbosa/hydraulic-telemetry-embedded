use ads1x1x::{Ads1x1x, FullScaleRange, SlaveAddr};
use embedded_hal::prelude::*;
use strum::{Display, EnumIter};

use crate::i2c::I2C;

type Adc = ads1x1x::Ads1x1x<
    ads1x1x::interface::I2cInterface<I2C>,
    ads1x1x::ic::Ads1115,
    ads1x1x::ic::Resolution16Bit,
    ads1x1x::mode::OneShot,
>;

#[derive(Display, EnumIter)]
#[repr(u8)]
pub enum AnalogInput {
    A0,
    A1,
    A2,
    A3
}

#[derive(Debug)]
pub struct I2CADCRead {
    pub a0: u16,
    pub a1: u16,
    pub a2: u16,
    pub a3: u16,
}

pub struct I2CADC {
    adc: Adc,
}

impl I2CADC {
    pub fn new(i2c: I2C) -> Self {
        let address = SlaveAddr::default();
        let mut adc: ads1x1x::Ads1x1x<
            ads1x1x::interface::I2cInterface<I2C>,
            ads1x1x::ic::Ads1115,
            ads1x1x::ic::Resolution16Bit,
            ads1x1x::mode::OneShot,
        > = Ads1x1x::new_ads1115(i2c, address);
        adc.set_full_scale_range(FullScaleRange::Within6_144V)
            .unwrap();
        I2CADC { adc }
    }

    pub fn read_single_mv(&mut self, pin: AnalogInput) -> u16 {
        let mut value = match pin {
            AnalogInput::A0 => nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA0)).unwrap(),
            AnalogInput::A1 => nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA1)).unwrap(),
            AnalogInput::A2 => nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA2)).unwrap(),
            AnalogInput::A3 => nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA3)).unwrap(),
        } as i32;
        if value < 0 {
            value = 0;
        }
        let float_value = value * 3 / 16;
        float_value as u16
    }

    pub fn read_all_mv(&mut self) -> I2CADCRead {
        I2CADCRead {
            a0: self.read_single_mv(AnalogInput::A0),
            a1: self.read_single_mv(AnalogInput::A1),
            a2: self.read_single_mv(AnalogInput::A2),
            a3: self.read_single_mv(AnalogInput::A3),
        }
    }

    pub fn destroy(self) -> I2C {
        self.adc.destroy_ads1115()
    }
}
