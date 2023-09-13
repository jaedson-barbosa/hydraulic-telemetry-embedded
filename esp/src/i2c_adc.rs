use ads1x1x::{Ads1x1x, FullScaleRange, SlaveAddr};
use hal::{
    i2c::I2C,
    peripherals::I2C0,
    prelude::*,
};
use strum::{Display, EnumIter};

type Adc = ads1x1x::Ads1x1x<
    ads1x1x::interface::I2cInterface<I2C<'static, I2C0>>,
    ads1x1x::ic::Ads1115,
    ads1x1x::ic::Resolution16Bit,
    ads1x1x::mode::OneShot,
>;

#[derive(Display, EnumIter)]
pub enum AnalogInput {
    Pressure,
    Generator,
    Battery,
    // RegulatorOutput,
}

#[derive(Debug)]
pub struct I2CADCRead {
    pub pressure: f32,
    pub generator: f32,
    pub battery: f32,
    // pub regulator_output: f32
}

pub struct I2CADC {
    adc: Adc,
}

impl I2CADC {
    pub fn new(i2c: I2C<'static, I2C0>) -> Self {
        let address = SlaveAddr::default();
        let mut adc: ads1x1x::Ads1x1x<
            ads1x1x::interface::I2cInterface<I2C<'_, I2C0>>,
            ads1x1x::ic::Ads1115,
            ads1x1x::ic::Resolution16Bit,
            ads1x1x::mode::OneShot,
        > = Ads1x1x::new_ads1115(i2c, address);
        adc.set_full_scale_range(FullScaleRange::Within6_144V)
            .unwrap();
        I2CADC { adc }
    }

    pub fn read_single(&mut self, pin: AnalogInput) -> f32 {
        let value = match pin {
            AnalogInput::Pressure => nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA0)).unwrap(),
            AnalogInput::Generator => nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA3)).unwrap(),
            AnalogInput::Battery => nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA2)).unwrap(),
            // AnalogInput::RegulatorOutput => nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA3)).unwrap(),
        };
        // println!("analog read: {value} from pin {pin}");
        let float_value = (value as f32 * 0.1875) / 1000.0;
        float_value
    }

    pub fn read_all(&mut self) -> I2CADCRead {
        I2CADCRead {
            pressure: self.read_single(AnalogInput::Pressure),
            generator: self.read_single(AnalogInput::Generator),
            battery: self.read_single(AnalogInput::Battery),
            // regulator_output: self.read_single(AnalogInput::RegulatorOutput)
        }
    }
}
