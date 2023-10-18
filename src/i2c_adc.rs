use ads1x1x::{Ads1x1x, FullScaleRange, SlaveAddr};
use embedded_hal::adc::OneShot;
use hal::{i2c::I2C, peripherals::I2C0};

pub struct I2CADCReader {
    adc: ads1x1x::Ads1x1x<
        ads1x1x::interface::I2cInterface<I2C<'static, I2C0>>,
        ads1x1x::ic::Ads1115,
        ads1x1x::ic::Resolution16Bit,
        ads1x1x::mode::OneShot,
    >,
}

impl I2CADCReader {
    pub fn new(i2c: I2C<'static, I2C0>) -> Self {
        let address = SlaveAddr::default();
        let mut adc = Ads1x1x::new_ads1115(i2c, address);
        adc.set_full_scale_range(FullScaleRange::Within6_144V)
            .unwrap();
        Self { adc }
    }

    fn get_mv(val: i16) -> u16 {
        if val < 0 {
            0
        } else {
            (val as u32 * 3 / 16) as u16
        }
    }

    pub fn read(&mut self) -> I2CADCRead {
        let dif_a1_a3 = nb::block!(self.adc.read(&mut ads1x1x::channel::DifferentialA1A3)).unwrap();
        let a0 = nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA0)).unwrap();
        let a1 = nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA1)).unwrap();
        let a2 = nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA2)).unwrap();
        let a3 = nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA3)).unwrap();
        I2CADCRead {
            battery_ma: dif_a1_a3 / 16,
            battery_mv: Self::get_mv(a1),
            ldo_inp_mv: Self::get_mv(a3),
            esp_vin_mv: Self::get_mv(a2),
            pressure_mv: Self::get_mv(a0),
        }
    }
}

#[derive(serde::Serialize, Clone, Copy, Debug, Default)]
pub struct I2CADCRead {
    battery_ma: i16,
    battery_mv: u16,
    ldo_inp_mv: u16,
    esp_vin_mv: u16,
    pressure_mv: u16,
}
