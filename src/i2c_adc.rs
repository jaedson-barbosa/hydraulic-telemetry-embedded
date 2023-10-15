use core::sync::atomic::{AtomicU16, Ordering};

use ads1x1x::{Ads1x1x, FullScaleRange, SlaveAddr};
use embedded_hal::adc::OneShot;
use hal::{peripherals::I2C0, i2c::I2C};

static BATTERY_MA: AtomicU16 = AtomicU16::new(0);
static BATTERY_MV: AtomicU16 = AtomicU16::new(0);
static ESP_VIN_MV: AtomicU16 = AtomicU16::new(0);
static BUCK_OUT_MV: AtomicU16 = AtomicU16::new(0);
static PRESSURE_MV: AtomicU16 = AtomicU16::new(0);

pub struct I2CADC
{
    adc: ads1x1x::Ads1x1x<
        ads1x1x::interface::I2cInterface<I2C<'static, I2C0>>,
        ads1x1x::ic::Ads1115,
        ads1x1x::ic::Resolution16Bit,
        ads1x1x::mode::OneShot,
    >,
}

#[derive(serde::Serialize, Clone, Copy, Debug)]
pub struct I2CADCRead {
    battery_ma: u16,
    battery_mv: u16,
    esp_vin_mv: u16,
    buck_out_mv: u16,
    pressure_mv: u16
}

impl I2CADCRead {
    pub fn get() -> Self {
        Self {
            battery_ma: Self::get_battery_ma(),
            battery_mv: Self::get_battery_mv(),
            esp_vin_mv: Self::get_esp_vin_mv(),
            buck_out_mv: Self::get_buck_out_mv(),
            pressure_mv: Self::get_pressure_mv()
        }
    }

    pub fn get_battery_ma() -> u16 {
        BATTERY_MA.load(Ordering::Acquire)
    }

    pub fn get_battery_mv() -> u16 {
        BATTERY_MV.load(Ordering::Acquire)
    }

    pub fn get_esp_vin_mv() -> u16 {
        ESP_VIN_MV.load(Ordering::Acquire)
    }

    pub fn get_buck_out_mv() -> u16 {
        BUCK_OUT_MV.load(Ordering::Acquire)
    }

    pub fn get_pressure_mv() -> u16 {
        PRESSURE_MV.load(Ordering::Acquire)
    }
}

impl I2CADC
{
    pub fn new(i2c: I2C<'static, I2C0>) -> Self {
        let address = SlaveAddr::default();
        let mut adc = Ads1x1x::new_ads1115(i2c, address);
        adc.set_full_scale_range(FullScaleRange::Within6_144V)
            .unwrap();
        I2CADC { adc }
    }

    fn get_mv(val: i16) -> u16 {
        if val < 0 { 0 } else { (val as u32 * 3 / 16) as u16 }
    }

    pub fn read_all_inputs(&mut self) {
        let dif_a0_a1 = nb::block!(self.adc.read(&mut ads1x1x::channel::DifferentialA0A1)).unwrap();
        BATTERY_MA.store(Self::get_mv(dif_a0_a1), Ordering::Release);
        let a0 = nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA0)).unwrap();
        BATTERY_MV.store(Self::get_mv(a0), Ordering::Release);
        let a1 = nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA1)).unwrap();
        ESP_VIN_MV.store(Self::get_mv(a1), Ordering::Release);
        let a2 = nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA2)).unwrap();
        BUCK_OUT_MV.store(Self::get_mv(a2), Ordering::Release);
        let a3 = nb::block!(self.adc.read(&mut ads1x1x::channel::SingleA3)).unwrap();
        PRESSURE_MV.store(Self::get_mv(a3), Ordering::Release);
    }
}
