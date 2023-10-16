use core::sync::atomic::{AtomicU16, Ordering, AtomicI16};

use ads1x1x::{Ads1x1x, FullScaleRange, SlaveAddr};
use embassy_time::Duration;
use embedded_hal::adc::OneShot;
use hal::{peripherals::I2C0, i2c::I2C};

static BATTERY_MA: AtomicI16 = AtomicI16::new(0);
static BATTERY_MV: AtomicU16 = AtomicU16::new(0);
static ESP_VIN_MV: AtomicU16 = AtomicU16::new(0);
static BUCK_OUT_MV: AtomicU16 = AtomicU16::new(0);
static PRESSURE_MV: AtomicU16 = AtomicU16::new(0);

#[derive(serde::Serialize, Clone, Copy, Debug)]
pub struct I2CADCRead {
    battery_ma: i16,
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

    pub fn get_battery_ma() -> i16 {
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

#[embassy_executor::task]
pub async fn monitor_i2c_adc_task(i2c: I2C<'static, I2C0>) {
    let address = SlaveAddr::default();
    let mut adc = Ads1x1x::new_ads1115(i2c, address);
    adc.set_full_scale_range(FullScaleRange::Within6_144V).unwrap();

    loop {
        let dif_a0_a1 = nb::block!(adc.read(&mut ads1x1x::channel::DifferentialA0A1)).unwrap();
        BATTERY_MA.store(dif_a0_a1 / 16, Ordering::Release);
        let a0 = nb::block!(adc.read(&mut ads1x1x::channel::SingleA0)).unwrap();
        ESP_VIN_MV.store(get_mv(a0), Ordering::Release);
        let a1 = nb::block!(adc.read(&mut ads1x1x::channel::SingleA1)).unwrap();
        BATTERY_MV.store(get_mv(a1), Ordering::Release);
        let a2 = nb::block!(adc.read(&mut ads1x1x::channel::SingleA2)).unwrap();
        BUCK_OUT_MV.store(get_mv(a2), Ordering::Release);
        let a3 = nb::block!(adc.read(&mut ads1x1x::channel::SingleA3)).unwrap();
        PRESSURE_MV.store(get_mv(a3), Ordering::Release);

        embassy_time::Timer::after(Duration::from_millis(100)).await;
    }
}

fn get_mv(val: i16) -> u16 {
    if val < 0 { 0 } else { (val as u32 * 3 / 16) as u16 }
}
