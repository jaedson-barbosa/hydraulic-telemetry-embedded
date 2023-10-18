use hal::{gpio::{GpioPin, Output, PushPull}, prelude::_embedded_hal_digital_v2_OutputPin};

use crate::device_state::BATTERY_MV;

#[embassy_executor::task]
pub async fn charger_task(mut charger_en_pin: GpioPin<Output<PushPull>, 13>) {
    loop {
        let battery_mv = BATTERY_MV.wait().await;
        if battery_mv > 4200 {
            charger_en_pin.set_low().unwrap();
        } else {
            charger_en_pin.set_high().unwrap();
        }
    }
}