use core::convert::Infallible;

use ds18b20::Ds18b20;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use esp_println::println;
use one_wire_bus::{OneWire, OneWireResult};

pub struct Temperature<P>
where
    P: InputPin<Error = Infallible> + OutputPin<Error = Infallible>,
{
    one_wire_bus: OneWire<P>,
    delay: hal::Delay,
    sensor: Ds18b20,
}

impl<P> Temperature<P>
where
    P: InputPin<Error = Infallible> + OutputPin<Error = Infallible>,
{
    pub fn new(one_wire_pin: P, mut delay: hal::Delay) -> OneWireResult<Self, Infallible> {
        let mut one_wire_bus = one_wire_bus::OneWire::new(one_wire_pin)?;
        // ds18b20::start_simultaneous_temp_measurement(&mut one_wire_bus, &mut delay)?;
        // Resolution::Bits12.delay_for_measurement_time(&mut delay);
        let mut search_state = None;
        loop {
            if let Some((device_address, state)) =
                one_wire_bus.device_search(search_state.as_ref(), false, &mut delay)?
            {
                search_state = Some(state);
                if device_address.family_code() != ds18b20::FAMILY_CODE {
                    // skip other devices
                    continue;
                }
                // You will generally create the sensor once, and save it for later
                let sensor = Ds18b20::new(device_address)?;

                // contains the read temperature, as well as config info such as the resolution used
                let sensor_data = sensor.read_data(&mut one_wire_bus, &mut delay)?;
                println!(
                    "Device at {:?} is {}°C",
                    device_address, sensor_data.temperature
                );
                return Ok(Self {
                    one_wire_bus,
                    delay,
                    sensor,
                });
            } else {
                break;
            }
        }
        Err(one_wire_bus::OneWireError::Timeout)
    }

    pub fn read(&mut self) -> OneWireResult<f32, Infallible> {
        let result = self.sensor.read_data(&mut self.one_wire_bus, &mut self.delay)?;
        Ok(result.temperature)
    }
}
