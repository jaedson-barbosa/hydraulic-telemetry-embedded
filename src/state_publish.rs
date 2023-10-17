use embassy_net::{
    // dns::DnsQueryType,
    tcp::TcpSocket,
    Ipv4Address,
    Stack,
};
use embassy_time::{Duration, Instant, Timer};
use hal::{gpio::{GpioPin, Unknown}, prelude::_embedded_hal_digital_v2_OutputPin};
use esp_println::println;
use esp_wifi::wifi::WifiDevice;
use mqttrs::{decode_slice, encode_slice};

use crate::{i2c_adc::I2CADCRead, charger::get_pwm_pct, pulse_counter::get_n_pulses, pressure_boost::PressureController};

#[derive(serde::Serialize, Clone, Copy, Debug)]
pub struct DeviceState {
    pub adc_state: I2CADCRead,
    pub pwm_pct: u8,
    pub n_pulses: u16,
    pub time_ms: u64,
}

impl DeviceState {
    pub fn current() -> Self {
        DeviceState {
            adc_state: I2CADCRead::get(),
            pwm_pct: get_pwm_pct(),
            n_pulses: get_n_pulses(),
            time_ms: Instant::now().as_millis(),
        }
    }
}

async fn connect_mqtt<'a>(socket: &mut TcpSocket<'a>) {
    let mut buffer = [0u8; 1024];
    let connect = mqttrs::Connect {
        clean_session: true,
        client_id: "esp32c3-for-test",
        keep_alive: 120,
        last_will: None,
        password: None,
        username: None,
        protocol: mqttrs::Protocol::MQTT311,
    };
    let size = encode_slice(&connect.into(), &mut buffer).unwrap();
    socket.write(&buffer[..size]).await.unwrap();
    socket.flush().await.unwrap();
    let size = socket.read(&mut buffer).await.unwrap();
    let decoded = decode_slice(&buffer[..size]).unwrap().unwrap();
    println!("{decoded:?}");
}

#[embassy_executor::task]
pub async fn publish_mqtt_task(stack: &'static Stack<WifiDevice<'static>>, led_pin: GpioPin<Unknown, 2>, mut pressure_controller: PressureController) {
    pressure_controller.set_enable(true);

    let mut led_pin = led_pin.into_push_pull_output();
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    loop {
        led_pin.set_low().unwrap();
        loop {
            if stack.is_link_up() {
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        println!("Waiting to get IP address...");

        loop {
            if let Some(config) = stack.config_v4() {
                println!("Got IP: {}", config.address);
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        Timer::after(Duration::from_millis(1_000)).await;
        let address = Ipv4Address::new(54, 80, 140, 156); // start broker and fix ip
        let mut socket = TcpSocket::new(&stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_keep_alive(Some(Duration::from_secs(10)));
        socket.set_timeout(Some(embassy_time::Duration::from_secs(30)));

        let remote_endpoint = (address, 1883);
        while let Err(v) = socket.connect(remote_endpoint).await {
            println!("Error while trying to connect to socket: {v:?}");
            Timer::after(Duration::from_millis(500)).await;
        }
        println!("Connected to remote socket");

        connect_mqtt(&mut socket).await;
        println!("Connected to MQTT broker");

        led_pin.set_high().unwrap();

        let mut buffer = [0u8; 1024];
        loop {
            let state = DeviceState::current();
            let json =  serde_json_core::to_string::<DeviceState, 1024>(&state).unwrap();
            let publish = mqttrs::Publish {
                dup: false,
                payload: json.as_bytes(),
                qospid: mqttrs::QosPid::AtMostOnce,
                retain: false,
                topic_name: "jaedson/tocloud",
            };
            let size = encode_slice(&publish.into(), &mut buffer).unwrap();
            if let Err(v) = socket.write(&buffer[..size]).await {
                match v {
                    embassy_net::tcp::Error::ConnectionReset => {
                        println!("Connection reset while writting to buffer");
                        break;
                    }
                }
            };
            println!("Device state sent successfully");
            Timer::after(Duration::from_millis(1000)).await;
        }
    }
}
