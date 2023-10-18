use crate::device_state::DeviceStateBatch;
use embassy_net::{tcp::TcpSocket, Ipv4Address, Stack};
use embassy_time::{Duration, Timer};
use esp_println::println;
use esp_wifi::wifi::WifiDevice;
use hal::{
    gpio::{GpioPin, Unknown},
    prelude::_embedded_hal_digital_v2_OutputPin,
};
use mqttrs::{decode_slice, encode_slice};

async fn connect_mqtt<'a>(socket: &mut TcpSocket<'a>) {
    let mut buffer = [0u8; 1024];
    let connect = mqttrs::Connect {
        clean_session: true,
        client_id: "esp32",
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
pub async fn publish_mqtt_task(
    stack: &'static Stack<WifiDevice<'static>>,
    led_pin: GpioPin<Unknown, 2>,
) {
    let mut led_pin = led_pin.into_push_pull_output();
    let mut rx_buffer = [0; 10000];
    let mut tx_buffer = [0; 10000];

    let mut retry: Option<DeviceStateBatch> = None;

    loop {
        led_pin.set_low().unwrap();
        loop {
            if stack.is_config_up() {
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        let address = Ipv4Address::new(54, 80, 140, 156);
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

        let mut buffer = [0u8; 10000];
        loop {
            let state_batch: DeviceStateBatch = match retry {
                Some(mut v) => {
                    // limit at 10 retrys per second
                    Timer::after(Duration::from_millis(100)).await;
                    v.uptate_time();
                    retry = None;
                    v
                }
                None => DeviceStateBatch::receive().await,
            };
            let json = serde_json_core::to_string::<DeviceStateBatch, 10000>(&state_batch).unwrap();
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
                        retry = Some(state_batch);
                        break;
                    }
                }
            };
            println!("Device state sent successfully");
        }
    }
}
