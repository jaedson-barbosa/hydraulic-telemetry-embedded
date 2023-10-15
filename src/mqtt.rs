use embassy_net::{
    // dns::DnsQueryType,
    tcp::TcpSocket,
    Ipv4Address,
    Stack
};
use embassy_time::{Timer, Duration, Ticker, Instant};
use esp_println::println;
use esp_wifi::wifi::WifiDevice;
use mqttrs::{decode_slice, encode_slice};

use crate::device_state::{DeviceState, receive_state};

async fn connect_mqtt<'a>(socket: &mut TcpSocket<'a>) {
    let mut buffer = [0u8; 1024];
    let connect = mqttrs::Connect {
        clean_session: false,
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
pub async fn publish_mqtt_task(stack: &'static Stack<WifiDevice<'static>>) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    loop {
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
        // let address: embassy_net::IpAddress = stack
        //     .dns_query("12345678", DnsQueryType::A)
        //     .await
        //     .unwrap()[0];
        let address = Ipv4Address::new(54, 80, 140, 156); // start broker and fix ip
        let mut socket = TcpSocket::new(&stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        let remote_endpoint = (address, 1883);
        while let Err(v) = socket.connect(remote_endpoint).await {
            println!("Error while trying to connect to socket: {v:?}");
            Timer::after(Duration::from_millis(500)).await;
        }
        println!("Connected to remote socket");

        connect_mqtt(&mut socket).await;

        let mut buffer = [0u8; 1024];
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            ticker.next().await;
            let json = {
                let mut state = receive_state().await;
                state.transmission_time_ms = Instant::now().as_millis();
                serde_json_core::to_string::<DeviceState, 256>(&state).unwrap()
            };
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
                        break;
                    }
                }
            };
            if let Err(v) = socket.flush().await {
                match v {
                    embassy_net::tcp::Error::ConnectionReset => {
                        break;
                    }
                }
            };
        }
    }
}
