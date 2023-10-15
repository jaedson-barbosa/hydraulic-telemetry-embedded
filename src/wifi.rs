use crate::{digital_input::DigitalInputState, led_output::WiFiState};
use embassy_net::Stack;
use embassy_time::{Duration, Timer};
use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration, Wifi};
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiState};

#[embassy_executor::task]
pub async fn wifi_controller_task(
    mut controller: WifiController<'static>,
    ssid: &'static str,
    password: &'static str,
) {
    WiFiState::set(WiFiState::Disabled);
    println!("{ssid} e {password}");
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: ssid.into(),
        password: password.into(),
        auth_method: AuthMethod::None,
        ..Default::default()
    });
    controller.set_configuration(&client_config).unwrap();
    println!("Here");

    loop {
        Timer::after(Duration::from_secs(5)).await;
        let wifi_en = DigitalInputState::get_wifi_en();
        let wifi_state = esp_wifi::wifi::get_wifi_state();
        let connected = match wifi_state {
            WifiState::StaConnected => true,
            _ => false
        };
        println!("Here {wifi_en} {wifi_state:?}");
        if !wifi_en && connected {
            controller.stop().await.unwrap();
            println!("Wifi stopped!");
            WiFiState::set(WiFiState::Disabled);
        } else if wifi_en && !connected {
            if !matches!(controller.is_started(), Ok(true)) {
                controller.start().await.unwrap();
                println!("Wifi started!");
            }
            WiFiState::set(WiFiState::Connecting);
            match controller.connect().await {
                Ok(_) => {
                    WiFiState::set(WiFiState::Connected);
                    println!("Wifi connected!")
                },
                Err(e) => {
                    println!("Failed to connect to wifi: {e:?}");
                    WiFiState::set(WiFiState::Error);
                    Timer::after(Duration::from_secs(5)).await
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}
