use embassy_net::Stack;
use embassy_time::{Duration, Timer};
use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration, Wifi};
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiState, WifiEvent};

#[embassy_executor::task]
pub async fn wifi_controller_task(
    mut controller: WifiController<'static>,
    ssid: &'static str,
    password: &'static str,
) {
    println!("{ssid} e {password}");
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: ssid.into(),
        password: password.into(),
        auth_method: AuthMethod::None,
        ..Default::default()
    });
    controller.set_configuration(&client_config).unwrap();
    println!("Starting wifi");
    controller.start().await.unwrap();
    println!("Wifi started!");
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_secs(5)).await
            }
            _ => {}
        }
        println!("About to connect...");
        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_secs(5)).await
            }
        }
    }
}

#[embassy_executor::task]
pub async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}
