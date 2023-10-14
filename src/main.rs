#![no_std]
#![no_main]
#![feature(cell_update)]
#![feature(future_join)]
#![feature(type_alias_impl_trait)]

// read about async Rust in https://rust-lang.github.io/async-book/01_getting_started/01_chapter.html
// hal code examples in https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
// wifi code examples in https://github.com/esp-rs/esp-wifi/blob/main/examples-esp32c3
// board repo in https://github.com/Xinyuan-LilyGO/LilyGo-T-OI-PLUS

mod int_adc;

use dotenvy_macro::dotenv;
use embassy_executor::{Executor, Spawner, _export::StaticCell};
use embassy_net::{
    // dns::DnsQueryType,
    tcp::TcpSocket,
    Config,
    Ipv4Address,
    Stack,
    StackResources,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration, Wifi};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{
    initialize,
    wifi::{WifiController, WifiDevice, WifiEvent, WifiMode, WifiState},
    EspWifiInitFor,
};
use hal::{
    clock::ClockControl,
    embassy,
    gpio::{GpioPin, Output, PushPull, IO},
    // i2c::I2C,
    interrupt,
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, LowSpeed, LEDC,
    },
    peripherals::{self, Peripherals},
    prelude::*,
    timer::TimerGroup,
    Rng,
};
use int_adc::IntADC;
use mqttrs::{decode_slice, encode_slice};

const SSID: &str = dotenv!("SSID");
const PASSWORD: &str = dotenv!("PASSWORD");
const PRESSURE_EN: bool = true;

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

#[derive(serde::Serialize, Clone, Copy, Debug, Default)]
struct DeviceState {
    n_pulses: u16,
    generator_mv: u16,
    battery_mv: u16,
    pressure_mv: u16,
    pressure_en: bool,
    buck_dc: u8,
    buck_target: u16,
}

static STATE: Mutex<CriticalSectionRawMutex, DeviceState> = Mutex::new(DeviceState {
    battery_mv: 0,
    buck_dc: 0,
    generator_mv: 0,
    n_pulses: 0,
    pressure_en: PRESSURE_EN,
    pressure_mv: 0,
    buck_target: 3900,
});

#[entry]
fn entry() -> ! {
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(main(spawner)).ok();
    });
}

// async fn read_attiny_state<I2C>(i2c: &mut I2C) -> AttinyResponse
// where
//     I2C: embedded_hal_async::i2c::I2c,
// {
//     let mut buffer = [0u8; 3];
//     loop {
//         let response = i2c.read(8, &mut buffer).await;
//         match response {
//             Ok(_) => {
//                 let data = AttinyResponse::deserialize_from(&buffer);
//                 println!("Read data: {data:?} from buffer: {buffer:?}");
//                 break data;
//             }
//             Err(v) => println!("Error while receiving i2c data: {v:?}"),
//         }
//         embassy_time::Timer::after(Duration::from_secs(1)).await;
//     }
// }

// #[embassy_executor::task]
// async fn i2c_controller(mut i2c: I2C<'static, I2C0>) {
//     loop {
//         embassy_time::Timer::after(Duration::from_secs(1)).await;
//         println!("Reading ATTINY state...");
//         let attiny_state = read_attiny_state(&mut i2c).await;
//         let mut state = STATE.lock().await;
//         state.n_pulses += 1;
//         state.generator_mv = attiny_state.generator_mv;
//     }
// }

#[embassy_executor::task]
async fn pressure_monitor(mut pressure_en_pin: GpioPin<Output<PushPull>, 18>) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    pressure_en_pin.set_high().unwrap();
    loop {
        ticker.next().await;
        let state = STATE.lock().await;
        if state.pressure_en {
            pressure_en_pin.set_low().unwrap(); // enable on low state
        } else {
            pressure_en_pin.set_high().unwrap();
        }
    }
}

#[embassy_executor::task]
async fn int_adc_monitor(mut int_adc: IntADC) {
    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        ticker.next().await;
        let read = int_adc.read_mv();
        let mut state = STATE.lock().await;
        state.pressure_mv = read.gpio36;
        state.battery_mv = read.gpio39 * 2;
    }
}

#[embassy_executor::task]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();

    let system = peripherals.DPORT.split();
    let clocks = &*singleton!(ClockControl::boot_defaults(system.clock_control).freeze());
    let mut peripheral_clock_control = system.peripheral_clock_control;

    let timer =
        hal::timer::TimerGroup::new(peripherals.TIMG1, &clocks, &mut peripheral_clock_control)
            .timer0;
    let rng = Rng::new(peripherals.RNG);
    let radio_control = system.radio_clock_control;
    let init = initialize(EspWifiInitFor::Wifi, timer, rng, radio_control, &clocks).unwrap();

    let (wifi, _) = peripherals.RADIO.split();
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiMode::Sta).unwrap();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks, &mut peripheral_clock_control);
    embassy::init(&clocks, timer_group0.timer0);

    let config = Config::dhcpv4(Default::default());

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let stack = &*singleton!(Stack::new(
        wifi_interface,
        config,
        singleton!(StackResources::<3>::new()),
        seed
    ));

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let internal_adc = IntADC::new(
        peripherals.SENS.split().adc1,
        io.pins.gpio32,
        io.pins.gpio34,
        io.pins.gpio35,
        io.pins.gpio36,
        io.pins.gpio39,
    );
    spawner.spawn(int_adc_monitor(internal_adc)).ok();

    spawner
        .spawn(pressure_monitor(io.pins.gpio18.into_push_pull_output()))
        .ok();
    let mut led = io.pins.gpio3.into_push_pull_output();

    led.set_high().unwrap();
    let pulse_pin: GpioPin<hal::gpio::Input<hal::gpio::PullDown>, 10> =
        io.pins.gpio10.into_pull_down_input();
    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority1).unwrap();
    interrupt::enable(
        peripherals::Interrupt::I2C_EXT0,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    // let i2c = I2C::new(
    //     peripherals.I2C0,
    //     io.pins.gpio6,
    //     io.pins.gpio7,
    //     32u32.kHz(),
    //     &mut peripheral_clock_control,
    //     &clocks,
    // );
    // spawner.spawn(i2c_controller(i2c)).ok();

    let mut ledc = LEDC::new(peripherals.LEDC, &clocks, &mut peripheral_clock_control);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer2);

    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 500u32.kHz(),
        })
        .unwrap();

    let pressure_pwm_pin: GpioPin<Output<PushPull>, 5> = io.pins.gpio5.into_push_pull_output();
    let mut channel0 = ledc.get_channel(channel::Number::Channel0, pressure_pwm_pin);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 6,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    channel0.set_duty(50).unwrap();

    let buck_pwm_pin: GpioPin<Output<PushPull>, 19> = io.pins.gpio19.into_push_pull_output();

    spawner.spawn(pulse_counter(pulse_pin)).ok();
    spawner.spawn(wifi_controller_task(controller)).ok();
    spawner.spawn(net_task(&stack)).ok();
    spawner.spawn(publish_mqtt_task(&stack)).ok();
    spawner
        .spawn(buck_pwm_controller_task(ledc, buck_pwm_pin))
        .ok();
}

#[embassy_executor::task]
async fn pulse_counter(mut pin: GpioPin<hal::gpio::Input<hal::gpio::PullDown>, 10>) {
    loop {
        println!("waiting pulse...");
        pin.wait_for_rising_edge().await.unwrap();
        Timer::after(Duration::from_millis(10)).await;
        let mut state = STATE.lock().await;
        state.n_pulses += 1;
    }
}

#[embassy_executor::task]
async fn buck_pwm_controller_task(
    ledc: LEDC<'static>,
    buck_pwm_pin: GpioPin<Output<PushPull>, 19>,
) {
    let mut lstimer1 = ledc.get_timer::<LowSpeed>(timer::Number::Timer1);
    lstimer1
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 100u32.kHz(),
        })
        .unwrap();

    let mut buck_pwm_channel = ledc.get_channel(channel::Number::Channel1, buck_pwm_pin);
    buck_pwm_channel
        .configure(channel::config::Config {
            timer: &lstimer1,
            duty_pct: 100,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        ticker.next().await;
        let mut state = STATE.lock().await;
        if state.battery_mv > state.buck_target && state.buck_dc > 0 {
            state.buck_dc -= 1;
        } else if state.battery_mv < state.buck_target && state.buck_dc < 100 {
            state.buck_dc += 1;
        }
        buck_pwm_channel.set_duty(state.buck_dc).unwrap();
    }
}

#[embassy_executor::task]
async fn wifi_controller_task(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_secs(5)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            println!("{SSID} e {PASSWORD}");
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.into(),
                password: PASSWORD.into(),
                auth_method: AuthMethod::None,
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start().await.unwrap();
            println!("Wifi started!");
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
async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}

#[embassy_executor::task]
async fn publish_mqtt_task(stack: &'static Stack<WifiDevice<'static>>) {
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
                let mut state = STATE.lock().await;
                let json = serde_json_core::to_string::<DeviceState, 256>(&state).unwrap();
                state.n_pulses = 0;
                json
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
