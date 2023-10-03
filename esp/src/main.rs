#![no_std]
#![no_main]
#![feature(cell_update)]
#![feature(future_join)]
#![feature(type_alias_impl_trait)]

// read about async Rust in https://rust-lang.github.io/async-book/01_getting_started/01_chapter.html
// hal code examples in https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
// wifi code examples in https://github.com/esp-rs/esp-wifi/blob/main/examples-esp32c3
// board repo in https://github.com/Xinyuan-LilyGO/LilyGo-T-OI-PLUS

mod i2c_adc;
mod int_adc;
mod out_control;

use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use desse::Desse;
use dotenvy_macro::dotenv;
use embassy_executor::{Executor, Spawner, _export::StaticCell};
use embassy_net::{
    // dns::DnsQueryType,
    tcp::{TcpReader, TcpSocket, TcpWriter},
    Config,
    Ipv4Address,
    Stack,
    StackResources,
};
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
    clock::{ClockControl, CpuClock},
    embassy,
    gpio::{Gpio10, GpioPin, Input, Output, PullDown, PushPull, IO},
    i2c::I2C,
    interrupt,
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, LowSpeed, LEDC,
    },
    peripherals::{self, Peripherals, I2C0},
    prelude::*,
    systimer::SystemTimer,
    timer::TimerGroup,
    Rng, Rtc,
};
use i2c_adc::I2CADC;
use mqttrs::{decode_slice, encode_slice, SubscribeTopic};
use shared::{AttinyRequest, AttinyResponse};

use crate::out_control::OutControl;

const SSID: &str = dotenv!("SSID");
const PASSWORD: &str = dotenv!("PASSWORD");

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

static N_PULSES: AtomicU16 = AtomicU16::new(0);
static CHARGER_EN: AtomicBool = AtomicBool::new(false);
static A0_TENSION: AtomicU16 = AtomicU16::new(0);
static A1_TENSION: AtomicU16 = AtomicU16::new(0);
static A2_TENSION: AtomicU16 = AtomicU16::new(0);
static A3_TENSION: AtomicU16 = AtomicU16::new(0);

#[entry]
fn entry() -> ! {
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(main(spawner)).ok();
    });
}

fn read_initial_attiny_state<I2C>(i2c: &mut I2C) -> AttinyResponse
where
    I2C: embedded_hal::i2c::I2c,
{
    let mut buffer = [0u8; 1];
    loop {
        let response = i2c.read(8, &mut buffer);
        match response {
            Ok(_) => {
                let data = AttinyResponse::deserialize_from(&buffer);
                println!("Read data: {data:?}");
                break data;
            }
            Err(v) => println!("Error while receiving i2c data: {v:?}"),
        }
    }
}

fn update_charger_en<I2C>(i2c: &mut I2C, current_charger_en: &mut bool)
where
    I2C: embedded_hal::i2c::I2c,
{
    let desired_charger_en = CHARGER_EN.load(Ordering::Acquire);
    if *current_charger_en == desired_charger_en {
        return;
    }
    let set_command: [u8; 2] = AttinyRequest::UpdateChargerEn(desired_charger_en).serialize();
    let res = i2c.write(8, &set_command);
    match res {
        Ok(_) => {
            *current_charger_en = desired_charger_en;
            println!("Updated charger enable to {current_charger_en}");
        }
        Err(v) => println!("Error while writting i2c data: {v:?}"),
    };
}

/*fn update_n_pulses<I2C>(i2c: &mut I2C) where I2C: embedded_hal::i2c::I2c {
    let response = {
        let request = AttinyCommand::GetNPulses.serialize();
        let mut response_buffer = [0u8; 3];
        let res = i2c.write_read(8, &request, &mut response_buffer);
        match res {
            Ok(_) => AttinyResponse::deserialize_from(&response_buffer),
            Err(v) => {
                println!("Error while receiving i2c data: {v:?}");
                return;
            }
        }
    };
    match response {
        AttinyResponse::NPulses(val) => N_PULSES.store(val, Ordering::Release)
    }
}*/

#[embassy_executor::task]
async fn i2c_controller(mut i2c: I2C<'static, I2C0>) {
    let initial_state = read_initial_attiny_state(&mut i2c);
    let mut current_charger_en = initial_state.charger_en;
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;
        update_charger_en(&mut i2c, &mut current_charger_en);
        // TODO uncomment after implementing this in attiny
        // update_n_pulses(&mut i2c);
        let mut i2c_adc = I2CADC::new(i2c);
        let i2c_read = i2c_adc.read_all_mv();
        A0_TENSION.store(i2c_read.a0, Ordering::Release);
        A1_TENSION.store(i2c_read.a1, Ordering::Release);
        A2_TENSION.store(i2c_read.a2, Ordering::Release);
        A3_TENSION.store(i2c_read.a3, Ordering::Release);
        i2c = i2c_adc.destroy();
        println!("Read analog inputs: {i2c_read:?}");
    }
}

#[embassy_executor::task]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    let mut peripheral_clock_control = system.peripheral_clock_control;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    rtc.swd.disable();
    rtc.rwdt.disable();

    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
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
    let pulse_pin = io.pins.gpio10.into_pull_down_input();
    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority1).unwrap();
    interrupt::enable(
        peripherals::Interrupt::I2C_EXT0,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio18,
        io.pins.gpio19,
        32u32.kHz(),
        &mut peripheral_clock_control,
        &clocks,
    );
    spawner.spawn(i2c_controller(i2c)).ok();

    let pressure_pwm_pin: GpioPin<Output<PushPull>, 5> = io.pins.gpio5.into_push_pull_output();

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

    let mut channel0 = ledc.get_channel(channel::Number::Channel0, pressure_pwm_pin);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 6,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    channel0.set_duty(50).unwrap();

    let out_control = OutControl::new(io.pins.gpio6, io.pins.gpio7);

    /*let internal_adc = {
        let analog = peripherals.APB_SARADC.split();
        let mut adc1_config = AdcConfig::new();
        let analog1 = adc1_config
            .enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(io.pins.gpio2.into_analog(), ATTENUATION);
        let analog2 = adc1_config
            .enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(io.pins.gpio4.into_analog(), ATTENUATION);
        let adc1 =
            ADC::<ADC1>::adc(&mut peripheral_clock_control, analog.adc1, adc1_config).unwrap();
        IntADC {
            analog1,
            analog2,
            adc1,
        }
    };*/

    spawner.spawn(pulse_counter(pulse_pin)).ok();
    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(&stack)).ok();
    spawner.spawn(task(&stack, out_control)).ok();
}

#[embassy_executor::task]
async fn pulse_counter(mut pin: Gpio10<Input<PullDown>>) {
    loop {
        println!("waiting...");
        pin.wait_for_rising_edge().await.unwrap();
        let n_pulses: u16 = N_PULSES.fetch_add(1, Ordering::Release) + 1;
        println!("number of pulses: {n_pulses}");
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
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
                Timer::after(Duration::from_millis(5000)).await
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

async fn subscribe_mqtt_topics<'a>(socket: &mut TcpSocket<'a>) {
    let mut buffer = [0u8; 1024];
    let mut topics = heapless::Vec::<SubscribeTopic, 5>::new();
    topics
        .push(SubscribeTopic {
            qos: mqttrs::QoS::AtMostOnce,
            topic_path: "jaedson/todevice".into(),
        })
        .unwrap();
    let subscribe = mqttrs::Subscribe {
        pid: mqttrs::Pid::new(),
        topics,
    };
    let size = encode_slice(&subscribe.into(), &mut buffer).unwrap();
    socket.write(&buffer[..size]).await.unwrap();
    socket.flush().await.unwrap();
    let size = socket.read(&mut buffer).await.unwrap();
    let decoded = decode_slice(&buffer[..size]).unwrap().unwrap();
    println!("{decoded:?}");
}

#[derive(serde::Serialize, Debug)]
struct PublishState {
    n_pulses: u16,
    a0: u16,
    a1: u16,
    a2: u16,
    a3: u16,
}

#[derive(serde::Deserialize, Debug)]
struct ReceiveState {
    enable_charger: bool,
    enable_pressure: bool,
}

async fn mqtt_publish<'a, 'b>(socket: &mut TcpWriter<'a>) {
    // let mut ping_buffer = [0u8; 1024];
    // let ping = mqttrs::Packet::Pingreq {};
    // let ping_size = encode_slice(&ping, &mut ping_buffer).unwrap();
    let mut ping = false;
    let mut buffer = [0u8; 1024];

    let mut ticker = Ticker::every(Duration::from_secs(2));
    loop {
        ticker.next().await;
        if ping {
            // socket.write(&ping_buffer[..ping_size]).await.unwrap();
        } else {
            let n_pulses = N_PULSES.fetch_min(0, Ordering::Acquire);
            // let battery_mv = i2c_adc.read_mv(&AnalogInput::Battery);
            let state = PublishState {
                n_pulses,
                a0: A0_TENSION.load(Ordering::Acquire),
                a1: A1_TENSION.load(Ordering::Acquire),
                a2: A2_TENSION.load(Ordering::Acquire),
                a3: A3_TENSION.load(Ordering::Acquire),
            };
            let json = serde_json_core::to_string::<PublishState, 256>(&state).unwrap();
            let publish = mqttrs::Publish {
                dup: false,
                payload: json.as_bytes(),
                qospid: mqttrs::QosPid::AtMostOnce,
                retain: false,
                topic_name: "jaedson/tocloud",
            };
            let size = encode_slice(&publish.into(), &mut buffer).unwrap();
            socket.write(&buffer[..size]).await.unwrap();
        }
        socket.flush().await.unwrap();
        ping = !ping;
    }
}

async fn mqtt_read<'a>(socket: &mut TcpReader<'a>, mut outputs_controller: OutControl) {
    let mut buffer = [0u8; 1024];

    loop {
        Timer::after(Duration::from_millis(1_000)).await;

        match socket.read(&mut buffer).await {
            Ok(size) => {
                if size == 0 {
                    continue;
                }
                let decoded = match decode_slice(&buffer[..size]) {
                    Ok(v) => match v {
                        Some(v) => v,
                        None => continue,
                    },
                    Err(e) => {
                        println!("error while parsing: {e:?}");
                        continue;
                    }
                };
                match decoded {
                    mqttrs::Packet::Publish(v) => {
                        let topic = v.topic_name;
                        let (state, _) =
                            serde_json_core::from_slice::<ReceiveState>(&v.payload).unwrap();
                        CHARGER_EN.store(state.enable_charger, Ordering::Release);
                        outputs_controller.pressure_en_set(state.enable_pressure);
                        println!("received message on topic {topic} and updated states");
                    }
                    v => println!("received: {v:?}"),
                }
            }
            Err(e) => {
                println!("error while reading: {e:?}")
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}

#[embassy_executor::task]
async fn task(stack: &'static Stack<WifiDevice<'static>>, outputs_controller: OutControl) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

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
    let address = Ipv4Address::new(192, 168, 1, 107);
    let mut socket = TcpSocket::new(&stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    println!("connecting...");
    let remote_endpoint = (address, 1883);
    socket.connect(remote_endpoint).await.unwrap();
    println!("connected!");

    connect_mqtt(&mut socket).await;
    subscribe_mqtt_topics(&mut socket).await;

    let (mut reader, mut writer) = socket.split();
    let _result = core::future::join!(
        mqtt_read(&mut reader, outputs_controller),
        mqtt_publish(&mut writer)
    )
    .await;
}
