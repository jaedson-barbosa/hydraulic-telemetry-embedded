#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(generic_arg_infer)]
#![feature(error_in_core)]

// read about async Rust in https://rust-lang.github.io/async-book/01_getting_started/01_chapter.html
// hal code examples in https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
// wifi code examples in https://github.com/esp-rs/esp-wifi/blob/main/examples-esp32c3
// board repo in https://github.com/Xinyuan-LilyGO/LilyGo-T-OI-PLUS

use core::sync::atomic::{AtomicU16, Ordering};
use desse::{Desse, DesseSized};
use dotenvy_macro::dotenv;
use embedded_storage::{ReadStorage, Storage};
use embedded_svc::{
    io::{Read, Write},
    ipv4::Interface,
    wifi::{ClientConfiguration, Configuration, Wifi},
};
use esp_backtrace as _;
use esp_println::println;
use esp_storage::FlashStorage;
use esp_wifi::{
    current_millis, initialize,
    wifi::{utils::create_network_interface, WifiMode},
    wifi_interface::{Socket, WifiStack},
    EspWifiInitFor,
};
use hal::{
    clock::{ClockControl, Clocks},
    gpio::{GpioPin, Output, PushPull, Unknown, IO, RTCPin},
    interrupt,
    ledc::{LSGlobalClkSource, LowSpeed, LEDC},
    macros::ram,
    peripherals::{self, Peripherals},
    prelude::*,
    rtc_cntl::{
        get_wakeup_cause,
        sleep::{Ext0WakeupSource, TimerWakeupSource, WakeupLevel},
    },
    Delay, Rng, Rtc,
};
use mqttrs::{decode_slice, encode_slice, Pid};

const FLASH_ADDR: u32 = 0x9000;
const INTERVAL_SEC: u64 = 60;

const SSID: &str = dotenv!("SSID");
const PASSWORD: &str = dotenv!("PASSWORD");

static ADITIONAL_PULSES: AtomicU16 = AtomicU16::new(0);

#[derive(serde::Serialize, Clone, Copy, Debug, Default)]
pub struct I2CADCRead {
    pub battery_ma: i16,
    pub battery_mv: u16,
    pub ldo_inp_mv: u16,
    pub pressure_mv: u16,
}

#[derive(serde::Serialize, Clone, Copy, Debug, Default)]
pub struct DeviceState {
    pub adc_state: I2CADCRead,
    pub n_pulses: u16,
    pub time_sec: u64,
}

#[derive(Desse, DesseSized, Default)]
pub struct PersistentState {
    pub n_pulses: u16,
    pub next_transmission: u64,
}

impl PersistentState {
    pub fn reset() -> Self {
        let counter = PersistentState::default();
        let bytes = counter.serialize();
        let mut flash = FlashStorage::new();
        flash.write(FLASH_ADDR, &bytes).unwrap();
        counter
    }

    pub fn get(add_pulse: bool) -> Self {
        let mut flash = FlashStorage::new();
        let mut bytes = [0u8; _];
        flash.read(FLASH_ADDR, &mut bytes).unwrap();
        let mut counter: PersistentState = PersistentState::deserialize_from(&bytes);
        if add_pulse {
            counter.n_pulses += 1;
            let bytes = counter.serialize();
            flash.write(FLASH_ADDR, &bytes).unwrap();
        }
        counter
    }

    pub fn next(&self, new_pulses: Option<u16>) -> Self {
        let mut flash = FlashStorage::new();
        let state = PersistentState {
            n_pulses: match new_pulses {
                Some(v) => v,
                None => self.n_pulses,
            },
            next_transmission: self.next_transmission + INTERVAL_SEC,
        };
        let bytes = state.serialize();
        flash.write(FLASH_ADDR, &bytes).unwrap();
        state
    }
}

#[entry]
fn entry() -> ! {
    let peripherals = Peripherals::take();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let (pulse_wake, timer_wake) = match get_wakeup_cause() {
        hal::reset::SleepSource::Ext0 => (true, false),
        hal::reset::SleepSource::Timer => (false, true),
        _ => (false, false),
    };

    let persistent_state = if !pulse_wake && !timer_wake {
        PersistentState::reset()
    } else {
        PersistentState::get(pulse_wake)
    };

    let system = peripherals.DPORT.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut peripheral_clock_control = system.peripheral_clock_control;

    let rtc = Rtc::new(peripherals.RTC_CNTL);
    let wake_pin = io.pins.gpio33;
    {
        let time = rtc.get_time_ms() / 1000;
        if time < persistent_state.next_transmission {
            // reenter in sleep mode if we still need to wait
            go_sleep(&clocks, persistent_state.next_transmission, wake_pin, rtc);
        }
    }

    // time to send data
    let mut active_state_led = io.pins.gpio2.into_push_pull_output();
    active_state_led.set_high().unwrap();

    let mut pulse_pin = io.pins.gpio12.into_pull_up_input();
    pulse_pin.listen(hal::gpio::Event::FallingEdge);
    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority1).unwrap();

    interrupt::enable(
        peripherals::Interrupt::I2C_EXT0,
        interrupt::Priority::Priority1,
    )
    .unwrap();
    let i2c = hal::i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio4,
        io.pins.gpio16,
        100u32.kHz(),
        &mut peripheral_clock_control,
        &clocks,
    );

    let mut ledc = LEDC::new(peripherals.LEDC, &clocks, &mut peripheral_clock_control);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut pressure_en_pin = io.pins.gpio22.into_push_pull_output();
    pressure_en_pin.set_high().unwrap();
    start_boost_pwm(&ledc, io.pins.gpio23);

    println!("preparing to send data...");

    let timer =
        hal::timer::TimerGroup::new(peripherals.TIMG1, &clocks, &mut peripheral_clock_control)
            .timer0;
    let rng = Rng::new(peripherals.RNG);
    let radio_control = system.radio_clock_control;
    let init = initialize(EspWifiInitFor::Wifi, timer, rng, radio_control, &clocks).unwrap();

    let (wifi, _) = peripherals.RADIO.split();
    let mut socket_set_entries: [_; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiMode::Sta, &mut socket_set_entries).unwrap();
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });
    controller.set_configuration(&client_config).unwrap();
    controller.start().unwrap();
    controller.connect().unwrap();

    println!("Wait to get connected");
    loop {
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", controller.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            println!("got ip {:?}", wifi_stack.get_ip_info());
            break;
        }
    }

    println!("preparing to send device state");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    let address = smoltcp::wire::Ipv4Address::new(192, 168, 0, 114); // 54, 80, 140, 156

    socket.work();
    socket.open(address.into_address(), 1883).unwrap();

    let adc_state = read_i2c_adc(i2c);
    pressure_en_pin.set_low().unwrap();
    control_charger(io.pins.gpio15.into_push_pull_output(), adc_state.battery_mv);

    let initial_pulses = persistent_state.n_pulses;
    let sent_n_pulses = connect_to_broker_and_publish_device(
        adc_state,
        initial_pulses,
        &mut socket,
        &rtc,
    );

    socket.work();
    socket.disconnect();

    let aditional_pulses = ADITIONAL_PULSES.load(Ordering::Acquire);
    let remaining_n_pulses = initial_pulses + aditional_pulses - sent_n_pulses;
    let next = persistent_state.next(Some(remaining_n_pulses));
    active_state_led.set_low().unwrap();
    go_sleep(&clocks, next.next_transmission, wake_pin, rtc);
}

fn connect_to_broker_and_publish_device<'a, 'b>(
    adc_state: I2CADCRead,
    initial_pulses: u16,
    socket: &mut Socket<'a, 'b>,
    rtc: &Rtc<'static>,
) -> u16 {
    if let Err(err) = connect_to_mqtt_broker(socket) {
        println!("Fail to connect to broker: {err:?}");
        return 0;
    }
    let aditional_pulses = ADITIONAL_PULSES.load(Ordering::Acquire);
    let device_state = DeviceState {
        adc_state,
        n_pulses: initial_pulses + aditional_pulses,
        time_sec: rtc.get_time_ms() / 1000,
    };
    println!("{device_state:?}");
    if let Err(err) = publish_device_state(socket, device_state) {
        println!("error while sending state: {err:?}");
        return 0;
    }
    println!("device state sent");
    device_state.n_pulses
}

#[ram]
#[interrupt]
unsafe fn GPIO() {
    ADITIONAL_PULSES.fetch_add(1, Ordering::Release);
}

fn go_sleep(
    clocks: &Clocks<'static>,
    next_transmission: u64,
    wake_pin: GpioPin<Unknown, 33>,
    mut rtc: Rtc,
) -> ! {
    let time = rtc.get_time_ms() / 1000;
    let mut delay = Delay::new(&clocks);
    let remaining_time = next_transmission - time;
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(remaining_time));
    let mut ext0_pin = wake_pin.into_pull_up_input();
    let ext0 = Ext0WakeupSource::new(&mut ext0_pin, WakeupLevel::Low);
    rtc.sleep_deep(&[&timer, &ext0], &mut delay);
}

pub fn control_charger(mut charger_en_pin: GpioPin<Output<PushPull>, 15>, battery_mv: u16) {
    let enabled = charger_en_pin.is_set_high().unwrap();
    if battery_mv > 4250 {
        charger_en_pin.set_low().unwrap();
    } else if battery_mv < 4150 && !enabled {
        charger_en_pin.set_high().unwrap();
    }
    charger_en_pin.rtcio_pad_hold(true);
}

pub fn start_boost_pwm<'a>(ledc: &'a LEDC<'a>, pwm_pin: GpioPin<Unknown, 23>) {
    use hal::ledc::{channel, timer};

    let pwm_pin = pwm_pin.into_push_pull_output();

    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer2);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 500u32.kHz(),
        })
        .unwrap();

    let mut channel0 = ledc.get_channel(channel::Number::Channel0, pwm_pin);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 50,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
}

fn connect_to_mqtt_broker<'a, 'b>(
    socket: &mut Socket<'a, 'b>,
) -> Result<(), esp_wifi::wifi_interface::IoError> {
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
    socket.write(&buffer[..size])?;
    socket.flush()?;

    let mut buffer = [0u8; 512];
    socket.read(&mut buffer).unwrap();
    let decoded = decode_slice(&buffer).unwrap().unwrap();
    esp_println::println!("{decoded:?}");
    Ok(())
}

pub fn publish_device_state<'a, 'b>(
    socket: &mut Socket<'a, 'b>,
    device_state: DeviceState,
) -> Result<(), esp_wifi::wifi_interface::IoError> {
    let mut buffer = [0u8; 10000];
    let json = serde_json_core::to_string::<DeviceState, 10000>(&device_state).unwrap();
    let publish = mqttrs::Publish {
        dup: false,
        payload: json.as_bytes(),
        qospid: mqttrs::QosPid::AtLeastOnce(Pid::new()),
        retain: false,
        topic_name: "jaedson/tocloud",
    };
    let size = encode_slice(&publish.into(), &mut buffer).unwrap();
    socket.write(&buffer[..size])?;
    socket.flush()?;
    esp_println::println!("awaiting...");
    let len = socket.read(&mut buffer).unwrap();
    let decoded = decode_slice(&buffer[..len]).unwrap().unwrap();
    esp_println::println!("{decoded:?}");
    esp_println::println!("ok");
    Ok(())
}

pub fn read_i2c_adc(i2c: hal::i2c::I2C<'static, hal::peripherals::I2C0>) -> I2CADCRead {
    use ads1x1x::{Ads1x1x, FullScaleRange, SlaveAddr};

    let get_mv = |val: i16| -> u16 {
        if val < 0 {
            0
        } else {
            (val as u32 * 3 / 16) as u16
        }
    };

    let address = SlaveAddr::default();
    let mut adc = Ads1x1x::new_ads1115(i2c, address);
    adc.set_full_scale_range(FullScaleRange::Within6_144V)
        .unwrap();
    let dif_a2_a3 = nb::block!(adc.read(&mut ads1x1x::channel::DifferentialA2A3)).unwrap();
    let a0 = nb::block!(adc.read(&mut ads1x1x::channel::SingleA0)).unwrap();
    let a1 = nb::block!(adc.read(&mut ads1x1x::channel::SingleA1)).unwrap();
    let a2 = nb::block!(adc.read(&mut ads1x1x::channel::SingleA2)).unwrap();
    I2CADCRead {
        battery_ma: dif_a2_a3 / 8,
        battery_mv: get_mv(a2),
        ldo_inp_mv: get_mv(a1),
        pressure_mv: get_mv(a0),
    }
}
