#![no_std]
#![no_main]
#![feature(cell_update)]
#![feature(future_join)]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]
#![feature(let_chains)]

// read about async Rust in https://rust-lang.github.io/async-book/01_getting_started/01_chapter.html
// hal code examples in https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
// wifi code examples in https://github.com/esp-rs/esp-wifi/blob/main/examples-esp32c3
// board repo in https://github.com/Xinyuan-LilyGO/LilyGo-T-OI-PLUS

mod device_state;
mod i2c_adc;
mod mqtt_publish;
mod pressure_boost;
mod pulse_counter;
mod wifi;

use dotenvy_macro::dotenv;
use embassy_executor::_export::StaticCell;
use embassy_net::{Config, Stack, StackResources};
use esp_backtrace as _;
use esp_wifi::{initialize, wifi::WifiMode, EspWifiInitFor};
use hal::{
    clock::ClockControl,
    embassy,
    gpio::IO,
    i2c::I2C,
    interrupt,
    ledc::{LSGlobalClkSource, LEDC},
    peripherals::{self, Peripherals},
    prelude::*,
    timer::TimerGroup,
    Rng,
};
use pressure_boost::PressureController;

use crate::i2c_adc::I2CADCReader;

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

#[entry]
fn entry() -> ! {
    let peripherals = Peripherals::take();

    let system = peripherals.DPORT.split();
    let clocks = &*singleton!(ClockControl::max(system.clock_control).freeze());
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

    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority1).unwrap();
    interrupt::enable(
        peripherals::Interrupt::I2C_EXT0,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio15,
        io.pins.gpio4,
        100u32.kHz(),
        &mut peripheral_clock_control,
        &clocks,
    );

    let ledc = &mut *singleton!(LEDC::new(
        peripherals.LEDC,
        &clocks,
        &mut peripheral_clock_control
    ));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let pressure_controller = PressureController::new(ledc, io.pins.gpio23, io.pins.gpio22);
    let i2c_adc = I2CADCReader::new(i2c);

    static EXECUTOR: StaticCell<embassy::executor::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(embassy::executor::Executor::new());
    executor.run(|spawner| {
        spawner
            .spawn(pulse_counter::pulse_counter(io.pins.gpio12))
            .ok();
        spawner
            .spawn(wifi::wifi_controller_task(
                controller,
                SSID.into(),
                PASSWORD.into(),
            ))
            .ok();
        spawner.spawn(wifi::net_task(&stack)).ok();
        spawner
            .spawn(device_state::device_state_monitor_task(
                pressure_controller,
                i2c_adc,
            ))
            .ok();
        spawner
            .spawn(mqtt_publish::publish_mqtt_task(&stack, io.pins.gpio2))
            .ok();
    });
}
