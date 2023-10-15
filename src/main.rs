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

mod charger;
mod device_state;
mod digital_input;
mod i2c_adc;
mod int_adc;
mod led_output;
mod mqtt;
mod pressure_boost;
mod pulse_counter;
mod temperature;
mod wifi;

use device_state::state_sampling_task;
use dotenvy_macro::dotenv;
use embassy_executor::{Spawner, _export::StaticCell};
use embassy_net::{
    // dns::DnsQueryType,
    Config,
    Stack,
    StackResources,
};
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
use i2c_adc::I2CADC;
use int_adc::IntADC;
use led_output::{wifi_led_state_task, PulseLed};
use mqtt::publish_mqtt_task;
use pressure_boost::PressureController;
use temperature::Temperature;
use wifi::{net_task, wifi_controller_task};

const SSID: &str = dotenv!("SSID");
const PASSWORD: &str = dotenv!("PASSWORD");

static mut CORE_STACK: hal::cpu_control::Stack<8192> = hal::cpu_control::Stack::new();

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
    static EXECUTOR: StaticCell<embassy::executor::Executor> = StaticCell::new();
    let executor = EXECUTOR.init(embassy::executor::Executor::new());
    executor.run(|spawner| {
        spawner.spawn(main(spawner)).ok();
    });
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

    digital_input::start_digital_input_monitor_tasks(
        &spawner,
        io.pins.gpio2,
        io.pins.gpio4,
        io.pins.gpio16,
        io.pins.gpio17,
    );

    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority1).unwrap();
    interrupt::enable(
        peripherals::Interrupt::I2C_EXT0,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio6,
        io.pins.gpio7,
        32u32.kHz(),
        &mut peripheral_clock_control,
        &clocks,
    );
    let mut i2c_adc = I2CADC::new(i2c);

    let ledc = &mut *singleton!(LEDC::new(
        peripherals.LEDC,
        &clocks,
        &mut peripheral_clock_control
    ));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let pulse_led = PulseLed::init(io.pins.gpio26);
    spawner.spawn(wifi_led_state_task(io.pins.gpio33)).ok();
    spawner
        .spawn(pulse_counter::pulse_counter(io.pins.gpio15, pulse_led))
        .ok();
    spawner
        .spawn(wifi_controller_task(
            controller,
            SSID.into(),
            PASSWORD.into(),
        ))
        .ok();
    spawner.spawn(net_task(&stack)).ok();
    spawner.spawn(publish_mqtt_task(&stack)).ok();

    let int_adc = IntADC::new(
        peripherals.SENS.split().adc1,
        io.pins.gpio32,
        io.pins.gpio34,
        io.pins.gpio35,
        io.pins.gpio36,
        io.pins.gpio39,
    );
    let one_wire_pin = io.pins.gpio19.into_open_drain_output();
    let temperature_sensor = Temperature::new(one_wire_pin, hal::Delay::new(&clocks)).unwrap();
    let pressure_controller = PressureController::new(ledc, io.pins.gpio12, io.pins.gpio13);
    spawner
        .spawn(state_sampling_task(
            int_adc,
            pressure_controller,
            temperature_sensor,
        ))
        .ok();

    spawner
        .spawn(charger::charger_control_task(ledc, io.pins.gpio23))
        .ok();

    let mut cpu_control = hal::cpu_control::CpuControl::new(system.cpu_control);
    let _guard = cpu_control.start_app_core(unsafe { &mut CORE_STACK }, move || loop {
        i2c_adc.read_all_inputs();
    });
}
