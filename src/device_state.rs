use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Timer, Duration, Instant};
use crate::i2c_adc::{I2CADCRead, I2CADCReader};
use crate::pressure_boost::PressureController;
use crate::pulse_counter::get_n_pulses;

const INTERVAL_MS: u64 = 100;
const BATCH_SIZE: usize = 10;
const N_EN_PRESSURE: usize = 5;

pub static BATTERY_MV: Signal<CriticalSectionRawMutex, u16> = Signal::new();

#[derive(serde::Serialize, Clone, Copy, Debug, Default)]
pub struct DeviceState {
    pub adc_state: I2CADCRead,
    pub n_pulses: u16,
    pub time_sec: u64
}

#[derive(serde::Serialize, Clone, Copy, Debug, Default)]
pub struct DeviceStateBatch {
    pub data: [DeviceState; BATCH_SIZE],
    pub time_sec: u64,
}

impl DeviceStateBatch {
    pub async fn receive() -> Self {
        Self {
            data: STATE.receive().await,
            time_sec: Instant::now().as_secs(),
        }
    }

    pub fn uptate_time(&mut self) {
        self.time_sec = Instant::now().as_secs();
    }
}

static STATE: Channel<CriticalSectionRawMutex, [DeviceState; BATCH_SIZE], 100> = Channel::new();

#[embassy_executor::task]
pub async fn device_state_monitor_task(
    mut pressure_controller: PressureController,
    mut i2c_adc: I2CADCReader,
) {
    let mut batch = [DeviceState::default(); BATCH_SIZE];
    loop {
        for (index, item) in batch.iter_mut().enumerate() {
            pressure_controller.set_enable(index > 1 && index < N_EN_PRESSURE + 2);
            Timer::after(Duration::from_millis(INTERVAL_MS)).await;
            let adc_state = i2c_adc.read();
            BATTERY_MV.signal(adc_state.battery_mv);
            item.adc_state = adc_state;
            item.n_pulses = get_n_pulses();
            item.time_sec = Instant::now().as_secs();
        }
        STATE.send(batch).await
    }
}
