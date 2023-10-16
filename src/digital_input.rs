const WIFI_EN: bool = true;
const HIGH_FREQ_EN: bool = true;

#[derive(serde::Serialize, Clone, Copy, Debug)]
pub struct DigitalInputState {
    pub wifi_en: bool,
    pub high_freq_en: bool
}

impl DigitalInputState {
    pub fn get() -> Self {
        Self {
            wifi_en: Self::get_wifi_en(),
            high_freq_en: Self::get_high_freq_en()
        }
    }

    pub fn get_wifi_en() -> bool {
        WIFI_EN
    }

    pub fn get_high_freq_en() -> bool {
        HIGH_FREQ_EN
    }
}
