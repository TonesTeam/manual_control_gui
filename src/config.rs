use serde::{Deserialize, Serialize};
use std::path::PathBuf;

pub const SELECTOR_8_MAX: u16 = 8;
pub const SELECTOR_16_MAX: u16 = 16;
pub const MAIN_PUMP_MAX: u16 = 3810;
pub const SECONDARY_PUMP_MAX: u16 = 12000;

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Settings {
    pub port: String,
    pub baud_rate: u32,
    pub main_pump_addr: u8,
    pub secondary_pump_addr: u8,
    pub selector1_addr: u8,
    pub selector2_addr: u8,
    pub selector3_addr: u8,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            port: String::new(),
            baud_rate: 9600,
            main_pump_addr: 1,
            secondary_pump_addr: 2,
            selector1_addr: 3,
            selector2_addr: 4,
            selector3_addr: 5,
        }
    }
}

fn config_path() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|p| p.to_path_buf()))
        .unwrap_or_else(|| PathBuf::from("."))
        .join("tstand_settings.json")
}

impl Settings {
    pub fn load() -> Self {
        let path = config_path();
        match std::fs::read_to_string(&path) {
            Ok(text) => serde_json::from_str(&text).unwrap_or_default(),
            Err(_) => Self::default(),
        }
    }

    pub fn save(&self) -> Result<(), String> {
        let path = config_path();
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(&path, text).map_err(|e| e.to_string())
    }
}
