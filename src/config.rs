use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use crate::devices::DeviceId;

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum Backend {
    /// Talk to the devices over the RS485 adapter. controller_v2 must not be
    /// using the same port at the same time.
    Serial,
    /// Built-in simulator; no hardware needed.
    Simulator,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct Settings {
    pub backend: Backend,
    pub port: String,
    pub baud_rate: u32,
    pub pp01_addr: u8,
    pub pp02_addr: u8,
    pub sv01_addr: u8,
    pub sv02_addr: u8,
    pub sv03_addr: u8,
    pub poll_interval_ms: u64,
    pub reply_timeout_ms: u64,
    pub pp01_max_steps: u16,
    pub pp01_ul_per_step: f32,
    pub pp02_max_steps: u16,
    pub pp02_ul_per_step: f32,
    pub controller_url: String,
    /// Whole-window zoom.
    pub ui_scale: f32,
    /// Size of readouts and labels on the schematic.
    pub label_scale: f32,
    /// Working volume of one slot, for the fill gauge (controller_v2 SLOT_VOLUME_UL).
    pub slot_volume_ul: f32,
    /// Read slot states from controller_v2's /slot-status every 2 s.
    pub poll_controller: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            backend: Backend::Simulator,
            port: String::new(),
            baud_rate: 9600,
            pp01_addr: DeviceId::Pp01.default_addr(),
            pp02_addr: DeviceId::Pp02.default_addr(),
            sv01_addr: DeviceId::Sv01.default_addr(),
            sv02_addr: DeviceId::Sv02.default_addr(),
            sv03_addr: DeviceId::Sv03.default_addr(),
            poll_interval_ms: 300,
            reply_timeout_ms: 250,
            pp01_max_steps: 3810,
            pp01_ul_per_step: 2.083,
            pp02_max_steps: 12000,
            pp02_ul_per_step: 0.416,
            controller_url: "127.0.0.1:3000".to_string(),
            ui_scale: 1.0,
            label_scale: 1.2,
            slot_volume_ul: 300.0,
            poll_controller: true,
        }
    }
}

impl Settings {
    pub fn addr(&self, id: DeviceId) -> u8 {
        match id {
            DeviceId::Pp01 => self.pp01_addr,
            DeviceId::Pp02 => self.pp02_addr,
            DeviceId::Sv01 => self.sv01_addr,
            DeviceId::Sv02 => self.sv02_addr,
            DeviceId::Sv03 => self.sv03_addr,
        }
    }

    pub fn addr_mut(&mut self, id: DeviceId) -> &mut u8 {
        match id {
            DeviceId::Pp01 => &mut self.pp01_addr,
            DeviceId::Pp02 => &mut self.pp02_addr,
            DeviceId::Sv01 => &mut self.sv01_addr,
            DeviceId::Sv02 => &mut self.sv02_addr,
            DeviceId::Sv03 => &mut self.sv03_addr,
        }
    }

    pub fn max_steps(&self, id: DeviceId) -> u16 {
        match id {
            DeviceId::Pp01 => self.pp01_max_steps,
            DeviceId::Pp02 => self.pp02_max_steps,
            _ => 0,
        }
    }

    pub fn ul_per_step(&self, id: DeviceId) -> f32 {
        match id {
            DeviceId::Pp01 => self.pp01_ul_per_step,
            DeviceId::Pp02 => self.pp02_ul_per_step,
            _ => 0.0,
        }
    }

    pub fn load() -> Self {
        match std::fs::read_to_string(data_file(SETTINGS_FILE)) {
            Ok(text) => serde_json::from_str(&text).unwrap_or_default(),
            Err(_) => Self::default(),
        }
    }

    pub fn save(&self) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(data_file(SETTINGS_FILE), text).map_err(|e| e.to_string())
    }
}

const SETTINGS_FILE: &str = "tstand_settings.json";

const APP_DIR: &str = "TonesLiquidProcessing";

/// Settings, layout and levels live next to the executable when that folder is
/// writable (portable copies, `cargo run`). Otherwise they go to the per-user
/// config folder: `%APPDATA%` on Windows, `~/Library/Application Support` on
/// macOS, `$XDG_CONFIG_HOME` or `~/.config` elsewhere.
pub fn data_file(name: &str) -> PathBuf {
    static DIR: OnceLock<PathBuf> = OnceLock::new();
    DIR.get_or_init(data_dir).join(name)
}

fn data_dir() -> PathBuf {
    let exe_dir = std::env::current_exe().ok().and_then(|p| p.parent().map(Path::to_path_buf));
    if let Some(dir) = &exe_dir
        && is_writable(dir)
    {
        return dir.clone();
    }
    if let Some(dir) = user_config_dir().map(|d| d.join(APP_DIR))
        && std::fs::create_dir_all(&dir).is_ok()
    {
        return dir;
    }
    exe_dir.unwrap_or_else(|| PathBuf::from("."))
}

fn is_writable(dir: &Path) -> bool {
    let probe = dir.join(".tstand_write_test");
    let ok = std::fs::write(&probe, b"").is_ok();
    let _ = std::fs::remove_file(&probe);
    ok
}

fn user_config_dir() -> Option<PathBuf> {
    let env = |key: &str| std::env::var_os(key).filter(|v| !v.is_empty()).map(PathBuf::from);
    if cfg!(target_os = "windows") {
        env("APPDATA")
    } else if cfg!(target_os = "macos") {
        env("HOME").map(|home| home.join("Library/Application Support"))
    } else {
        env("XDG_CONFIG_HOME").or_else(|| env("HOME").map(|home| home.join(".config")))
    }
}
