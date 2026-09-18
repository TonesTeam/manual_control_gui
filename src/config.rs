use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use crate::devices::DeviceId;
use crate::rig::Rig;

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum Backend {
    /// Talk to the devices over the RS485 adapter. controller_v2 must not be
    /// using the same port at the same time.
    Serial,
    /// Built-in simulator; no hardware needed.
    Simulator,
    /// Drive nothing locally: send commands to `tstand_server` on the rig's
    /// computer and mirror the state it publishes. The adapter stays plugged
    /// into that machine, so no serial port is opened here.
    Remote,
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
    /// `host:port` of `tstand_server`, used when `backend` is `Remote`.
    pub server_host: String,
    /// Shared secret the server checks on connect. Empty means the server was
    /// started without `--token` and accepts anyone who can reach the port.
    pub server_token: String,
    /// What the *remote* server should drive. Never `Remote` itself.
    pub server_backend: Backend,
    /// Whole-window zoom.
    pub ui_scale: f32,
    /// Size of readouts and labels on the schematic.
    pub label_scale: f32,
    /// Working volume of one slot, for the fill gauge (controller_v2 SLOT_VOLUME_UL).
    pub slot_volume_ul: f32,
    /// Read slot states from controller_v2's /slot-status every 2 s.
    pub poll_controller: bool,
    /// Which slots, ports and sensors this rig actually has.
    pub rig: Rig,
    /// Drive the Peltier slot-temperature board over CAN.
    pub temp_enabled: bool,
    /// Serial port of the CAN adapter. **Leave empty.**
    ///
    /// Empty resolves the adapter through `/dev/serial/by-id`, which names it
    /// by its USB serial number and so survives re-enumeration. A hard-coded
    /// `/dev/ttyACM0` does not: unplugging the adapter, or a bus glitch, brings
    /// it back as `ttyACM2` and the rig loses its sensors and its heater until
    /// someone edits this. That has already happened once on this rig.
    ///
    /// The adapter is shared with the optical sensors, so only one program may
    /// hold it.
    pub temp_port: String,
    /// CAN id the optical sensor board answers on. The firmware ships as
    /// 0x700; some builds of its own crate assume 0x7FF.
    pub sensor_can_id: u16,
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
            server_host: "tonespi.local:7373".to_string(),
            server_token: String::new(),
            server_backend: Backend::Serial,
            ui_scale: 1.0,
            label_scale: 1.2,
            slot_volume_ul: 300.0,
            poll_controller: true,
            rig: Rig::default(),
            temp_enabled: true,
            temp_port: String::new(),
            sensor_can_id: crate::sensors::DEFAULT_DEVICE_ID,
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

    /// The backend that actually moves the hardware: in `Remote` mode that is
    /// whatever the server was told to run, not `Remote` itself. Callers that
    /// care about simulated-versus-real (level tracking, the demo button) want
    /// this rather than `backend`.
    pub fn effective_backend(&self) -> Backend {
        match self.backend {
            Backend::Remote => self.server_backend,
            other => other,
        }
    }

    /// Takes over the fields the rig's machine owns, leaving this machine's
    /// own preferences alone.
    ///
    /// The adapter, the slave addresses and the pump calibration belong to
    /// wherever the hardware is plugged in; a GUI that connects must show
    /// those rather than whatever it happened to save last time. Zoom, the
    /// server address and the token stay local.
    pub fn adopt_server_fields(&mut self, from: &Settings) {
        self.server_backend = match from.backend {
            Backend::Remote => self.server_backend,
            other => other,
        };
        self.port = from.port.clone();
        self.baud_rate = from.baud_rate;
        self.pp01_addr = from.pp01_addr;
        self.pp02_addr = from.pp02_addr;
        self.sv01_addr = from.sv01_addr;
        self.sv02_addr = from.sv02_addr;
        self.sv03_addr = from.sv03_addr;
        self.poll_interval_ms = from.poll_interval_ms;
        self.reply_timeout_ms = from.reply_timeout_ms;
        self.pp01_max_steps = from.pp01_max_steps;
        self.pp01_ul_per_step = from.pp01_ul_per_step;
        self.pp02_max_steps = from.pp02_max_steps;
        self.pp02_ul_per_step = from.pp02_ul_per_step;
        self.slot_volume_ul = from.slot_volume_ul;
        self.controller_url = from.controller_url.clone();
        // The adapters and the plumbing are the rig's, not this window's.
        self.rig = from.rig.clone();
        self.temp_enabled = from.temp_enabled;
        self.temp_port = from.temp_port.clone();
        self.sensor_can_id = from.sensor_can_id;
    }

    pub fn load() -> Self {
        Self::load_from(SETTINGS_FILE)
    }

    pub fn save(&self) -> Result<(), String> {
        self.save_to(SETTINGS_FILE)
    }

    pub fn load_from(name: &str) -> Self {
        match std::fs::read_to_string(data_file(name)) {
            Ok(text) => serde_json::from_str(&text).unwrap_or_default(),
            Err(_) => Self::default(),
        }
    }

    pub fn save_to(&self, name: &str) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(data_file(name), text).map_err(|e| e.to_string())
    }
}

const SETTINGS_FILE: &str = "tstand_settings.json";

/// Where `tstand_server` keeps the settings it was last told to apply.
pub const SERVER_SETTINGS_FILE: &str = "tstand_server.json";

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
