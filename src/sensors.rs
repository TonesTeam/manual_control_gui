//! The optical liquid-sensor board, on the same CAN adapter as the Peltier.
//!
//! One board carries six fibre-optic detectors and multiplexes them into a
//! single frame. The firmware's own parameter table calls them `A0`–`A5`;
//! which of them the rig has wired, and where, is [`crate::rig`]'s business.
//!
//! # Why the protocol is spelled out here
//!
//! There is a crate for this board, `optical-sensor-can`, but it lives in a
//! private repository reached over SSH. Depending on it would mean this
//! program only builds for someone holding a deploy key — including CI. The
//! protocol is one request and one reply, so it is written out below instead,
//! and the transport is borrowed from the temperature crate's shared SLCAN
//! handle so both boards still share the one adapter.
//!
//! # Wire format
//!
//! Standard 11-bit ID, DLC 8, 1 Mbit/s. Byte 0 carries a 7-bit op code and a
//! read/write bit: `op << 1` asks, `op << 1 | 1` answers, and `0x80` marks an
//! unsolicited change notification. The six states follow in bytes 1–6.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

/// How many detectors one board has.
pub const CHANNELS: usize = 6;

/// Read the six liquid states.
pub const OP_SENSOR_STATES: u8 = 0x00;

/// Read the six raw 8-bit ADC readings behind those states.
///
/// The raw value is worth having because the binary state cannot tell a solid
/// column from froth: both read wet. The raw one moves continuously while a
/// front passes, so the transition is visible instead of being a single flip.
///
/// **Do not assume a polarity, and do not assume the channels share one.**
/// Measured on this rig by aspirating a front back past the two in-line
/// detectors while sampling:
///
/// * A5 fell smoothly from 252 down to 19 as the front withdrew, and its state
///   turned wet once it dropped under about 30.
/// * A4 sat at 251 throughout and read wet the whole time.
///
/// Those two cannot both be explained by the single pair of thresholds the
/// board reports (115 and 30), so the thresholds are either per channel or the
/// detectors are not all the same. `optical-sensor-can` documents one global
/// rule — wet below the max threshold — and that does not fit A4.
///
/// So the raw reading is exposed and plotted, and nothing here converts it to
/// a state or a margin. The board's own state bit is the authority on wet
/// versus dry; the raw value says how *settled* that state is, which is the
/// question worth asking of it.
pub const OP_RAW_ADC: u8 = 0x03;

/// Read the threshold a channel must fall below to read wet.
pub const OP_MAX_THRESHOLD: u8 = 0x01;
/// Read the threshold a channel must climb above to read dry again.
pub const OP_MIN_THRESHOLD: u8 = 0x02;

/// Byte 0 of an unsolicited "this sensor changed" frame.
pub const NOTIFICATION: u8 = 0x80;

/// The board's CAN id.
///
/// `0x7FF` is what the boards on this rig answer on, confirmed against the
/// hardware. The firmware flasher's parameter table calls `0x700` the factory
/// default, but **`0x700` must never be used here**: it is the Peltier
/// controller's command id, so asking for sensor states on it would send the
/// temperature board a malformed command on every poll. See
/// [`collides_with_temperature_board`].
pub const DEFAULT_DEVICE_ID: u16 = 0x7FF;

/// The ids the Peltier slot-temperature controller owns: `0x700` for commands
/// and `0x701`–`0x71A` for its telemetry.
pub const TEMPERATURE_IDS: std::ops::RangeInclusive<u16> = 0x700..=0x71F;

/// Whether this sensor id would tread on the temperature board.
///
/// The two boards share one adapter and neither protocol carries a node
/// address, so an id in the Peltier's range is not a configuration to be
/// respected — it is a command aimed at a heater.
pub fn collides_with_temperature_board(device_id: u16) -> bool {
    TEMPERATURE_IDS.contains(&device_id)
}

/// The frame that asks for something, by op code.
pub fn request(op: u8) -> [u8; 8] {
    let mut frame = [0u8; 8];
    frame[0] = op << 1;
    frame
}

/// The frame that asks for all six states.
pub fn request_states() -> [u8; 8] {
    request(OP_SENSOR_STATES)
}

/// The frame that asks for all six raw readings.
pub fn request_adc() -> [u8; 8] {
    request(OP_RAW_ADC)
}

/// Reads a reply carrying one value per channel, for the op that was asked.
fn parse_six(data: &[u8; 8], op: u8) -> Option<[u8; CHANNELS]> {
    if data[0] & 0x01 == 0 || data[0] >> 1 != op {
        return None;
    }
    let mut out = [0u8; CHANNELS];
    out.copy_from_slice(&data[1..1 + CHANNELS]);
    Some(out)
}

/// Reads a reply to [`request_adc`], or `None` if this frame is something else.
pub fn parse_adc(data: &[u8; 8]) -> Option<[u8; CHANNELS]> {
    parse_six(data, OP_RAW_ADC)
}

/// Reads a reply carrying a single byte, such as a threshold.
pub fn parse_threshold(data: &[u8; 8], op: u8) -> Option<u8> {
    (data[0] & 0x01 != 0 && data[0] >> 1 == op).then(|| data[1])
}

/// Reads a reply to [`request_states`], or `None` if this frame is something
/// else — an echo of the request, another op code, or a notification.
pub fn parse_states(data: &[u8; 8]) -> Option<[bool; CHANNELS]> {
    // The low bit distinguishes an answer from the question; without it a
    // reflected request would be read as "all six dry".
    parse_six(data, OP_SENSOR_STATES).map(|raw| raw.map(|v| v != 0))
}

/// A change the board reported without being asked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Change {
    pub channel: u8,
    /// True when liquid arrived, false when it left.
    pub liquid: bool,
    /// Milliseconds since this channel last changed.
    pub since_ms: u32,
}

/// Reads a change notification, or `None` if this frame is not one.
pub fn parse_change(data: &[u8; 8]) -> Option<Change> {
    if data[0] != NOTIFICATION {
        return None;
    }
    Some(Change {
        channel: data[1],
        // Direction 0 is rising — liquid present. 1 is falling.
        liquid: data[2] == 0,
        // uint32 little-endian, unlike the 16-bit fields on the other board.
        since_ms: u32::from_le_bytes([data[3], data[4], data[5], data[6]]),
    })
}

/// What the board is reporting, mirrored to every screen.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct SensorState {
    /// The rig is configured to have a sensor board at all.
    pub enabled: bool,
    /// The board is answering.
    pub connected: bool,
    pub error: Option<String>,
    /// Liquid present, per channel. `None` until the board has answered once.
    pub liquid: Option<[bool; CHANNELS]>,
    /// Raw reading per channel, lower being more liquid.
    pub adc: Option<[u8; CHANNELS]>,
    /// The board's own switching thresholds, when it has been asked.
    pub max_threshold: Option<u8>,
    pub min_threshold: Option<u8>,
    pub last_seen: Option<f64>,
}

impl SensorState {
    /// Whether liquid is on `channel`, or `None` when nothing is known — the
    /// board is silent, or the channel is out of range.
    pub fn channel(&self, channel: u8) -> Option<bool> {
        if !self.connected {
            return None;
        }
        self.liquid?.get(channel as usize).copied()
    }

    pub fn go_offline(&mut self, why: Option<String>) {
        self.connected = false;
        self.error = why;
        self.liquid = None;
        self.adc = None;
    }

    /// The raw reading on `channel`, if the board has given one.
    pub fn adc_at(&self, channel: u8) -> Option<u8> {
        if !self.connected {
            return None;
        }
        self.adc?.get(channel as usize).copied()
    }

}

/// How a sensor reads on the diagram.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reading {
    /// No board, or nothing known yet.
    Unknown,
    Liquid,
    Dry,
}

impl Reading {
    pub fn of(state: &SensorState, channel: u8) -> Self {
        match state.channel(channel) {
            None => Reading::Unknown,
            Some(true) => Reading::Liquid,
            Some(false) => Reading::Dry,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Reading::Unknown => "—",
            Reading::Liquid => "liquid",
            Reading::Dry => "dry",
        }
    }
}

/// Keeps the last few changes, for the log and for a tooltip.
#[derive(Clone, Debug, Default)]
pub struct Changes(VecDeque<Change>);

impl Changes {
    pub fn push(&mut self, change: Change) {
        self.0.push_back(change);
        while self.0.len() > 32 {
            self.0.pop_front();
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &Change> {
        self.0.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_request_asks_for_the_states_and_nothing_else() {
        let f = request_states();
        assert_eq!(f[0], 0x00, "op 0 shifted up, read bit clear");
        assert_eq!(f[1..], [0u8; 7], "the rest of the frame is unused");
    }

    #[test]
    fn a_reply_reads_six_channels() {
        // Answer to op 0: low bit set, then one byte per detector.
        let frame = [0x01, 1, 0, 1, 0, 0, 1, 0];
        let states = parse_states(&frame).expect("this is a states reply");
        assert_eq!(states, [true, false, true, false, false, true]);
    }

    /// The adapter sees its own request as well as the answer. Reading the
    /// echo as data would report every detector dry the instant we asked.
    #[test]
    fn the_echo_of_our_own_request_is_not_a_reply() {
        assert_eq!(parse_states(&request_states()), None);
    }

    #[test]
    fn other_frames_are_left_alone() {
        // A reply to a different op code (raw ADC, op 3).
        let other = [(0x03 << 1) | 1, 9, 9, 9, 9, 9, 9, 0];
        assert_eq!(parse_states(&other), None);
        // A change notification is not a states reply.
        let note = [NOTIFICATION, 2, 0, 0, 0, 0, 0, 0];
        assert_eq!(parse_states(&note), None);
    }

    #[test]
    fn a_change_notification_says_which_way_it_went() {
        // Channel 2, direction 0 (rising = liquid), 500 ms since last change.
        let mut frame = [NOTIFICATION, 2, 0, 0, 0, 0, 0, 0];
        frame[3..7].copy_from_slice(&500u32.to_le_bytes());
        let change = parse_change(&frame).expect("this is a notification");
        assert_eq!(change, Change { channel: 2, liquid: true, since_ms: 500 });

        // Direction 1 is falling — the liquid left.
        frame[2] = 1;
        assert!(!parse_change(&frame).unwrap().liquid);
    }

    /// Reading sensor states on 0x700 would not merely fail: 0x700 is the
    /// Peltier controller's command id, so every poll would hand the heater a
    /// frame whose first byte it reads as a command.
    #[test]
    fn the_sensor_id_must_not_be_the_temperature_board() {
        assert!(collides_with_temperature_board(0x700), "the Peltier's command id");
        assert!(collides_with_temperature_board(0x710), "its telemetry");
        assert!(!collides_with_temperature_board(DEFAULT_DEVICE_ID));
        assert_eq!(DEFAULT_DEVICE_ID, 0x7FF, "what the boards on this rig answer on");
    }

    #[test]
    fn a_states_reply_is_not_a_change_notification() {
        assert_eq!(parse_change(&[0x01, 1, 0, 1, 0, 0, 1, 0]), None);
    }

    /// The raw reading is carried, not interpreted.
    ///
    /// An earlier version of this file turned it into a "margin from the
    /// switching level", which assumed every channel crosses in the same
    /// direction. Aspirating a front back past the detectors showed A5 turning
    /// wet as it fell under ~30 while A4 read wet sitting at 251, so that
    /// assumption was wrong and the conversion is gone. The state bit decides
    /// wet or dry; the raw value only says how settled it is.
    #[test]
    fn the_raw_reading_is_reported_without_being_interpreted() {
        let state = SensorState {
            enabled: true,
            connected: true,
            adc: Some([67, 104, 89, 114, 251, 19]),
            max_threshold: Some(115),
            min_threshold: Some(30),
            liquid: Some([false, false, false, false, true, true]),
            ..Default::default()
        };
        // Both of these read wet, at opposite ends of the range. Any rule that
        // maps one number to one state would have to call one of them a lie.
        assert_eq!(state.adc_at(4), Some(251));
        assert_eq!(state.adc_at(5), Some(19));
        assert_eq!(state.channel(4), Some(true));
        assert_eq!(state.channel(5), Some(true));
        // Nothing is readable once the board stops answering.
        let mut gone = state.clone();
        gone.go_offline(Some("unplugged".into()));
        assert_eq!(gone.adc_at(4), None);
    }

    #[test]
    fn a_raw_reply_is_told_from_a_states_reply() {
        let adc = [(OP_RAW_ADC << 1) | 1, 66, 104, 88, 114, 251, 252, 0];
        assert_eq!(parse_adc(&adc), Some([66, 104, 88, 114, 251, 252]));
        // The two ops share a frame shape, so each must refuse the other's.
        assert_eq!(parse_states(&adc), None);
        assert_eq!(parse_adc(&[0x01, 1, 0, 1, 0, 0, 1, 0]), None);
    }

    #[test]
    fn nothing_is_known_until_the_board_answers() {
        let mut state = SensorState { enabled: true, ..Default::default() };
        assert_eq!(Reading::of(&state, 4), Reading::Unknown);

        state.connected = true;
        state.liquid = Some([false, false, true, false, true, false]);
        assert_eq!(Reading::of(&state, 4), Reading::Liquid);
        assert_eq!(Reading::of(&state, 0), Reading::Dry);
        // A channel this board does not have.
        assert_eq!(Reading::of(&state, 9), Reading::Unknown);

        // A board that stops answering must not keep showing its last reading.
        state.go_offline(Some("timeout".into()));
        assert_eq!(Reading::of(&state, 4), Reading::Unknown);
    }
}
