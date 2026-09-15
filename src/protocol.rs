//! Runze Fluid "self-defined" 8-byte serial protocol, shared by the SV-07
//! selector valves and the RP-01 / SY-08 piston pumps (RS232/RS485/CAN).
//!
//! Request:  CC ADDR FUNC P_LO P_HI DD SUM_LO SUM_HI
//! Response: CC ADDR STAT V_LO V_HI DD SUM_LO SUM_HI
//! SUM is the 16-bit sum of bytes 0..=5, little-endian.
//! Source: RP-01 Piston Pump manual v1.1 §2.2.3, SY-01B manual v1.0 §2.3.

pub const STX: u8 = 0xCC;
pub const ETX: u8 = 0xDD;

// Query commands
pub const Q_ADDRESS: u8 = 0x20;
pub const Q_CHANNEL: u8 = 0x3E; // current valve channel / port
pub const Q_VERSION: u8 = 0x3F;
pub const Q_MOTOR_STATUS: u8 = 0x4A;
pub const Q_PISTON_POS: u8 = 0x66;
pub const SYNC_PISTON_POS: u8 = 0x67;

// Action commands
pub const PUMP_DISPENSE: u8 = 0x42; // CW, relative steps (towards home)
pub const VALVE_SWITCH: u8 = 0x44; // optimal-path switch to port
pub const RESET: u8 = 0x45; // pump home / valve reset
pub const STOP: u8 = 0x49;
pub const SET_SPEED: u8 = 0x4B; // 1..500 rpm (pump)
pub const VALVE_RESET: u8 = 0x4C;
pub const PUMP_ASPIRATE: u8 = 0x4D; // CCW, relative steps (away from home)
pub const PUMP_ABS: u8 = 0x4E; // absolute position
pub const FORCED_RESET: u8 = 0x4F;
pub const SOLENOID_HIGH: u8 = 0x60; // IO3 high (MC12M / MC10+MOS only)
pub const SOLENOID_LOW: u8 = 0x61; // IO3 low

pub fn checksum(bytes: &[u8]) -> [u8; 2] {
    let sum: u16 = bytes.iter().fold(0u16, |acc, &b| acc.wrapping_add(b as u16));
    [(sum & 0xFF) as u8, (sum >> 8) as u8]
}

pub fn encode(addr: u8, func: u8, param: u16) -> [u8; 8] {
    let mut f = [STX, addr, func, (param & 0xFF) as u8, (param >> 8) as u8, ETX, 0, 0];
    let cs = checksum(&f[..6]);
    f[6] = cs[0];
    f[7] = cs[1];
    f
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reply {
    pub addr: u8,
    pub status: u8,
    pub value: u16,
}

pub fn decode(buf: &[u8; 8]) -> Result<Reply, String> {
    if buf[0] != STX || buf[5] != ETX {
        return Err(format!("bad frame {:02X?}", buf));
    }
    let cs = checksum(&buf[..6]);
    if cs != [buf[6], buf[7]] {
        return Err(format!("checksum mismatch {:02X?}", buf));
    }
    Ok(Reply {
        addr: buf[1],
        status: buf[2],
        value: u16::from_le_bytes([buf[3], buf[4]]),
    })
}

/// Motor status codes (B2 of the response frame).
pub fn status_text(code: u8) -> &'static str {
    match code {
        0x00 => "Normal",
        0x01 => "Frame error",
        0x02 => "Parameter error",
        0x03 => "Optocoupler error",
        0x04 => "Motor busy",
        0x05 => "Motor stalled",
        0x06 => "Unknown position",
        0x07 => "Command rejected",
        0x08 => "Illegal location",
        0xFE => "Task running",
        0xFF => "Unknown error",
        _ => "Unrecognized",
    }
}

pub fn status_is_fault(code: u8) -> bool {
    !matches!(code, 0x00 | 0x04 | 0xFE)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_matches_controller_frames() {
        // controller_v2: [CC, addr, 0x44, lo, hi, DD] + sum(lo, hi)
        let f = encode(4, VALVE_SWITCH, 16);
        assert_eq!(f[..6], [0xCC, 4, 0x44, 16, 0, 0xDD]);
        let sum = 0xCCu16 + 4 + 0x44 + 16 + 0xDD;
        assert_eq!([f[6], f[7]], [(sum & 0xFF) as u8, (sum >> 8) as u8]);
    }

    #[test]
    fn roundtrip() {
        let f = encode(2, 0xFE, 3820);
        let r = decode(&f).unwrap();
        assert_eq!(r, Reply { addr: 2, status: 0xFE, value: 3820 });
    }

    #[test]
    fn rejects_bad_checksum() {
        let mut f = encode(1, Q_PISTON_POS, 0);
        f[7] ^= 1;
        assert!(decode(&f).is_err());
    }
}
