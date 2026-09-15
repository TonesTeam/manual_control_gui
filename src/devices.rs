//! Components of the Tones liquid processing rig, as drawn in
//! `Tones_Liqud_Processing.drawio`, with the datasheet figures we rely on.

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum DeviceId {
    Pp01,
    Pp02,
    Sv01,
    Sv02,
    Sv03,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Pump { has_solenoid: bool },
    Valve { ports: u16 },
}

impl DeviceId {
    pub const ALL: [DeviceId; 5] = [
        DeviceId::Pp01,
        DeviceId::Pp02,
        DeviceId::Sv01,
        DeviceId::Sv02,
        DeviceId::Sv03,
    ];

    pub fn tag(self) -> &'static str {
        match self {
            DeviceId::Pp01 => "PP01",
            DeviceId::Pp02 => "PP02",
            DeviceId::Sv01 => "SV01",
            DeviceId::Sv02 => "SV02",
            DeviceId::Sv03 => "SV03",
        }
    }

    pub fn role(self) -> &'static str {
        match self {
            DeviceId::Pp01 => "Main pump (solenoid piston pump)",
            DeviceId::Pp02 => "Drain pump (piston pump)",
            DeviceId::Sv01 => "Reagent / wash selector",
            DeviceId::Sv02 => "Main selector (holding coil)",
            DeviceId::Sv03 => "Drain selector",
        }
    }

    pub fn kind(self) -> Kind {
        match self {
            DeviceId::Pp01 => Kind::Pump { has_solenoid: true },
            DeviceId::Pp02 => Kind::Pump { has_solenoid: false },
            DeviceId::Sv01 | DeviceId::Sv03 => Kind::Valve { ports: 8 },
            DeviceId::Sv02 => Kind::Valve { ports: 16 },
        }
    }

    pub fn is_pump(self) -> bool {
        matches!(self.kind(), Kind::Pump { .. })
    }

    /// RS485 slave addresses used by controller_v2 (`transport_control.rs`).
    pub fn default_addr(self) -> u8 {
        match self {
            DeviceId::Pp01 => 1,
            DeviceId::Pp02 => 2,
            DeviceId::Sv01 => 3,
            DeviceId::Sv02 => 4,
            DeviceId::Sv03 => 5,
        }
    }

    pub fn spec(self) -> &'static Spec {
        match self {
            DeviceId::Pp01 => &PP01_SPEC,
            DeviceId::Pp02 => &PP02_SPEC,
            DeviceId::Sv01 | DeviceId::Sv03 => &SV_8_SPEC,
            DeviceId::Sv02 => &SV_16_SPEC,
        }
    }

    /// What is plumbed to a valve port according to the diagram.
    /// Port 0 is the reset position: the common port is disconnected.
    pub fn port_label(self, port: u16) -> &'static str {
        if port == 0 {
            return "Reset (common closed)";
        }
        match self {
            DeviceId::Sv01 => match port {
                1 => "C1 (1 L)",
                2 => "C2 (1 L)",
                3 => "C3 (1 L)",
                6 => "Router Wash",
                7 => "Waste",
                4 | 5 | 8 => "spare",
                _ => "?",
            },
            DeviceId::Sv02 => match port {
                1 => "C1 (1 L)",
                2 => "C2 (1 L)",
                3 => "C3 (1 L)",
                4 => "C4 (0.5 L)",
                5 => "C5 (0.5 L)",
                6 => "C6 (0.5 L)",
                7 => "Router",
                8 => "Danger Waste",
                9 => "Waste",
                10..=15 => SLOT_NAMES[(16 - port) as usize - 1],
                16 => "Air filter",
                _ => "?",
            },
            DeviceId::Sv03 => match port {
                1..=6 => SLOT_NAMES[(7 - port) as usize - 1],
                7 => "Waste",
                8 => "Danger Waste",
                _ => "?",
            },
            _ => "",
        }
    }
}

const SLOT_NAMES: [&str; 6] = [
    "Slot 1 (S3)",
    "Slot 2 (S4)",
    "Slot 3 (S5)",
    "Slot 4 (S6)",
    "Slot 5 (S7)",
    "Slot 6 (S8)",
];

/// SV02 port feeding slot `slot` (1..=6).
pub fn sv02_port_for_slot(slot: u16) -> u16 {
    16 - slot
}

/// SV03 port draining slot `slot` (1..=6).
pub fn sv03_port_for_slot(slot: u16) -> u16 {
    7 - slot
}

/// Slot fed by an SV02 port, if it is a slot port.
pub fn slot_for_sv02_port(port: u16) -> Option<u16> {
    (10..=15).contains(&port).then(|| 16 - port)
}

/// Slot drained by an SV03 port, if it is a slot port.
pub fn slot_for_sv03_port(port: u16) -> Option<u16> {
    (1..=6).contains(&port).then(|| 7 - port)
}

pub struct Spec {
    pub part_number: &'static str,
    pub model: &'static str,
    pub source: &'static str,
    pub source_url: &'static str,
    pub rows: &'static [(&'static str, &'static str)],
    pub notes: &'static [&'static str],
}

pub static SV_8_SPEC: Spec = Spec {
    part_number: "QHF-SV07-X-S-T08-K1.2-S",
    model: "Runze SV-07 selector valve, 8 port, 1.2 mm orifice, sapphire rotor/stator",
    source: "Runze multiport selector valve datasheet",
    source_url: "https://www.runzefluid.com/uploads/file/multiport-selector-valve.pdf",
    rows: &[
        ("Configuration", "8 ports + center common port"),
        ("Orifice", "1.2 mm"),
        ("Port-to-port volume", "27.5 µL"),
        ("Dead volume", "5.41 µL"),
        ("Pressure rating", "0–1.0 MPa (air) / 0–1.6 MPa (water)"),
        ("Switching", "≤2 s per revolution, shortest path"),
        ("Wetted material", "UPE, sapphire rotor/stator"),
        ("Liquid temperature", "0–150 °C"),
        ("Connection", "1/4-28 UNF"),
        ("Interface", "RS232 / RS485 / CAN"),
        ("Baud rate", "9600 (default) … 115200 bps"),
        ("Power", "DC 24 V / 3 A, 60 W max"),
        ("Operating temperature", "−10…50 °C, ≤80 % RH"),
        ("Dimensions / weight", "60 × 51 × 150 mm, 0.73 kg"),
    ],
    notes: &[
        "Resets CCW; after reset the rotor sits between port 1 and the last port and the common port is closed.",
        "Origin detection on power-up can be enabled or disabled.",
    ],
};

pub static SV_16_SPEC: Spec = Spec {
    part_number: "QHF-SV07-X-S-T16-K1.0-S",
    model: "Runze SV-07 selector valve, 16 port, 1.0 mm orifice, sapphire rotor/stator",
    source: "Runze multiport selector valve datasheet",
    source_url: "https://www.runzefluid.com/uploads/file/multiport-selector-valve.pdf",
    rows: &[
        ("Configuration", "16 ports + center common port"),
        ("Orifice", "1.0 mm"),
        ("Port-to-port volume", "28.564 µL"),
        ("Dead volume", "5.006 µL"),
        ("Pressure rating", "0–1.0 MPa (air) / 0–1.6 MPa (water)"),
        ("Switching", "≤3.3 s per revolution, shortest path"),
        ("Wetted material", "UPE, sapphire rotor/stator"),
        ("Liquid temperature", "0–150 °C"),
        ("Connection", "1/4-28 UNF"),
        ("Interface", "RS232 / RS485 / CAN"),
        ("Baud rate", "9600 (default) … 115200 bps"),
        ("Power", "DC 24 V / 3 A, 60 W max"),
        ("Operating temperature", "−10…50 °C, ≤80 % RH"),
        ("Dimensions / weight", "60 × 51 × 179.5 mm, 1.02 kg"),
    ],
    notes: &[
        "Resets CCW; after reset the common port is closed.",
        "No spare ports on this rig: 6 reagents, router, 2 wastes, 6 slots, air.",
    ],
};

pub static PP01_SPEC: Spec = Spec {
    part_number: "ZSB-LS-1.8-1-3-M-Q",
    model: "Runze RP-01 family piston pump, 1.8° stepper, encoder (M), integrated driver (Q)",
    source: "Runze RP-01 Piston Pump manual v1.1",
    source_url: "https://www.runzefluid.com/uploads/file/rp-01-piston-pump-v1-1.pdf",
    rows: &[
        ("Accuracy", "≤1 % (100 % stroke)"),
        ("Precision (repeatability)", "0.3–0.7 % (100 % stroke)"),
        ("Rated stroke", "19.1 mm = 3820 steps (self-defined protocol)"),
        ("Resolution", "0.005 mm = 1.5707 µL/step (6 mL head)"),
        ("Max speed", "500 rpm; 2.2 s – 1146 s per full stroke"),
        ("Cylinder / actuator", "ID 20 mm, lead screw 1 mm"),
        ("Pressure", "+0.8 MPa / −0.06 MPa (1 min hold)"),
        ("Solenoid valve", "PVR16-1E-PSDC24V, 24 V ±10 %, 154 mA start / 42 mA hold, −0.075…0.2 MPa"),
        ("Dead volume", "1.716 mL (double-hole head with valve)"),
        ("Wetted material", "PC, ceramics, PTFE, PEEK, EPDM, FKM"),
        ("Home detection", "Photoelectric, piston origin"),
        ("Service life", "3 million strokes without leakage (water)"),
        ("Connection", "1/4-28 UNF"),
        ("Power", "DC 24 V / 1.5 A"),
        ("Operating temperature", "5…55 °C, <80 % RH"),
    ],
    notes: &[
        "Datasheet naming puts volume in the third field, so '-1-3-' reads as a 3 mL head; the 6 mL figures above are the RP-01 reference. Verify the label.",
        "The part number has no '-F' (solenoid) suffix, but the diagram and controller_v2 drive a solenoid via 0x60/0x61 (IO3, MC12M driver only). Verify the fitted driver.",
        "controller_v2 calibrates this pump at 2.083 µL/step over 3840 steps; the pump itself rejects moves beyond 3820 steps (0x0EEC). Both values are editable in Settings.",
    ],
};

pub static PP02_SPEC: Spec = Spec {
    part_number: "ZSB08-LS-0.9-1-5-1-Q",
    model: "Runze SY-08 syringe pump, 0.9° stepper, 1 mm lead screw, 5 mL, single channel, integrated driver",
    source: "Runze SY-08 datasheet",
    source_url: "https://www.runzefluid.com/uploads/file/sy-08.pdf",
    rows: &[
        ("Syringe", "5 mL, borosilicate glass, PCTFE valve head, PTFE piston"),
        ("Accuracy", "≤1 % (100 % rated stroke)"),
        ("Precision (repeatability)", "0.3–0.7 % (100 % rated stroke)"),
        ("Rated stroke", "30 mm = 12000 steps"),
        ("Resolution", "0.0025 mm = 0.416 µL/step"),
        ("Max speed", "600 rpm; 0.017–10 mm/s; 3–1800 s per full stroke"),
        ("Back pressure", "0.95 MPa"),
        ("Service life", "3 million strokes without leakage (water)"),
        ("Connection", "1/4-28 UNF"),
        ("Interface", "RS232 / RS485 / CAN, same 0xCC…0xDD frame"),
        ("Baud rate", "9600 (default) … 115200 bps"),
        ("Power", "DC 24 V / 3 A"),
        ("Operating temperature", "5…55 °C, ≤80 % RH"),
        ("Dimensions", "42 × 42 × 192.8 mm"),
    ],
    notes: &[
        "Datasheet stroke and resolution match controller_v2's calibration (12000 steps, 0.416 µL/step).",
        "This pump accepts 1–600 rpm (0x4B up to 0x0258); the GUI currently limits both pumps to 1–500 rpm.",
    ],
};
