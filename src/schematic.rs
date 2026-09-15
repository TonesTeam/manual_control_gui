//! Live fluidic schematic, laid out after `Tones_Liqud_Processing.drawio`.
//! Every component can be dragged in "move objects" mode. Tubes are routed
//! orthogonally from the component positions: stubs leave each port along an
//! aligned column or row, parallel runs sit on evenly spaced lanes, and
//! corners are rounded.

use std::collections::BTreeMap;

use eframe::egui::{self, Align2, Color32, CornerRadius, CursorIcon, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2, pos2, vec2};
use serde::{Deserialize, Serialize};

use crate::bus::{BusState, DevState};
use crate::config::Settings;
use crate::devices::{DeviceId, sv02_port_for_slot, sv03_port_for_slot};
use crate::tracking::{BOTTLE_CAPACITY_ML, BottleState, FluidView, SlotState};

const W: f32 = 1760.0;
const H: f32 = 1000.0;
const GRID: f32 = 10.0;
/// Spacing between parallel tubes.
const LANE: f32 = 9.0;
const LAYOUT_FILE: &str = "tstand_layout_v2.json";

/// Default centers in virtual coordinates, in hit-test order (later = on top).
const DEFAULTS: &[(&str, (f32, f32))] = &[
    ("TC01", (110.0, 190.0)),
    ("TC02", (110.0, 600.0)),
    ("C1", (260.0, 700.0)),
    ("C2", (380.0, 700.0)),
    ("C3", (500.0, 700.0)),
    ("C4", (260.0, 880.0)),
    ("C5", (380.0, 880.0)),
    ("C6", (500.0, 880.0)),
    ("SLOT1", (1420.0, 170.0)),
    ("SLOT2", (1535.0, 170.0)),
    ("SLOT3", (1650.0, 170.0)),
    ("SLOT4", (1420.0, 560.0)),
    ("SLOT5", (1535.0, 560.0)),
    ("SLOT6", (1650.0, 560.0)),
    ("COIL", (860.0, 400.0)),
    ("SV01", (330.0, 400.0)),
    ("SV02", (1200.0, 400.0)),
    ("SV03", (1060.0, 870.0)),
    ("PP01", (620.0, 400.0)),
    ("PP02", (830.0, 870.0)),
    ("AIR", (1200.0, 180.0)),
    ("S1", (730.0, 400.0)),
    ("S2", (990.0, 400.0)),
    ("TAG_ROUTER_WASH", (480.0, 330.0)),
    ("TAG_SV01_WASTE", (470.0, 470.0)),
    ("TAG_ROUTER", (1070.0, 590.0)),
    ("TAG_SV02_DANGER", (1200.0, 650.0)),
    ("TAG_SV02_WASTE", (1280.0, 590.0)),
    ("TAG_SV03_WASTE", (1200.0, 920.0)),
    ("TAG_SV03_DANGER", (940.0, 975.0)),
];

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Layout {
    pub nodes: BTreeMap<String, [f32; 2]>,
}

impl Default for Layout {
    fn default() -> Self {
        let nodes = DEFAULTS.iter().map(|(k, p)| (k.to_string(), [p.0, p.1])).collect();
        Self { nodes }
    }
}

impl Layout {
    pub fn pos(&self, key: &str) -> Pos2 {
        self.nodes
            .get(key)
            .map(|p| pos2(p[0], p[1]))
            .or_else(|| DEFAULTS.iter().find(|(k, _)| *k == key).map(|(_, p)| pos2(p.0, p.1)))
            .unwrap_or(pos2(W * 0.5, H * 0.5))
    }

    fn set(&mut self, key: &str, p: Pos2) {
        let p = pos2(p.x.clamp(0.0, W), p.y.clamp(0.0, H));
        self.nodes.insert(key.to_string(), [p.x, p.y]);
    }

    /// Puts one component back where the drawio layout has it.
    pub fn reset(&mut self, key: &str) {
        if let Some((_, p)) = DEFAULTS.iter().find(|(k, _)| *k == key) {
            self.nodes.insert(key.to_string(), [p.0, p.1]);
        }
    }

    pub fn load() -> Self {
        let mut layout = std::fs::read_to_string(crate::config::data_file(LAYOUT_FILE))
            .ok()
            .and_then(|t| serde_json::from_str::<Layout>(&t).ok())
            .unwrap_or_default();
        for (k, p) in DEFAULTS {
            layout.nodes.entry(k.to_string()).or_insert([p.0, p.1]);
        }
        layout
    }

    pub fn save(&self) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        std::fs::write(crate::config::data_file(LAYOUT_FILE), text).map_err(|e| e.to_string())
    }
}

pub struct Palette {
    pub bg: Color32,
    pub grid: Color32,
    pub line: Color32,
    pub bundle: Color32,
    pub text: Color32,
    pub dim: Color32,
    pub flow: Color32,
    pub flow_idle: Color32,
    pub ok: Color32,
    pub busy: Color32,
    pub fault: Color32,
    pub offline: Color32,
    pub panel: Color32,
    pub liquid: Color32,
    pub tag: Color32,
    pub tag_text: Color32,
}

impl Palette {
    pub fn for_ui(ui: &egui::Ui) -> Self {
        if ui.visuals().dark_mode {
            Self {
                bg: Color32::from_rgb(24, 26, 30),
                grid: Color32::from_rgb(44, 47, 54),
                line: Color32::from_rgb(150, 155, 165),
                bundle: Color32::from_rgb(88, 92, 100),
                text: Color32::from_rgb(225, 228, 232),
                dim: Color32::from_rgb(130, 135, 145),
                flow: Color32::from_rgb(64, 170, 255),
                flow_idle: Color32::from_rgb(52, 120, 180),
                ok: Color32::from_rgb(80, 190, 110),
                busy: Color32::from_rgb(240, 180, 60),
                fault: Color32::from_rgb(235, 80, 80),
                offline: Color32::from_rgb(95, 98, 105),
                panel: Color32::from_rgb(36, 39, 45),
                liquid: Color32::from_rgb(60, 120, 200),
                tag: Color32::from_rgb(120, 100, 40),
                tag_text: Color32::from_rgb(245, 240, 225),
            }
        } else {
            Self {
                bg: Color32::from_rgb(250, 250, 248),
                grid: Color32::from_rgb(228, 229, 232),
                line: Color32::from_rgb(70, 72, 78),
                bundle: Color32::from_rgb(185, 188, 194),
                text: Color32::from_rgb(25, 27, 30),
                dim: Color32::from_rgb(120, 124, 130),
                flow: Color32::from_rgb(0, 115, 220),
                flow_idle: Color32::from_rgb(90, 150, 210),
                ok: Color32::from_rgb(30, 150, 70),
                busy: Color32::from_rgb(210, 140, 0),
                fault: Color32::from_rgb(210, 40, 40),
                offline: Color32::from_rgb(170, 172, 178),
                panel: Color32::from_rgb(255, 255, 255),
                liquid: Color32::from_rgb(170, 205, 245),
                tag: Color32::from_rgb(255, 236, 170),
                tag_text: Color32::BLACK,
            }
        }
    }
}

#[derive(Default)]
pub struct SchematicResponse {
    pub clicked_device: Option<DeviceId>,
    pub clicked_port: Option<(DeviceId, u16)>,
    pub layout_changed: bool,
    /// What was under the pointer when a right-click happened this frame.
    pub menu_target: Option<Target>,
    /// Responses that own the right-click menu: the canvas, plus every component in move mode.
    pub menu_responses: Vec<egui::Response>,
}

/// Something on the schematic a right-click can land on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    Device(DeviceId),
    Port(DeviceId, u16),
    Slot(u16),
    Bottle(u16),
    /// Waste / router tag, with the valve port that feeds it.
    Tag(&'static str, DeviceId, u16),
    /// Other layout components: TC blocks, sensors, coil, air filter.
    Part(&'static str),
    Background,
}

impl Target {
    /// Layout key of the component, for "Reset position".
    pub fn layout_key(&self) -> Option<String> {
        match *self {
            Target::Device(id) | Target::Port(id, _) => Some(id.tag().to_string()),
            Target::Slot(n) => Some(slot_key(n)),
            Target::Bottle(c) => Some(format!("C{c}")),
            Target::Tag(key, _, _) | Target::Part(key) => Some(key.to_string()),
            Target::Background => None,
        }
    }
}

/// Valve port that feeds a waste / router tag.
fn tag_port(key: &str) -> (DeviceId, u16) {
    match key {
        "TAG_ROUTER_WASH" => (DeviceId::Sv01, 6),
        "TAG_SV01_WASTE" => (DeviceId::Sv01, 7),
        "TAG_ROUTER" => (DeviceId::Sv02, 7),
        "TAG_SV02_DANGER" => (DeviceId::Sv02, 8),
        "TAG_SV02_WASTE" => (DeviceId::Sv02, 9),
        "TAG_SV03_WASTE" => (DeviceId::Sv03, 7),
        _ => (DeviceId::Sv03, 8),
    }
}

fn target_for_key(key: &'static str) -> Target {
    if let Some(id) = device_for_key(key) {
        return Target::Device(id);
    }
    if let Some(n) = key.strip_prefix("SLOT").and_then(|n| n.parse().ok()) {
        return Target::Slot(n);
    }
    if key.starts_with("TAG") {
        let (id, port) = tag_port(key);
        return Target::Tag(key, id, port);
    }
    if let Some(c) = key.strip_prefix('C').and_then(|n| n.parse().ok()) {
        return Target::Bottle(c);
    }
    Target::Part(key)
}

/// Topmost component at a virtual-canvas point, matching the drawn shapes.
fn target_at(l: &Layout, p: Pos2, big: f32) -> Target {
    for id in [DeviceId::Sv01, DeviceId::Sv02, DeviceId::Sv03] {
        let c = l.pos(id.tag());
        let (r, ports) = valve_geom(id);
        let port_r = if ports == 16 { 12.0 } else { 14.0 };
        if let Some(port) = (1..=ports).find(|&k| p.distance(port_pos(c, r, ports, k)) <= port_r) {
            return Target::Port(id, port);
        }
        if p.distance(c) <= r + 4.0 {
            return Target::Device(id);
        }
    }
    for id in [DeviceId::Pp01, DeviceId::Pp02] {
        let body = Rect::from_center_size(l.pos(id.tag()), vec2(90.0, 160.0)).expand(10.0);
        if body.contains(p) || pump_card_rect(l, id, big).contains(p) {
            return Target::Device(id);
        }
    }
    DEFAULTS
        .iter()
        .rev()
        .map(|(k, _)| *k)
        .find(|k| device_for_key(k).is_none() && hit_rect(k, l.pos(k)).contains(p))
        .map_or(Target::Background, target_for_key)
}

fn device_for_key(key: &str) -> Option<DeviceId> {
    DeviceId::ALL.into_iter().find(|id| id.tag() == key)
}

fn valve_geom(id: DeviceId) -> (f32, u16) {
    match id {
        DeviceId::Sv02 => (100.0, 16),
        _ => (70.0, 8),
    }
}

/// Port angles as drawn in the drawio: the 8-port valves run clockwise with
/// port 4 at the top (1 lower-left, 6 right, 8 bottom); the 16-port valve
/// runs counter-clockwise with 16 at the top (4 left, 8 bottom, 12 right).
fn port_angle(ports: u16, port: u16) -> f32 {
    let deg = if ports == 16 { -90.0 - 22.5 * port as f32 } else { -90.0 + 45.0 * (port as f32 - 4.0) };
    deg.to_radians()
}

fn port_pos(center: Pos2, r: f32, ports: u16, port: u16) -> Pos2 {
    let a = port_angle(ports, port);
    center + vec2(a.cos(), a.sin()) * r
}

/// Rim point halfway between two adjacent ports, where a common-port tube enters.
fn rim_between(center: Pos2, r: f32, ports: u16, a: u16, b: u16) -> Pos2 {
    let (ta, tb) = (port_angle(ports, a), port_angle(ports, b));
    let v = vec2(ta.cos() + tb.cos(), ta.sin() + tb.sin()).normalized();
    center + v * r
}

fn tag_text(key: &str) -> &'static str {
    match key {
        "TAG_ROUTER_WASH" => "Router Wash",
        "TAG_ROUTER" => "Router",
        "TAG_SV02_DANGER" | "TAG_SV03_DANGER" => "Danger Waste",
        _ => "Waste",
    }
}

fn tag_width(key: &str) -> f32 {
    tag_text(key).len() as f32 * 7.5 + 20.0
}

fn tag_rect(layout: &Layout, key: &str) -> Rect {
    Rect::from_center_size(layout.pos(key), vec2(tag_width(key), 26.0))
}

/// Hit-test rectangle of a movable component, in virtual coordinates.
fn hit_rect(key: &str, c: Pos2) -> Rect {
    let size = match key {
        "SV01" | "SV03" => vec2(170.0, 170.0),
        "SV02" => vec2(230.0, 230.0),
        "PP01" | "PP02" => vec2(120.0, 240.0),
        "COIL" => vec2(130.0, 130.0),
        "S1" | "S2" => vec2(30.0, 40.0),
        "TC01" | "TC02" => vec2(34.0, 200.0),
        "AIR" => vec2(70.0, 60.0),
        k if k.starts_with("TAG") => vec2(tag_width(k), 28.0),
        k if k.starts_with("SLOT") => return Rect::from_center_size(c + vec2(0.0, 16.0), vec2(100.0, 200.0)),
        k if k.starts_with('C') => vec2(80.0, 100.0),
        _ => vec2(40.0, 40.0),
    };
    Rect::from_center_size(c, size)
}

fn tc_channel(layout: &Layout, key: &str, k: usize, right: bool) -> Pos2 {
    let c = layout.pos(key);
    pos2(c.x + if right { 10.0 } else { -10.0 }, c.y - 85.0 + (k as f32 - 1.0) * 21.25)
}

/// Readout card for a pump, growing with the label size: under PP01, and to
/// the left of PP02 (top-aligned) so the two cards never meet.
fn pump_card_rect(layout: &Layout, id: DeviceId, big: f32) -> Rect {
    let c = layout.pos(id.tag());
    let size = vec2(140.0, 112.0) * big;
    let min = if id == DeviceId::Pp01 { c + vec2(-45.0, 95.0) } else { c + vec2(-55.0 - size.x, -80.0) };
    Rect::from_min_size(min, size)
}

fn slot_key(slot: u16) -> String {
    format!("SLOT{slot}")
}

/// Feed inlet: bottom of the slot's sensor, entered from below.
fn slot_inlet(layout: &Layout, slot: u16) -> Pos2 {
    layout.pos(&slot_key(slot)) + vec2(-20.0, 105.0)
}

/// Drain outlet on the slot's right side, and the column in the gap beside
/// the slot that the drain tube runs down. Upper-row slots use the outer
/// column so their tubes pass the lower-row outlets without crossing.
fn slot_outlet(layout: &Layout, slot: u16) -> (Pos2, f32) {
    let c = layout.pos(&slot_key(slot));
    (c + vec2(45.0, 60.0), c.x + if slot <= 3 { 62.0 } else { 52.0 })
}

/// Drops repeated points and merges collinear runs.
fn clean(pts: Vec<Pos2>) -> Vec<Pos2> {
    let mut out: Vec<Pos2> = Vec::with_capacity(pts.len());
    for p in pts {
        if out.last().is_some_and(|q| q.distance(p) < 0.5) {
            continue;
        }
        if out.len() >= 2 {
            let a = out[out.len() - 2];
            let b = out[out.len() - 1];
            let (u, v) = (b - a, p - b);
            let off_line = (u.x * v.y - u.y * v.x).abs() / u.length().max(1e-3);
            if off_line < 0.5 && u.dot(v) >= 0.0 {
                *out.last_mut().unwrap() = p;
                continue;
            }
        }
        out.push(p);
    }
    out
}

/// Replaces sharp corners with short quadratic arcs.
fn rounded(pts: &[Pos2], radius: f32) -> Vec<Pos2> {
    if pts.len() < 3 || radius < 0.5 {
        return pts.to_vec();
    }
    let mut out = vec![pts[0]];
    for i in 1..pts.len() - 1 {
        let (a, b, c) = (pts[i - 1], pts[i], pts[i + 1]);
        let (d1, d2) = (b - a, c - b);
        let (l1, l2) = (d1.length(), d2.length());
        if l1 < 1e-3 || l2 < 1e-3 {
            out.push(b);
            continue;
        }
        let r = radius.min(l1 * 0.5).min(l2 * 0.5);
        let p0 = b - d1 / l1 * r;
        let p2 = b + d2 / l2 * r;
        for k in 0..=4 {
            let t = k as f32 / 4.0;
            let q0 = p0 + (b - p0) * t;
            let q1 = b + (p2 - b) * t;
            out.push(q0 + (q1 - q0) * t);
        }
    }
    out.push(pts[pts.len() - 1]);
    out
}

/// First bend out of a port: along whichever axis the port faces most.
fn stub(center: Pos2, port: Pos2, len: f32) -> Pos2 {
    let d = port - center;
    if d.x.abs() >= d.y.abs() - 0.01 {
        pos2(port.x + d.x.signum() * len, port.y)
    } else {
        pos2(port.x, port.y + d.y.signum() * len)
    }
}

/// Continues a tube into the nearest edge of a tag: straight when already
/// level with it, otherwise with a single bend.
fn into_tag(mut pts: Vec<Pos2>, r: Rect) -> Vec<Pos2> {
    let from = *pts.last().unwrap();
    if from.x > r.min.x + 4.0 && from.x < r.max.x - 4.0 {
        pts.push(pos2(from.x, if from.y < r.center().y { r.min.y } else { r.max.y }));
    } else {
        let x = if from.x < r.center().x { r.min.x } else { r.max.x };
        if from.y > r.min.y + 4.0 && from.y < r.max.y - 4.0 {
            pts.push(pos2(x, from.y));
        } else {
            pts.push(pos2(from.x, r.center().y));
            pts.push(pos2(x, r.center().y));
        }
    }
    pts
}

/// Horizontal run between two inline parts, with a grid-aligned jog if their rows differ.
fn inline(a: Pos2, b: Pos2) -> Vec<Pos2> {
    if (a.y - b.y).abs() < 0.5 {
        vec![a, b]
    } else {
        let mx = ((a.x + b.x) * 0.5 / GRID).round() * GRID;
        vec![a, pos2(mx, a.y), pos2(mx, b.y), b]
    }
}

struct Edge {
    pts: Vec<Pos2>,
    active: bool,
    /// +1: pump pushes along pts order, -1: pump pulls against it, 0: idle.
    dir: i8,
}

/// All tubes connected to the pumps and valves, ordered away from the pump
/// that drives them so flow direction can be animated.
fn route_tubes(l: &Layout, state: &BusState) -> Vec<Edge> {
    let dev = |id: DeviceId| state.dev(id);
    let sv01 = l.pos("SV01");
    let sv02 = l.pos("SV02");
    let sv03 = l.pos("SV03");
    let pp01 = l.pos("PP01");
    let pp02 = l.pos("PP02");
    let (r01, n01) = valve_geom(DeviceId::Sv01);
    let (r02, n02) = valve_geom(DeviceId::Sv02);
    let (r03, n03) = valve_geom(DeviceId::Sv03);

    let pump01 = dev(DeviceId::Pp01);
    let pump02 = dev(DeviceId::Pp02);
    let input = pump01.solenoid_input.unwrap_or(false);
    // Aspirating (position rising) pulls liquid towards the pump.
    let dir01 = -pump01.motion;
    let dir02 = -pump02.motion;
    let coil_side = if input { 0 } else { dir01 };
    let wash_side = if input { dir01 } else { 0 };
    let at = |id: DeviceId| dev(id).value.filter(|p| *p > 0);
    let (a01, a02, a03) = (at(DeviceId::Sv01), at(DeviceId::Sv02), at(DeviceId::Sv03));
    let on01 = |p: u16| (input && a01 == Some(p), if a01 == Some(p) { wash_side } else { 0 });
    let on02 = |p: u16| (!input && a02 == Some(p), if a02 == Some(p) { coil_side } else { 0 });
    let on03 = |p: u16| (a03 == Some(p), if a03 == Some(p) { dir02 } else { 0 });

    let mut edges = Vec::new();
    let mut push = |pts: Vec<Pos2>, (active, dir): (bool, i8)| edges.push(Edge { pts: clean(pts), active, dir });

    // PP01 <-> SV01 common, entering the rim between ports 7 and 8.
    {
        let e = rim_between(sv01, r01, n01, 7, 8);
        let pin = pp01 - vec2(45.0, 0.0);
        let col = pin.x - 18.0;
        let low = (sv01.y + r01 + 34.0).max(pin.y);
        push(vec![pin, pos2(col, pin.y), pos2(col, low), pos2(e.x, low), e, sv01], (input, wash_side));
    }

    // PP01 -> S1 -> holding coil -> S2 -> SV02 common (rim between ports 5 and 6).
    let (s1, coil, s2) = (l.pos("S1"), l.pos("COIL"), l.pos("S2"));
    let coil_path = (!input, coil_side);
    push(inline(pp01 + vec2(45.0, 0.0), s1 - vec2(11.0, 0.0)), coil_path);
    push(inline(s1 + vec2(11.0, 0.0), coil - vec2(65.0, 0.0)), coil_path);
    push(inline(coil + vec2(65.0, 0.0), s2 - vec2(11.0, 0.0)), coil_path);
    let sv02_left_lane = |k: u16| sv02.x - r02 - 24.0 - (k - 1) as f32 * LANE;
    {
        let e = rim_between(sv02, r02, n02, 5, 6);
        let a = s2 + vec2(11.0, 0.0);
        let jog = (sv02_left_lane(6) - 14.0).max(a.x + 8.0);
        push(vec![a, pos2(jog, a.y), pos2(jog, e.y), e, sv02], coil_path);
    }

    // PP02 -> SV03 common (rim between ports 8 and 1).
    {
        let e = rim_between(sv03, r03, n03, 8, 1);
        let side = if e.x >= pp02.x { 45.0 } else { -45.0 };
        let y = e.y.clamp(pp02.y - 70.0, pp02.y + 70.0);
        push(vec![pos2(pp02.x + side, y), pos2(e.x, y), e, sv03], (true, dir02));
    }

    // SV01 ports 1..3 <- TC01 channels 7..9 on stacked lanes left of the valve.
    for p in 1u16..=3 {
        let port = port_pos(sv01, r01, n01, p);
        let x = sv01.x - r01 - 24.0 - (3 - p) as f32 * LANE;
        let ch = tc_channel(l, "TC01", 6 + p as usize, true);
        push(vec![port, pos2(x, port.y), pos2(x, ch.y), ch], on01(p));
    }
    // SV01 6 -> Router Wash, 7 -> Waste.
    for (i, (p, key)) in [(6u16, "TAG_ROUTER_WASH"), (7, "TAG_SV01_WASTE")].into_iter().enumerate() {
        let port = port_pos(sv01, r01, n01, p);
        let x = sv01.x + r01 + 16.0 + i as f32 * LANE;
        push(into_tag(vec![port, pos2(x, port.y)], tag_rect(l, key)), on01(p));
    }

    // SV02 ports 1..6 <- TC01 channels 1..6: port 1 takes the inner lane and the top row.
    for p in 1u16..=6 {
        let port = port_pos(sv02, r02, n02, p);
        let x = sv02_left_lane(p);
        let ch = tc_channel(l, "TC01", p as usize, true);
        push(vec![port, pos2(x, port.y), pos2(x, ch.y), ch], on02(p));
    }
    // SV02 7 -> Router, 8 -> Danger Waste, 9 -> Waste, straight down first.
    for (p, key) in [(7u16, "TAG_ROUTER"), (8, "TAG_SV02_DANGER"), (9, "TAG_SV02_WASTE")] {
        let port = port_pos(sv02, r02, n02, p);
        push(into_tag(vec![port, pos2(port.x, sv02.y + r02 + 16.0)], tag_rect(l, key)), on02(p));
    }
    // SV02 10..15 -> slots. Inlets above the port get a single bend; the rest
    // share lanes right of the valve and rows under the inlets, nested so they
    // don't cross each other.
    {
        let mut below = Vec::new();
        for slot in 1u16..=6 {
            let p = sv02_port_for_slot(slot);
            let port = port_pos(sv02, r02, n02, p);
            let inlet = slot_inlet(l, slot);
            if inlet.y < port.y - 12.0 {
                push(vec![port, pos2(inlet.x, port.y), inlet], on02(p));
            } else {
                below.push((p, port, inlet));
            }
        }
        below.sort_by(|a, b| a.2.x.total_cmp(&b.2.x));
        let n = below.len();
        let base_y = below.iter().map(|b| b.2.y).fold(f32::MIN, f32::max);
        for (rank, (p, port, inlet)) in below.into_iter().enumerate() {
            let x = sv02.x + r02 + 24.0 + (n - 1 - rank) as f32 * LANE;
            let y = base_y + 15.0 + rank as f32 * LANE;
            push(vec![port, pos2(x, port.y), pos2(x, y), pos2(inlet.x, y), inlet], on02(p));
        }
    }
    // SV02 16 -> air filter.
    {
        let port = port_pos(sv02, r02, n02, 16);
        let air = l.pos("AIR") + vec2(0.0, 25.0);
        let pts = if (port.x - air.x).abs() < 20.0 {
            vec![port, pos2(port.x, air.y)]
        } else {
            let my = ((port.y + air.y) * 0.5 / GRID).round() * GRID;
            vec![port, pos2(port.x, my), pos2(air.x, my), air]
        };
        push(pts, on02(16));
    }

    // SV03 ports 1..6 -> slot drains. Ports leave on lanes beside the valve,
    // rise to one row each above it (port 1 highest), then run to the slot's
    // gap column.
    for slot in 1u16..=6 {
        let p = sv03_port_for_slot(slot);
        let port = port_pos(sv03, r03, n03, p);
        let row = sv03.y - r03 - 30.0 - (6 - p) as f32 * LANE;
        let rad = port - sv03;
        let mut pts = vec![port];
        if rad.x < -r03 * 0.3 {
            let x = sv03.x - r03 - 20.0 - (3 - p.min(3)) as f32 * LANE;
            pts.extend([pos2(x, port.y), pos2(x, row)]);
        } else if rad.x > r03 * 0.3 {
            let x = sv03.x + r03 + 20.0 + p.saturating_sub(5) as f32 * LANE;
            pts.extend([pos2(x, port.y), pos2(x, row)]);
        } else {
            pts.push(pos2(port.x, row));
        }
        let (outlet, col) = slot_outlet(l, slot);
        pts.extend([pos2(col, row), pos2(col, outlet.y), outlet]);
        push(pts, on03(p));
    }
    // SV03 7 -> Waste, 8 -> Danger Waste.
    for (p, key) in [(7u16, "TAG_SV03_WASTE"), (8, "TAG_SV03_DANGER")] {
        let port = port_pos(sv03, r03, n03, p);
        push(into_tag(vec![port, stub(sv03, port, 16.0)], tag_rect(l, key)), on03(p));
    }

    edges
}

/// Channel map from the drawio labels: (TC02 channel, TC01 channel).
const BUNDLE: [(usize, usize); 9] = [(1, 1), (2, 2), (3, 3), (4, 7), (5, 8), (6, 9), (7, 4), (8, 5), (9, 6)];

/// A reagent tube, ordered away from the pump: bundle tubes run TC01 -> TC02,
/// dip tubes run TC02 -> bottle.
struct ReagentTube {
    pts: Vec<Pos2>,
    /// TC02 channel the tube belongs to.
    tc02: usize,
}

/// Bottle on a TC02 channel: 1..3 and 4..6 are the two dip tubes of C1..C3, 7..9 feed C4..C6.
pub(crate) fn bottle_for_tc02(ch: usize) -> u16 {
    if ch <= 6 { ((ch - 1) % 3 + 1) as u16 } else { (ch - 3) as u16 }
}

fn tc01_for_tc02(ch: usize) -> usize {
    BUNDLE.iter().find(|(from, _)| *from == ch).map_or(0, |(_, to)| *to)
}

/// The reagent channel currently connected to PP01, if any: SV01 ports 1..3
/// when the solenoid faces SV01, SV02 ports 1..6 when it faces the coil.
/// Returns the TC02 channel and the flow direction away from the pump.
pub(crate) fn live_reagent(state: &BusState) -> Option<(usize, i8)> {
    let pump = state.dev(DeviceId::Pp01);
    let tc01 = if pump.solenoid_input.unwrap_or(false) {
        6 + state.dev(DeviceId::Sv01).value.filter(|p| (1..=3).contains(p))? as usize
    } else {
        state.dev(DeviceId::Sv02).value.filter(|p| (1..=6).contains(p))? as usize
    };
    BUNDLE.iter().find(|(_, to)| *to == tc01).map(|(from, _)| (*from, -pump.motion))
}

/// Reagent plumbing: the TC01 -> TC02 bundle and TC02 -> bottle dip tubes.
fn route_reagents(l: &Layout) -> Vec<ReagentTube> {
    let mut tubes = Vec::new();
    for (from, to) in BUNDLE {
        let a = tc_channel(l, "TC01", to, false);
        let b = tc_channel(l, "TC02", from, false);
        let x = a.x.min(b.x) - 12.0 - (from - 1) as f32 * 7.0;
        tubes.push(ReagentTube { pts: clean(vec![a, pos2(x, a.y), pos2(x, b.y), b]), tc02: from });
    }
    let tc02 = l.pos("TC02");
    for c in 1u16..=6 {
        let b = l.pos(&format!("C{c}"));
        let top = b.y - 45.0;
        if c <= 3 {
            // Two dip tubes per 1 L bottle: one to SV02, one to SV01.
            for (k, dx) in [(c as usize, 4.0), (c as usize + 3, -4.0)] {
                let ch = tc_channel(l, "TC02", k, true);
                tubes.push(ReagentTube { pts: clean(vec![ch, pos2(b.x + dx, ch.y), pos2(b.x + dx, top)]), tc02: k });
            }
        } else {
            // Small bottles sit under the large ones: drop on a lane, run under the large row.
            let idx = (c - 4) as f32;
            let k = c as usize + 3;
            let ch = tc_channel(l, "TC02", k, true);
            let x = tc02.x + 74.0 - idx * 12.0;
            let row = top - 64.0 + idx * 12.0;
            tubes.push(ReagentTube { pts: clean(vec![ch, pos2(x, ch.y), pos2(x, row), pos2(b.x, row), pos2(b.x, top)]), tc02: k });
        }
    }
    tubes
}

#[allow(clippy::too_many_arguments)]
pub fn show(
    ui: &mut egui::Ui,
    state: &BusState,
    settings: &Settings,
    fluid: &FluidView,
    selected: Option<DeviceId>,
    time: f64,
    layout: &mut Layout,
    edit: bool,
) -> SchematicResponse {
    let pal = Palette::for_ui(ui);
    let avail = ui.available_size();
    let (response, painter) = ui.allocate_painter(avail, Sense::click());
    let rect = response.rect;
    painter.rect_filled(rect, 6.0, pal.bg);

    let scale = (rect.width() / W).min(rect.height() / H);
    let origin = rect.center() - vec2(W, H) * scale * 0.5;
    let t = |p: Pos2| origin + p.to_vec2() * scale;
    let s = |v: f32| v * scale;
    // Stroke widths never drop below one physical pixel, whatever the window size or screen DPI.
    let hairline = 1.0 / ui.ctx().pixels_per_point();
    let sw = |v: f32| (v * scale).max(hairline);
    let font = |size: f32| FontId::proportional((size * scale).max(7.0));
    let big = settings.label_scale.clamp(0.8, 1.6);
    let lfont = |size: f32| FontId::proportional((size * big * scale).max(8.0));
    let mut out = SchematicResponse::default();

    // ── Move mode: drag components before anything is drawn ──
    let mut highlight: Option<&str> = None;
    if edit {
        for gx in 0..=(W / 50.0) as i32 {
            for gy in 0..=(H / 50.0) as i32 {
                painter.circle_filled(t(pos2(gx as f32 * 50.0, gy as f32 * 50.0)), s(1.5).max(1.0), pal.grid);
            }
        }
        for (key, _) in DEFAULTS {
            let c = layout.pos(key);
            let r = hit_rect(key, c);
            let screen = Rect::from_min_max(t(r.min), t(r.max));
            let resp = ui
                .interact(screen, ui.id().with(("layout_node", *key)), Sense::click_and_drag())
                .on_hover_and_drag_cursor(if ui.input(|i| i.pointer.primary_down()) { CursorIcon::Grabbing } else { CursorIcon::Grab });
            if resp.dragged() {
                layout.set(key, c + resp.drag_delta() / scale.max(1e-3));
                out.layout_changed = true;
                highlight = Some(key);
            }
            if resp.drag_stopped() {
                let p = layout.pos(key);
                layout.set(key, pos2((p.x / GRID).round() * GRID, (p.y / GRID).round() * GRID));
                out.layout_changed = true;
            }
            if resp.hovered() && highlight.is_none() {
                highlight = Some(key);
            }
            if (resp.clicked() || resp.drag_started())
                && let Some(id) = device_for_key(key)
            {
                out.clicked_device = Some(id);
            }
            if resp.secondary_clicked() {
                out.menu_target = Some(target_for_key(key));
            }
            out.menu_responses.push(resp);
        }
    }

    let text = |p: Pos2, anchor: Align2, txt: &str, size: f32, color: Color32| {
        painter.text(t(p), anchor, txt, font(size), color);
    };
    let tube = |pts: &[Pos2], stroke: Stroke| {
        let screen: Vec<Pos2> = pts.iter().map(|&p| t(p)).collect();
        let smooth = rounded(&screen, s(8.0));
        painter.line(smooth.clone(), stroke);
        smooth
    };

    let dev = |id: DeviceId| state.dev(id);
    let state_color = |st: DevState| match st {
        DevState::Offline => pal.offline,
        DevState::Fault => pal.fault,
        DevState::Moving => pal.busy,
        DevState::Ready => pal.ok,
    };
    let status_color = |id: DeviceId| state_color(dev(id).state());
    let slot_color = |st: SlotState| match st {
        SlotState::Missing => pal.offline,
        SlotState::Busy => pal.busy,
        SlotState::Filling | SlotState::Draining => pal.flow,
        SlotState::FillPath | SlotState::DrainPath => pal.flow_idle,
        SlotState::Filled => pal.ok,
        SlotState::Empty => pal.dim,
    };
    let bottle_color = |st: BottleState| match st {
        BottleState::Drawing | BottleState::Returning => pal.flow,
        BottleState::Linked => pal.flow_idle,
        BottleState::Empty => pal.fault,
        BottleState::Low => pal.busy,
        BottleState::Ok => pal.ok,
    };
    let text_on = |c: Color32| {
        if 0.299 * c.r() as f32 + 0.587 * c.g() as f32 + 0.114 * c.b() as f32 > 150.0 { Color32::BLACK } else { Color32::WHITE }
    };
    // Filled state badge centered on a virtual point.
    let badge = |at: Pos2, label: &str, color: Color32, font_id: FontId| {
        let galley = painter.layout_no_wrap(label.to_string(), font_id, text_on(color));
        let r = Rect::from_center_size(t(at), galley.size() + vec2(s(10.0), s(4.0)));
        painter.rect_filled(r, s(4.0), color);
        painter.galley(r.center() - galley.size() * 0.5, galley, text_on(color));
    };

    let l = &*layout;
    let pp01_input = dev(DeviceId::Pp01).solenoid_input.unwrap_or(false);

    // Tubes: idle reagent plumbing and idle pump/valve tubes, then active ones on top.
    // The reagent channel connected to PP01 stays lit past TC01, through the
    // bundle and TC02, down to its bottle.
    let live = live_reagent(state);
    let reagents = route_reagents(l);
    let is_live = |r: &ReagentTube| live.is_some_and(|(ch, _)| ch == r.tc02);
    for r in reagents.iter().filter(|r| !is_live(r)) {
        tube(&r.pts, Stroke::new(sw(2.0), pal.bundle));
    }
    let active_stroke = |dir: i8| Stroke::new(sw(3.5), if dir != 0 { pal.flow } else { pal.flow_idle });
    let edges = route_tubes(l, state);
    for pass in [false, true] {
        for e in edges.iter().filter(|e| e.active == pass) {
            let stroke = if e.active { active_stroke(e.dir) } else { Stroke::new(sw(1.6), pal.line) };
            let smooth = tube(&e.pts, stroke);
            if e.active && e.dir != 0 {
                draw_flow_dots(&painter, &smooth, e.dir, time, s(26.0), s(3.2), pal.bg);
            }
        }
    }
    if let Some((_, dir)) = live {
        for r in reagents.iter().filter(|r| is_live(r)) {
            let smooth = tube(&r.pts, active_stroke(dir));
            if dir != 0 {
                draw_flow_dots(&painter, &smooth, dir, time, s(26.0), s(3.2), pal.bg);
            }
        }
    }
    let live_tc = live.map(|(ch, _)| (tc01_for_tc02(ch), ch));
    let live_bottle = live.map(|(ch, _)| bottle_for_tc02(ch));

    // TC blocks with channel ports on both sides.
    for key in ["TC01", "TC02"] {
        let c = l.pos(key);
        let rr = Rect::from_center_size(t(c), vec2(s(20.0), s(190.0)));
        painter.rect(rr, s(2.0), pal.panel, Stroke::new(sw(1.5), pal.line), StrokeKind::Inside);
        for k in 1..=9 {
            let lit = live_tc.is_some_and(|(c1, c2)| k == if key == "TC01" { c1 } else { c2 });
            for right in [false, true] {
                painter.circle_filled(t(tc_channel(l, key, k, right)), s(if lit { 4.0 } else { 2.5 }), if lit { pal.flow } else { pal.line });
            }
        }
        text(c - vec2(0.0, 110.0), Align2::CENTER_CENTER, key, 13.0, pal.text);
    }

    // Bottles.
    for c in 1u16..=6 {
        let b = l.pos(&format!("C{c}"));
        let (st, level_ml) = fluid.bottles[c as usize - 1];
        let frac = (level_ml / BOTTLE_CAPACITY_ML[c as usize - 1]).clamp(0.0, 1.0);
        let r = Rect::from_center_size(t(b), vec2(s(70.0), s(90.0)));
        let fill = Rect::from_min_max(pos2(r.min.x, r.max.y - r.height() * frac), r.max);
        painter.rect_filled(r, s(8.0), pal.panel);
        painter.rect_filled(fill, s(8.0), pal.liquid);
        let lit = live_bottle == Some(c);
        let outline = if lit { Stroke::new(sw(3.0), pal.flow) } else { Stroke::new(sw(1.5), pal.line) };
        painter.rect_stroke(r, s(8.0), outline, StrokeKind::Inside);
        text(b - vec2(0.0, 31.0), Align2::CENTER_CENTER, &format!("C{c}"), 13.0, pal.text);
        badge(b - vec2(0.0, 11.0), st.label(), bottle_color(st), lfont(9.5));
        painter.text(t(b + vec2(0.0, 13.0)), Align2::CENTER_CENTER, format!("{level_ml:.0} mL"), lfont(11.5), pal.text);
        text(b + vec2(0.0, 31.0), Align2::CENTER_CENTER, if c <= 3 { "of 1 L" } else { "of 0.5 L" }, 10.5, pal.text);
    }
    text(l.pos("C2") + vec2(0.0, 59.0), Align2::CENTER_CENTER, "Large external reagents", 13.0, pal.dim);
    text(l.pos("C5") + vec2(0.0, 59.0), Align2::CENTER_CENTER, "Small external reagents", 13.0, pal.dim);

    // Tags.
    for (key, _) in DEFAULTS.iter().filter(|(k, _)| k.starts_with("TAG")) {
        let r = tag_rect(l, key);
        let screen = Rect::from_min_max(t(r.min), t(r.max));
        painter.rect(screen, s(4.0), pal.tag, Stroke::new(sw(1.0), pal.busy), StrokeKind::Inside);
        painter.text(screen.center(), Align2::CENTER_CENTER, tag_text(key), font(12.5), pal.tag_text);
    }

    // Air filter.
    let air_c = l.pos("AIR");
    painter.rect(Rect::from_center_size(t(air_c), vec2(s(60.0), s(50.0))), s(6.0), pal.panel, Stroke::new(sw(1.5), pal.line), StrokeKind::Inside);
    text(air_c, Align2::CENTER_CENTER, "Air", 13.0, pal.text);
    text(air_c + vec2(40.0, -10.0), Align2::LEFT_CENTER, "filter / port", 11.0, pal.dim);

    // Holding coil.
    let coil = l.pos("COIL");
    for k in 0..7 {
        painter.circle_stroke(t(coil), s(10.0 + k as f32 * 8.0), Stroke::new(sw(1.3), if !pp01_input { pal.flow_idle } else { pal.line }));
    }
    text(coil + vec2(0.0, 80.0), Align2::CENTER_CENTER, "Spiral tube (holding coil)", 12.5, pal.dim);

    // Sensors (optical, on CAN via controller_v2, not polled here).
    let sensor = |p: Pos2, name: &str, label_at: Vec2, anchor: Align2| {
        let r = Rect::from_center_size(t(p), vec2(s(22.0), s(28.0)));
        painter.rect(r, s(4.0), pal.panel, Stroke::new(sw(1.2), pal.line), StrokeKind::Inside);
        painter.circle_filled(t(p), s(5.0), pal.offline);
        text(p + label_at, anchor, name, 11.5, pal.dim);
    };
    sensor(l.pos("S1"), "S1", vec2(0.0, 26.0), Align2::CENTER_CENTER);
    sensor(l.pos("S2"), "S2", vec2(0.0, 26.0), Align2::CENTER_CENTER);

    let a02 = dev(DeviceId::Sv02).value.filter(|p| *p > 0);
    let a03 = dev(DeviceId::Sv03).value.filter(|p| *p > 0);

    // Slots.
    for slot in 1u16..=6 {
        let c = l.pos(&slot_key(slot));
        let rr = Rect::from_center_size(t(c), vec2(s(90.0), s(150.0)));
        let feeding = !pp01_input && a02 == Some(sv02_port_for_slot(slot));
        let draining = a03 == Some(sv03_port_for_slot(slot));
        let stroke = if feeding || draining { Stroke::new(sw(3.0), pal.flow) } else { Stroke::new(sw(1.5), pal.line) };
        painter.rect(rr, s(10.0), pal.panel, stroke, StrokeKind::Inside);
        // Vial filled to the estimated volume.
        let (st, vol_ul) = fluid.slots[slot as usize - 1];
        let vial = Rect::from_center_size(t(c + vec2(0.0, -4.0)), vec2(s(34.0), s(92.0)));
        painter.rect_filled(vial, s(3.0), pal.bg);
        let frac = (vol_ul / fluid.slot_capacity_ul.max(1.0)).clamp(0.0, 1.0);
        painter.rect_filled(Rect::from_min_max(pos2(vial.min.x, vial.max.y - vial.height() * frac), vial.max), s(3.0), pal.liquid);
        painter.rect_stroke(vial, s(3.0), Stroke::new(sw(1.0), pal.line), StrokeKind::Inside);
        text(c + vec2(-37.0, -63.0), Align2::LEFT_CENTER, &format!("Slot {slot}"), 12.5, pal.text);
        painter.text(t(c + vec2(0.0, 58.0)), Align2::CENTER_CENTER, format!("~{vol_ul:.0} µL"), lfont(11.0), pal.text);
        badge(c - vec2(0.0, 88.0), st.label(), slot_color(st), lfont(11.0));
        sensor(pos2(c.x - 20.0, c.y + 91.0), &format!("S{}", slot + 2), vec2(16.0, 0.0), Align2::LEFT_CENTER);
        painter.circle_filled(t(slot_outlet(l, slot).0), s(4.0), pal.line);
    }

    let pointer = response.interact_pointer_pos();
    if response.secondary_clicked()
        && let Some(q) = pointer
    {
        let v = pos2((q.x - origin.x) / scale.max(1e-3), (q.y - origin.y) / scale.max(1e-3));
        out.menu_target = Some(target_at(l, v, big));
    }
    let clicked = response.clicked() && !edit;
    let hover = if edit { None } else { response.hover_pos() };

    // Valves: name and current port are written inside the body, clear of the tubes.
    for id in [DeviceId::Sv01, DeviceId::Sv02, DeviceId::Sv03] {
        let center = l.pos(id.tag());
        let (r, ports) = valve_geom(id);
        let d = dev(id);
        let c = t(center);
        let sel = selected == Some(id);
        painter.circle(c, s(r + 4.0), pal.panel, Stroke::new(sw(if sel { 4.0 } else { 2.0 }), if sel { pal.flow } else { status_color(id) }));
        let port_r = if ports == 16 { 12.0 } else { 14.0 };
        let current = d.value.filter(|p| *p > 0);
        if let Some(p) = current {
            painter.line_segment([c, t(port_pos(center, r, ports, p))], Stroke::new(sw(9.0), pal.flow_idle.gamma_multiply(0.6)));
        }
        if let Some(target) = d.target.filter(|&tp| Some(tp) != current && tp > 0) {
            painter.line_segment([c, t(port_pos(center, r, ports, target))], Stroke::new(sw(2.0), pal.busy));
        }
        painter.circle_filled(c, s(10.0), if current.is_some() { pal.flow } else { pal.offline });
        for p in 1..=ports {
            let pp = t(port_pos(center, r, ports, p));
            let is_cur = current == Some(p);
            let hovered = hover.is_some_and(|h| h.distance(pp) < s(port_r));
            let fill = if is_cur { pal.flow } else if hovered { pal.bundle } else { pal.panel };
            painter.circle(pp, s(port_r), fill, Stroke::new(sw(1.3), pal.line));
            painter.text(pp, Align2::CENTER_CENTER, p.to_string(), font(if ports == 16 { 11.0 } else { 12.5 }), if is_cur { Color32::WHITE } else { pal.text });
            if hovered {
                response.clone().on_hover_text(format!("{} port {p}: {}", id.tag(), id.port_label(p)));
            }
            if clicked && pointer.is_some_and(|q| q.distance(pp) < s(port_r)) {
                out.clicked_port = Some((id, p));
            }
        }
        if clicked && out.clicked_port.is_none() && pointer.is_some_and(|q| q.distance(c) < s(r)) {
            out.clicked_device = Some(id);
        }
        let st = d.state();
        let k = if ports == 16 { 1.0 } else { 0.8 };
        painter.text(t(center - vec2(0.0, r * 0.5)), Align2::CENTER_CENTER, id.tag(), lfont(16.0 * k), pal.text);
        let port_txt = match current {
            Some(p) => format!("{p} · {}", id.port_label(p).split(" (").next().unwrap_or("")),
            None if d.online => "reset".to_string(),
            None => "—".to_string(),
        };
        painter.text(t(center + vec2(0.0, r * 0.27)), Align2::CENTER_CENTER, port_txt, lfont(14.0 * k), pal.text);
        badge(center + vec2(0.0, r * 0.56), st.label(), state_color(st), lfont(12.0 * k));
    }

    // Pumps.
    for id in [DeviceId::Pp01, DeviceId::Pp02] {
        let center = l.pos(id.tag());
        let d = dev(id);
        let sel = selected == Some(id);
        let body = Rect::from_center_size(t(center), vec2(s(90.0), s(160.0)));
        painter.rect(body, s(6.0), pal.panel, Stroke::new(sw(if sel { 4.0 } else { 2.0 }), if sel { pal.flow } else { status_color(id) }), StrokeKind::Inside);
        let max = settings.max_steps(id).max(1) as f32;
        let frac = d.value.map(|v| (v as f32 / max).clamp(0.0, 1.0)).unwrap_or(0.0);
        let barrel = Rect::from_min_max(t(center + vec2(-22.0, -50.0)), t(center + vec2(22.0, 55.0)));
        painter.rect(barrel, s(3.0), pal.bg, Stroke::new(sw(1.3), pal.line), StrokeKind::Inside);
        let liquid = Rect::from_min_max(barrel.min, pos2(barrel.max.x, barrel.min.y + barrel.height() * frac));
        painter.rect_filled(liquid, s(3.0), pal.liquid);
        let plunger_y = barrel.min.y + barrel.height() * frac;
        painter.line_segment([pos2(barrel.min.x, plunger_y), pos2(barrel.max.x, plunger_y)], Stroke::new(sw(4.0), pal.line));
        painter.line_segment([pos2(barrel.center().x, plunger_y), pos2(barrel.center().x, body.max.y - s(6.0))], Stroke::new(sw(3.0), pal.line));
        if let Some(target) = d.target {
            let ty = barrel.min.y + barrel.height() * (target as f32 / max).clamp(0.0, 1.0);
            painter.line_segment([pos2(barrel.min.x - s(8.0), ty), pos2(barrel.max.x + s(8.0), ty)], Stroke::new(sw(2.0), pal.busy));
        }
        // Readout card: state header, steps + fill %, volume, fill bar, motion + flow rate, speed + target.
        let st = d.state();
        let col = state_color(st);
        let ul = settings.ul_per_step(id);
        let card = pump_card_rect(l, id, big);
        let sr = Rect::from_min_max(t(card.min), t(card.max));
        painter.rect(sr, s(6.0), pal.panel, Stroke::new(sw(1.8), col), StrokeKind::Inside);
        let cr = s(6.0).round().clamp(0.0, 255.0) as u8;
        let head = Rect::from_min_size(sr.min, vec2(sr.width(), s(26.0 * big)));
        painter.rect_filled(head, CornerRadius { nw: cr, ne: cr, sw: 0, se: 0 }, col);
        let left = |dy: f32| t(pos2(card.min.x + 9.0 * big, card.min.y + dy * big));
        let right = |dy: f32| t(pos2(card.max.x - 9.0 * big, card.min.y + dy * big));
        painter.text(left(13.0), Align2::LEFT_CENTER, id.tag(), lfont(15.0), text_on(col));
        painter.text(right(13.0), Align2::RIGHT_CENTER, st.label(), lfont(13.0), text_on(col));
        let (steps_txt, vol_txt, pct_txt) = match d.value.filter(|_| d.online) {
            Some(v) => (format!("{v} st"), format!("{:.0} µL", v as f32 * ul), format!("{:.0}%", frac * 100.0)),
            None => ("—".to_string(), "— µL".to_string(), String::new()),
        };
        painter.text(left(44.0), Align2::LEFT_CENTER, steps_txt, lfont(21.0), pal.text);
        painter.text(right(44.0), Align2::RIGHT_CENTER, pct_txt, lfont(16.0), pal.flow);
        painter.text(left(67.0), Align2::LEFT_CENTER, format!("{vol_txt} of {:.0}", max * ul), lfont(13.5), pal.text);
        let bar = Rect::from_min_max(left(79.0), right(86.0));
        painter.rect_filled(bar, s(3.0), pal.bg);
        painter.rect_filled(Rect::from_min_size(bar.min, vec2(bar.width() * frac, bar.height())), s(3.0), pal.liquid);
        let rate = (d.steps_per_sec() * ul).abs();
        let (motion_txt, motion_col) = match (d.online, d.motion) {
            (false, _) => ("no reply".to_string(), pal.offline),
            (true, m) if m > 0 => (format!("aspirating {rate:.0} µL/s"), pal.busy),
            (true, m) if m < 0 => (format!("dispensing {rate:.0} µL/s"), pal.busy),
            _ => ("idle".to_string(), pal.dim),
        };
        painter.text(left(99.0), Align2::LEFT_CENTER, motion_txt.split(' ').next().unwrap_or(""), lfont(13.0), motion_col);
        // Right of the motion word: flow rate while moving, otherwise the last speed set.
        let (aux_txt, aux_col) = if d.online && d.motion != 0 {
            (format!("{rate:.0} µL/s"), pal.busy)
        } else {
            (d.speed.map(|v| format!("{v} rpm")).unwrap_or_default(), pal.dim)
        };
        painter.text(right(99.0), Align2::RIGHT_CENTER, aux_txt, lfont(12.5), aux_col);
        if id == DeviceId::Pp01 {
            let sol = Rect::from_center_size(t(center + vec2(0.0, -70.0)), vec2(s(60.0), s(20.0)));
            let (label, col) = match d.solenoid_input {
                Some(true) => ("to SV01", pal.flow),
                Some(false) => ("to coil", pal.flow),
                None => ("S ?", pal.dim),
            };
            painter.rect(sol, s(3.0), pal.panel, Stroke::new(sw(1.2), col), StrokeKind::Inside);
            text(center + vec2(0.0, -70.0), Align2::CENTER_CENTER, label, 11.0, col);
        }
        if clicked && pointer.is_some_and(|q| body.expand(s(20.0)).contains(q) || sr.contains(q)) {
            out.clicked_device = Some(id);
        }
    }

    // Move-mode outline of the hovered / dragged component.
    if let Some(key) = highlight {
        let r = hit_rect(key, layout.pos(key));
        let screen = Rect::from_min_max(t(r.min), t(r.max));
        painter.rect_stroke(screen, s(6.0), Stroke::new(2.0, pal.busy), StrokeKind::Outside);
        let p = layout.pos(key);
        painter.text(screen.left_top() - vec2(0.0, 4.0), Align2::LEFT_BOTTOM, format!("{key}  ({:.0}, {:.0})", p.x, p.y), FontId::proportional(12.0), pal.busy);
    }

    // Legend.
    let lg = pos2(24.0, 958.0);
    let items = [(pal.ok, "ready"), (pal.busy, "moving"), (pal.fault, "fault"), (pal.offline, "offline"), (pal.flow, "active path")];
    for (i, (c, name)) in items.iter().enumerate() {
        let p = lg + vec2(i as f32 * 110.0, 0.0);
        painter.circle_filled(t(p), s(6.0), *c);
        text(p + vec2(12.0, 0.0), Align2::LEFT_CENTER, name, 12.0, pal.dim);
    }
    let hint = if edit {
        "Move mode: drag any component; tubes re-route and it snaps to a 10 px grid on release."
    } else {
        "S1–S8 are CAN optical sensors read by controller_v2; they are not polled over RS485."
    };
    text(pos2(24.0, 982.0), Align2::LEFT_CENTER, hint, 11.0, if edit { pal.busy } else { pal.dim });

    out.menu_responses.insert(0, response);
    out
}

/// Moving dots along a polyline to show flow direction.
fn draw_flow_dots(painter: &egui::Painter, pts: &[Pos2], dir: i8, time: f64, spacing: f32, radius: f32, color: Color32) {
    let lens: Vec<f32> = pts.windows(2).map(|w| w[0].distance(w[1])).collect();
    let total: f32 = lens.iter().sum();
    if total < 1.0 || spacing < 1.0 {
        return;
    }
    let phase = ((time as f32 * 60.0) % spacing) * dir as f32;
    let mut d = phase.rem_euclid(spacing);
    while d < total {
        let mut acc = 0.0;
        for (i, &l) in lens.iter().enumerate() {
            if d <= acc + l {
                let k = if l > 0.0 { (d - acc) / l } else { 0.0 };
                painter.circle_filled(pts[i] + (pts[i + 1] - pts[i]) * k, radius, color);
                break;
            }
            acc += l;
        }
        d += spacing;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis_aligned(pts: &[Pos2]) -> bool {
        pts.windows(2).all(|w| (w[0].x - w[1].x).abs() < 0.01 || (w[0].y - w[1].y).abs() < 0.01)
    }

    #[test]
    fn port_numbering_matches_drawio() {
        let c = pos2(0.0, 0.0);
        let near = |p: Pos2, x: f32, y: f32| (p.x - x).abs() < 0.01 && (p.y - y).abs() < 0.01;
        assert!(near(port_pos(c, 1.0, 16, 16), 0.0, -1.0), "SV02 16 top");
        assert!(near(port_pos(c, 1.0, 16, 4), -1.0, 0.0), "SV02 4 left");
        assert!(near(port_pos(c, 1.0, 16, 8), 0.0, 1.0), "SV02 8 bottom");
        assert!(near(port_pos(c, 1.0, 16, 12), 1.0, 0.0), "SV02 12 right");
        assert!(near(port_pos(c, 1.0, 8, 4), 0.0, -1.0), "8-port 4 top");
        assert!(near(port_pos(c, 1.0, 8, 6), 1.0, 0.0), "8-port 6 right");
        assert!(near(port_pos(c, 1.0, 8, 2), -1.0, 0.0), "8-port 2 left");
    }

    #[test]
    fn clean_merges_collinear_points() {
        let pts = clean(vec![pos2(0.0, 0.0), pos2(5.0, 0.0), pos2(10.0, 0.0), pos2(10.0, 0.0), pos2(10.0, 7.0)]);
        assert_eq!(pts, vec![pos2(0.0, 0.0), pos2(10.0, 0.0), pos2(10.0, 7.0)]);
    }

    /// Tube points outside the valve bodies: the final hop from the rim to a
    /// valve's common port is hidden under the body and may be diagonal.
    fn visible(pts: &[Pos2], layout: &Layout) -> Vec<Pos2> {
        let into_center = pts.last().is_some_and(|p| ["SV01", "SV02", "SV03"].iter().any(|k| p.distance(layout.pos(k)) < 0.5));
        pts[..if into_center { pts.len() - 1 } else { pts.len() }].to_vec()
    }

    #[test]
    fn default_tubes_are_orthogonal() {
        let layout = Layout::default();
        let state = crate::bus::BusState::for_tests();
        for e in route_tubes(&layout, &state) {
            let pts = visible(&e.pts, &layout);
            assert!(axis_aligned(&pts), "{pts:?}");
        }
        for r in route_reagents(&layout) {
            assert!(axis_aligned(&r.pts), "{:?}", r.pts);
        }
    }

    #[test]
    fn default_tubes_never_share_a_segment() {
        let layout = Layout::default();
        let state = crate::bus::BusState::for_tests();
        let mut segs: Vec<(Pos2, Pos2)> = Vec::new();
        for e in route_tubes(&layout, &state) {
            segs.extend(visible(&e.pts, &layout).windows(2).map(|w| (w[0], w[1])));
        }
        for r in route_reagents(&layout) {
            segs.extend(r.pts.windows(2).map(|w| (w[0], w[1])));
        }
        for (i, a) in segs.iter().enumerate() {
            for b in &segs[i + 1..] {
                assert!(!overlaps(*a, *b), "tubes overlap: {a:?} and {b:?}");
            }
        }
    }

    #[test]
    fn pump_cards_stay_clear_of_tubes() {
        let layout = Layout::default();
        let state = crate::bus::BusState::for_tests();
        let mut segs: Vec<(Pos2, Pos2)> = Vec::new();
        for e in route_tubes(&layout, &state) {
            segs.extend(visible(&e.pts, &layout).windows(2).map(|w| (w[0], w[1])));
        }
        for r in route_reagents(&layout) {
            segs.extend(r.pts.windows(2).map(|w| (w[0], w[1])));
        }
        // Component outlines, as drawn.
        let mut parts: Vec<(String, Rect)> = Vec::new();
        for (key, _) in DEFAULTS {
            let c = layout.pos(key);
            let r = match *key {
                "SV01" | "SV03" => Rect::from_center_size(c, vec2(180.0, 180.0)),
                "SV02" => Rect::from_center_size(c, vec2(240.0, 240.0)),
                "PP01" | "PP02" => Rect::from_center_size(c, vec2(90.0, 160.0)),
                "COIL" => Rect::from_center_size(c, vec2(130.0, 130.0)),
                k if k.starts_with("TAG") => tag_rect(&layout, k),
                k => hit_rect(k, c),
            };
            parts.push((key.to_string(), r));
        }
        for big in [0.8, Settings::default().label_scale, 1.6] {
            let cards = [DeviceId::Pp01, DeviceId::Pp02].map(|id| (id, pump_card_rect(&layout, id, big).expand(4.0)));
            assert!(!cards[0].1.intersects(cards[1].1), "pump cards overlap at scale {big}");
            for (id, card) in cards {
                for (a, b) in &segs {
                    assert!(!card.intersects(Rect::from_two_pos(*a, *b)), "{} card at scale {big} hits tube {a:?}-{b:?}", id.tag());
                }
                for (name, r) in &parts {
                    assert!(!card.intersects(*r), "{} card at scale {big} covers {name}", id.tag());
                }
            }
        }
    }

    /// Renders the default layout's tube routes to SVG for visual review
    /// without a display: `ROUTES_SVG=out.svg cargo test export_routes_svg -- --ignored`.
    #[test]
    #[ignore]
    fn export_routes_svg() {
        use std::fmt::Write as _;
        let path = std::env::var("ROUTES_SVG").unwrap_or_else(|_| "routes.svg".into());
        let l = Layout::default();
        let state = crate::bus::BusState::for_tests();
        let mut svg = String::new();
        let _ = write!(svg, r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="Helvetica" font-size="12"><rect width="100%" height="100%" fill="#fafaf8"/>"##);
        let poly = |svg: &mut String, pts: &[Pos2], color: &str, width: f32| {
            let d: Vec<String> = rounded(pts, 8.0).iter().map(|p| format!("{:.1},{:.1}", p.x, p.y)).collect();
            let _ = write!(svg, r#"<polyline points="{}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"/>"#, d.join(" "));
        };
        for r in route_reagents(&l) {
            poly(&mut svg, &r.pts, "#b9bcc2", 2.0);
        }
        for e in route_tubes(&l, &state) {
            poly(&mut svg, &e.pts, "#46484e", 1.6);
        }
        let rect = |svg: &mut String, r: Rect, fill: &str, label: &str| {
            let _ = write!(svg, r##"<rect x="{:.1}" y="{:.1}" width="{:.1}" height="{:.1}" rx="5" fill="{fill}" stroke="#46484e"/><text x="{:.1}" y="{:.1}" text-anchor="middle" dominant-baseline="middle">{label}</text>"##, r.min.x, r.min.y, r.width(), r.height(), r.center().x, r.center().y);
        };
        for key in ["TC01", "TC02"] {
            rect(&mut svg, Rect::from_center_size(l.pos(key), vec2(20.0, 190.0)), "#fff", "");
        }
        for c in 1..=6 {
            rect(&mut svg, Rect::from_center_size(l.pos(&format!("C{c}")), vec2(70.0, 90.0)), "#aacdf5", &format!("C{c}"));
        }
        for slot in 1..=6u16 {
            rect(&mut svg, Rect::from_center_size(l.pos(&slot_key(slot)), vec2(90.0, 150.0)), "#fff", &format!("Slot {slot}"));
            rect(&mut svg, Rect::from_center_size(pos2(l.pos(&slot_key(slot)).x - 20.0, l.pos(&slot_key(slot)).y + 91.0), vec2(22.0, 28.0)), "#fff", "");
        }
        for (key, _) in DEFAULTS.iter().filter(|(k, _)| k.starts_with("TAG")) {
            rect(&mut svg, tag_rect(&l, key), "#ffecaa", tag_text(key));
        }
        rect(&mut svg, Rect::from_center_size(l.pos("AIR"), vec2(60.0, 50.0)), "#fff", "Air");
        for key in ["S1", "S2"] {
            rect(&mut svg, Rect::from_center_size(l.pos(key), vec2(22.0, 28.0)), "#fff", key);
        }
        for key in ["PP01", "PP02"] {
            rect(&mut svg, Rect::from_center_size(l.pos(key), vec2(90.0, 160.0)), "#fff", key);
        }
        let coil = l.pos("COIL");
        let _ = write!(svg, r##"<circle cx="{}" cy="{}" r="58" fill="#fff" stroke="#5a96d2"/><text x="{}" y="{}" text-anchor="middle">coil</text>"##, coil.x, coil.y, coil.x, coil.y);
        for id in [DeviceId::Sv01, DeviceId::Sv02, DeviceId::Sv03] {
            let c = l.pos(id.tag());
            let (r, n) = valve_geom(id);
            let _ = write!(svg, r##"<circle cx="{}" cy="{}" r="{}" fill="#fff" stroke="#46484e" stroke-width="2"/><text x="{}" y="{}" text-anchor="middle">{}</text>"##, c.x, c.y, r + 4.0, c.x, c.y, id.tag());
            for p in 1..=n {
                let q = port_pos(c, r, n, p);
                let _ = write!(svg, r##"<circle cx="{:.1}" cy="{:.1}" r="11" fill="#fff" stroke="#46484e"/><text x="{:.1}" y="{:.1}" text-anchor="middle" dominant-baseline="middle" font-size="10">{p}</text>"##, q.x, q.y, q.x, q.y);
            }
        }
        svg.push_str("</svg>");
        std::fs::write(&path, svg).unwrap();
    }

    #[test]
    fn live_reagent_follows_valve_to_bottle() {
        let mut state = crate::bus::BusState::for_tests();
        let set = |state: &mut BusState, id: DeviceId, port: u16| state.devices[id as usize].value = Some(port);
        // Solenoid towards the coil, SV02 on port 4: C4 via TC01 channel 4 <- TC02 channel 7.
        state.devices[DeviceId::Pp01 as usize].solenoid_input = Some(false);
        set(&mut state, DeviceId::Sv02, 4);
        let (ch, _) = live_reagent(&state).unwrap();
        assert_eq!((ch, tc01_for_tc02(ch), bottle_for_tc02(ch)), (7, 4, 4));
        // SV02 on a slot port: no reagent tube is live.
        set(&mut state, DeviceId::Sv02, 12);
        assert!(live_reagent(&state).is_none());
        // Solenoid towards SV01, port 2: C2's second dip tube via TC02 channel 5 -> TC01 channel 8.
        state.devices[DeviceId::Pp01 as usize].solenoid_input = Some(true);
        set(&mut state, DeviceId::Sv01, 2);
        let (ch, _) = live_reagent(&state).unwrap();
        assert_eq!((ch, tc01_for_tc02(ch), bottle_for_tc02(ch)), (5, 8, 2));
        // The lit chain is continuous: the bundle tube starts where the valve tube ends on TC01.
        let layout = Layout::default();
        let chain: Vec<ReagentTube> = route_reagents(&layout).into_iter().filter(|r| r.tc02 == ch).collect();
        assert_eq!(chain.len(), 2, "bundle tube + dip tube");
        assert_eq!(chain[0].pts[0], tc_channel(&layout, "TC01", 8, false));
        assert_eq!(*chain[0].pts.last().unwrap(), tc_channel(&layout, "TC02", 5, false));
        assert_eq!(chain[1].pts[0], tc_channel(&layout, "TC02", 5, true));
    }

    #[test]
    fn right_click_targets_match_drawn_shapes() {
        let l = Layout::default();
        let big = Settings::default().label_scale;
        let sv02 = l.pos("SV02");
        assert_eq!(target_at(&l, port_pos(sv02, 100.0, 16, 15), big), Target::Port(DeviceId::Sv02, 15));
        assert_eq!(target_at(&l, sv02, big), Target::Device(DeviceId::Sv02));
        assert_eq!(target_at(&l, pump_card_rect(&l, DeviceId::Pp02, big).center(), big), Target::Device(DeviceId::Pp02));
        assert_eq!(target_at(&l, l.pos("SLOT1"), big), Target::Slot(1));
        assert_eq!(target_at(&l, l.pos("C4"), big), Target::Bottle(4));
        assert_eq!(target_at(&l, l.pos("COIL"), big), Target::Part("COIL"));
        assert_eq!(target_at(&l, l.pos("TAG_SV03_WASTE"), big), Target::Tag("TAG_SV03_WASTE", DeviceId::Sv03, 7));
        assert_eq!(target_at(&l, pos2(20.0, 20.0), big), Target::Background);
        // Every tag names a port that really is plumbed to that kind of destination.
        for (key, _) in DEFAULTS.iter().filter(|(k, _)| k.starts_with("TAG")) {
            let (id, port) = tag_port(key);
            assert_eq!(id.port_label(port), tag_text(key), "{key}");
        }
    }

    /// True when two axis-aligned segments run on the same line and share more than a point.
    fn overlaps(a: (Pos2, Pos2), b: (Pos2, Pos2)) -> bool {
        let horiz = |s: (Pos2, Pos2)| (s.0.y - s.1.y).abs() < 0.01;
        let vert = |s: (Pos2, Pos2)| (s.0.x - s.1.x).abs() < 0.01;
        // Parallel runs closer than this read as one tube, even with a gap between them.
        let span = |lo: f32, hi: f32, lo2: f32, hi2: f32| lo.max(lo2) < hi.min(hi2) + 20.0;
        if horiz(a) && horiz(b) && (a.0.y - b.0.y).abs() < 1.0 {
            return span(a.0.x.min(a.1.x), a.0.x.max(a.1.x), b.0.x.min(b.1.x), b.0.x.max(b.1.x));
        }
        if vert(a) && vert(b) && (a.0.x - b.0.x).abs() < 1.0 {
            return span(a.0.y.min(a.1.y), a.0.y.max(a.1.y), b.0.y.min(b.1.y), b.0.y.max(b.1.y));
        }
        false
    }
}
