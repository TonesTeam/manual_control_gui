# manual_control_gui

Rust (egui) program to monitor and manually control the Tones liquid processing rig in real time: both piston pumps and all three selector valves, drawn as the fluidic schematic from `Tones_Liqud_Processing.drawio`, plus the `signal_sender` protocol commands for controller_v2.

It runs either as one program on the machine the adapters are plugged into, or split in two: **`tstand_server`** on the rig's computer, which owns the RS485 bus, and the GUI on a PC that watches and commands it over the network. See [Running the rig from another machine](#running-the-rig-from-another-machine).

## Tabs

- **Monitor**: the live schematic. Valve rotors point at the current port, pump barrels fill with piston position, the active flow path is highlighted, and moving dots show which way liquid is going. When PP01 is connected to a reagent port, the highlight continues through TC01, the TC01–TC02 bundle and TC02 to the dip tube, and the bottle's outline lights up. Each pump has a readout card (state, steps, µL, fill %, direction and flow rate, speed, target) and each valve shows its current port and a state badge; the side panel repeats these in large type. Click a device to control it from the side panel: valve port buttons labelled from the diagram; pump target, aspirate/dispense in steps or µL, speed, solenoid direction, home, forced reset and stop. **STOP ALL** is always in the top bar. The side panel also carries the [temperature](#temperature) readout and its setpoint, and the fitted slot shows its temperature on the schematic.
  Tubes are routed automatically and kept aligned: they leave each port in the direction it faces, run on evenly spaced parallel lanes, never share a segment, and have rounded corners. Valve port numbering follows the drawio (SV02 counter-clockwise with 16 at the top; SV01/SV03 clockwise with 4 at the top), so reagent ports face TC01, slot ports face the slots and waste ports point down.
  **Slot and bottle states.** Every slot and reagent bottle shows a state badge and an estimated level.
  - **Slots:**
    - FILLING / DRAINING while a pump moves liquid on the slot's path
    - FILL PATH / DRAIN PATH when a valve is set to the slot
    - BUSY or MISSING as reported by controller_v2 `/slot-status`
    - otherwise FILLED / EMPTY
  - **Bottles:** DRAWING / RETURN / LINKED while connected to PP01, otherwise OK, LOW (under 10 %) or EMPTY.
  - **How levels are estimated:** nothing on the rig measures liquid levels. Levels come from piston travel, counted only while the path stays the same between readings. Correct them from the right-click menu (mark a bottle refilled or set its level; mark a slot empty or full). With the RS485 backend, levels are saved to `tstand_levels.json`. The simulator starts full on every run and never writes that file.
  **Right-click** anything on the schematic for a menu that depends on what you clicked:
  - **Valve or port:** switch to that port or pick one from a list, reset, stop.
  - **Pump:** aspirate or dispense a set volume, move to 0–100 %, set speed and solenoid, home, forced reset, stop.
  - **Slot:** select its fill or drain path, or start a protocol for it.
  - **Bottle:** connect it to PP01 through SV01 or SV02.
  - **Waste or router tag:** switch the valve port that feeds it.
  - **Empty canvas:** stop all, home all.

  Every menu can also reset the component's position and save the layout.
  Turn on **Move objects** to drag any component (valves, pumps, coil, sensors, TC blocks, bottles, slots, waste tags) to a new place. Connections re-route as you drag, and positions snap to a 10 px grid on release. **Save layout** writes `tstand_layout.json` next to the executable; **Default layout** restores the drawio arrangement. `--edit-layout` starts in this mode, and `--demo` runs a simulated slot fill and drain.
- **Protocols**: the `signal_sender` workflow in a form. Build a packet (run protocol, robot step, drain, delay…), estimate its time, send it, then pause/resume/abort by task ID. Slot and protocol progress come from `/slot-status` and `/get-protocol-data`.
- **Datasheets**: specs for every part number, the port map for each valve, and the RS485 command and status tables, with links to the Runze sources.
- **Log**: command results and faults. Tick *Raw frames* to see every TX/RX frame in hex.
- **Settings**: backend, port, baud rate, slave addresses, poll rate, pump calibration, controller address, UI scale and schematic label size, plus the CAN temperature board and the rig's plumbed ports and fitted slots.

## Screenshots

**Draining slot 1.** PP02 pulls liquid out of slot 1 through SV03 port 6 (active path, flow dots, DRAINING badge). The side panel shows device, slot and reagent states and the selected pump's readout.

![Monitor tab while PP02 drains slot 1](docs/images/monitor-draining.png)

**Drawing reagent.** PP01 aspirates from C1 through SV01 port 1. The highlight runs from the pump through TC01, the bundle and TC02 to the bottle, and C1 shows DRAWING.

![Monitor tab while PP01 draws reagent from C1](docs/images/monitor-drawing-reagent.png)

**Right-click menu on a valve port.**

![Right-click menu on SV02 port 15](docs/images/context-menu.png)

Pump manufacturer documentation is in [`docs/datasheets/pumps`](docs/datasheets/pumps/README.md).

## Hardware connections

- USB to RS485 adapter (pumps and valves, 9600 8N1 by default)
- WeAct USB2CAN adapter (`/dev/ttyACM0`) — the Peltier slot-temperature board, at 1 Mbit/s

The optical liquid sensors are on the same CAN adapter. One board carries six fibre-optic detectors, numbered `A0`–`A5` in the firmware's own parameter table, and this program reads them alongside the temperature board.

> **The CAN adapter is opened exclusively.** Only one program may hold it, so controller_v2 and this program cannot both use it at once. Turn off *Settings → Temperature* here, or stop controller_v2.

> **One bus master at a time.** Only one program may drive the RS485 adapter. If controller_v2 is running on it, stop it before choosing the *RS485 serial* backend here — otherwise both talk on the bus at once and corrupt each other's frames. The same applies to `tstand_server`: it holds the port for as long as it runs, so a GUI set to *RS485 serial* against the same adapter is the same mistake. The *Protocols* tab only uses HTTP and is safe to use alongside controller_v2.

## Building and running

Runs on Windows, macOS and Linux (x86_64 and ARM). Install Rust 1.88 or newer from <https://rustup.rs>, then:

```sh
cargo run --release
```

The checkout builds two programs:

| Binary | What it is | Build |
| --- | --- | --- |
| `tstand_controler` | the GUI in this README | `cargo build --release` |
| `tstand_server` | the headless control server for the rig's computer | `cargo build --release --no-default-features --features can --bin tstand_server` |

`--no-default-features` drops eframe, so the server needs none of the display libraries listed below — which is what makes it buildable on a headless Raspberry Pi.

- **Windows:** the release build is a GUI app with no console window. Serial ports show up as `COM3` and so on; check Device Manager.
- **macOS:** pick the `/dev/cu.usbserial-*` port, not `/dev/tty.*`.
- **Linux:** building needs the X11/Wayland development headers:
  ```sh
  sudo apt install libxkbcommon-dev libwayland-dev libx11-dev libxcursor-dev libxrandr-dev libxi-dev libgl1-mesa-dev
  ```
  To open `/dev/ttyUSB0` without root, add yourself to the `dialout` group (`sudo usermod -aG dialout $USER`, then log in again). No libudev is needed.

### Screens and DPI

- **Display scaling:** the window follows the operating system's display scaling, including per-monitor DPI and moving between screens.
- **Window size:** it opens centered and never bigger than the monitor. The minimum size is 800×520 points, and the side panel width adapts to the window.
- **Zoom:** press Ctrl/Cmd `+`/`-`, or set **Settings → Display → UI scale**. **Schematic label size** scales the readouts on the diagram. Settings also shows the current screen scale.
- **Line thickness:** lines on the schematic never get thinner than one physical pixel.

### Graphics

The default renderer is wgpu: Direct3D 12 or Vulkan on Windows, Metal on macOS, Vulkan or OpenGL on Linux. On virtual machines, remote desktops or old GPUs where it fails to start, use OpenGL instead:

```sh
TSTAND_RENDERER=glow cargo run --release
```

On Windows PowerShell, set it first with `$env:TSTAND_RENDERER="glow"`.

### Where files are stored

`tstand_settings.json`, `tstand_layout_v2.json` and `tstand_levels.json` are kept next to the executable when that folder is writable. `tstand_server` uses the same rule for its own `tstand_server.json`. Otherwise they go to:
- Windows: `%APPDATA%\TonesLiquidProcessing`
- macOS: `~/Library/Application Support/TonesLiquidProcessing`
- Linux: `~/.config/TonesLiquidProcessing`

### Prebuilt binaries

Every push runs `.github/workflows/build.yml`, which tests and builds release binaries for Windows, macOS (Apple Silicon and Intel) and Linux, plus `tstand_server` cross-built for 64-bit ARM Linux — the Raspberry Pi build, ready to copy to the rig. The binaries are attached to the workflow run as artifacts.

The program starts with the **Simulator** backend, which needs no hardware and follows datasheet timing. To use real hardware, go to Settings, choose *RS485 serial*, pick the port and click **Apply & save**. It may be necessary to home components before other commands work.

| Device | Role | Part number | Slave ID |
| ------ | ---- | ----------- | -------- |
| PP01 | Main solenoid piston pump | ZSB-LS-1.8-1-3-M-Q | 1 |
| PP02 | Drain piston pump | ZSB08-LS-0.9-1-5-1-Q | 2 |
| SV01 | Reagent / wash selector, 8 port | QHF-SV07-X-S-T08-K1.2-S | 3 |
| SV02 | Main selector, 16 port | QHF-SV07-X-S-T16-K1.0-S | 4 |
| SV03 | Drain selector, 8 port | QHF-SV07-X-S-T08-K1.2-S | 5 |

## Running the rig from another machine

The rig's computer — `tonespi.local` — has the adapters; the screen is somewhere else. `tstand_server` runs there and owns the bus, and the GUI becomes a view of it: every reading on the schematic comes from the server, and every button sends a command to it. Nothing about the Monitor, Protocols or Log tabs changes.

```
     PC (GUI)  ──────TCP 7373──────►  tonespi.local
   tstand_controler                    tstand_server
                                         ├── /dev/ttyUSB0 ── RS485 ── pumps, valves
                                         └── 127.0.0.1:3000 ──────── controller_v2
```

Several GUIs can watch one rig at once. They all see the same state, because the server holds it, and any of them can send commands.

### On the rig's computer

Install Rust from <https://rustup.rs>, then build just the server. It needs no display libraries:

```sh
cargo build --release --no-default-features --features can --bin tstand_server
./target/release/tstand_server --port /dev/ttyUSB0 --token pick-something
```

`--features can` adds the temperature board; it needs libudev (`libudev-dev` on Debian, already present with systemd). Leave it out for a server that only drives RS485.

`--list-ports` prints the serial ports it can see. `--simulator` runs the built-in simulator instead of the adapter, which is enough to try a GUI against it. `--help` lists the rest.

To keep it running across reboots, install the systemd unit:

```sh
deploy/deploy.sh --install-service
```

That syncs this checkout to the rig, builds it there, installs `deploy/tstand-server.service` and starts it. Put the token in `/etc/tstand-server.env` (`TSTAND_TOKEN=...`), which the unit reads. Afterwards `deploy/deploy.sh` alone syncs, rebuilds and restarts. `journalctl -u tstand-server -f` follows its log.

The user it runs as must be in the `dialout` group to open `/dev/ttyUSB0`.

### On the PC

In **Settings → Connection**, choose *Remote server*, enter the address (`tonespi.local:7373` — the port may be left off) and the token, then **Apply & save**. The top bar then shows the server's name instead of a local port, and the *Log* tab carries the server's own log alongside this window's.

Port, slave addresses, poll rate and pump calibration all belong to the machine with the adapter. The GUI reads them from the server when it connects and shows them, so a second operator opening a GUI sees how the rig is really set up rather than their own saved guess. Editing them and pressing **Apply & save** writes them back to the server, where they persist in `tstand_server.json`.

*Simulator* and *RS485 serial* still work as before and run entirely on the PC, which is what to use for trying out the interface, or for plugging the adapter into a laptop directly.

### What happens when the link drops

The GUI reconnects by itself, roughly every two seconds, and says why it cannot in the top bar and the log. While it is down every device shows OFFLINE rather than its last reading, so a stale piston position is never mistaken for a live one, and commands sent meanwhile are refused out loud instead of silently queued.

**The rig keeps running.** The server does not stop the pumps when a GUI disappears — a protocol that is part-way through finishes. Closing the window is not an abort; use **STOP ALL** for that.

### Security

The token is a shared secret checked on connect, and the traffic is plain JSON over TCP: enough to keep the rig from being driven by accident on a lab network, not enough to stand up to someone hostile on it. Starting the server without `--token` lets in anyone who can reach the port, and it says so in its log.

For anything less trusted than a lab LAN, bind to loopback and tunnel over SSH instead:

```sh
# on the rig
./target/release/tstand_server --bind 127.0.0.1:7373 --port /dev/ttyUSB0
# on the PC
ssh -N -L 7373:localhost:7373 rpi@tonespi.local
```

then point the GUI at `localhost:7373`.

### controller_v2

controller_v2 listens on loopback where it runs, so a GUI on another machine cannot reach it directly. The server forwards the *Protocols* tab's requests for it, over the same connection. The address in **Settings → controller_v2 HTTP** is resolved on the server, so `127.0.0.1:3000` means the rig's own loopback.


## Temperature

One Peltier slot-temperature board (DRV8701 + STM32C092) holds the fitted slot at a setpoint over CAN. It runs its own PID; this program sets the target and starts or stops it, and watches the result.

The readout sits in the Monitor side panel and on the fitted slot in the schematic:

- **temperature, target and how far there is left to go**, plus the tolerance band;
- **bridge duty**, signed — the H-bridge reverses to cool, so `+100 %` is heating and `−100 %` is cooling — and the current it is drawing;
- **faults**, split into *active* (wrong now — a reason not to run) and *latched* (gone wrong since the last clear; event bits such as OVERCURRENT only ever appear here).

A trend under the reading plots the measurement against the target it was chasing, over a fixed 5 or 10 minute window — fixed rather than auto-scaled, so a curve that settles in ten seconds does not look like one that takes ten minutes.

**Hold temperature** hands the output to the board's PID, **Stop holding** takes it back and brakes the bridge. A setpoint can be set without starting the PID; it persists on the board across power cycles, which is why the target shown is read back from the board rather than remembered here.

Gains, autotune and manual duty are deliberately absent: they belong to commissioning, with the `slot-temp-sensor-can` crate's own examples, not to a screen where a mis-click drives a Peltier.

> **One board per bus.** The protocol has no node address — every board answers on CAN IDs `0x700`–`0x71A` — so exactly one temperature board can be on the adapter. That is the same constraint as one fitted slot.

The CAN link is behind the `can` cargo feature, on by default. A display-only build can drop it (`--no-default-features --features gui`) and still show temperature, because a remote GUI reads it from the server like any other state.

## Liquid sensors

The optical board multiplexes six detectors into one CAN frame. Which of them this rig has wired, and where each one sits, is **Settings → Rig**; as shipped:

| Diagram | Channel | Where |
| --- | --- | --- |
| S1 | `A4` | between PP01 and the holding coil |
| S2 | `A5` | between the coil and SV02's common port |
| slot feed | `A2` | the fitted slot |

On the schematic each one is drawn as what it is — a fork with the tube running through it — and lights up when the beam is broken. A sensor whose board is not answering reads as unknown rather than dry, so a dead link never looks like an empty tube.

> **The sensor board's CAN id must not be `0x700`.** That is the *Peltier controller's command id*: asking for sensor states on it sends the temperature board a malformed command on every poll. The boards on this rig answer on `0x7FF`, and any id in `0x700`–`0x71F` is refused with a message rather than used.

The protocol is implemented here rather than taken from the `optical-sensor-can` crate, which lives in a private repository reached over SSH — depending on it would mean the program only builds for someone holding a deploy key, CI included. It is one request and one reply; see `src/sensors.rs`.


## Where the liquid is

The pump displaces a known volume, the tubing has a known bore, and the optical sensors say wet or dry at fixed points. Together that is enough to say which stretches of tube hold liquid, which hold air, and — the part that matters — which nobody knows.

The schematic fills each modelled tube accordingly: liquid in the flow colour, air as empty bore, and **unknown hatched with a `?`**. A tube nobody has watched since the rig was switched on is not empty, it is unknown; treating the two as the same is how a slug of old reagent gets pushed into a slot and called a wash. `Unknown` therefore survives every operation that cannot rule it out, and a fresh rig is hatched end to end.

**Volume is the coordinate, not length.** Every measurement this rig can make is a volume, because the pump counts steps. Length appears only at the end, by dividing by the bore:

| | |
| --- | --- |
| 1 mm bore | 0.785 µL per mm (π/4 — a µL is a mm³) |
| PP01 → S1 | 417 µL ≈ **53 cm** (measured) |
| S1 → S2, through the coil | 1583 µL ≈ **2.0 m** (measured; an earlier run said 1750 µL / 2.2 m) |
| PP01 → S2 total | ≈ 2000 µL ≈ 2.5 m |
| one 40-step coarse chunk | 83 µL ≈ 106 mm of tube |
| one 4-step creep chunk | 8.3 µL ≈ **10.6 mm** — the real resolution of "stop exactly at S2" |

**Landmarks are sensors**, because a sensor is the only thing that can tell you a front has arrived. Links run S1→S2 and so on, and a link's volume is measured by pushing a front from one sensor to the next and reading the steps off the pump. Anything not measured that way is carried as an estimate and says so on screen (`≈`, and a dimmer wall).

**Sensors are ground truth.** Counting pump steps alone drifts — backlash, compliance, a bore that is only nominally 1 mm. A sensor transition is exact at one point, so everything is restated to agree with it whenever one fires.

The model is saved to `tstand_fluidics.json`, so a restart does not start blind.

### Reading the sensors

A detector with liquid standing at it reads the same for as long as you care to look. Air with a film on the wall, or a bubble going past, flickers. So "is this liquid" is really "has this reading held still for long enough", and the run waits **two seconds of an unchanging reading** with the piston stopped before believing a front has arrived — the same window controller_v2 uses (`MIN_STREAM_TIME_MS`).

A reading that flips while the piston is stopped is something physically moving past the detector. Those are counted and reported as bubbles: a run that reports several is describing a frothy line, and its numbers are approximate.

The same fact answers two different questions, and the answers are opposite:

- **"Has the front arrived?"** A flicker is a *no*. There is no liquid column at the detector, so the run keeps pushing.
- **"Is the line clear of liquid?"** A flicker is a *yes*, for the same reason.

A detector that reads wet steadily while millilitres are pulled through it is neither: nothing is flowing at all, and the run says so and names the air port as the thing to check, rather than reporting froth it did not see.


## Deciding what is at a detector

The board gives a raw 8-bit reading and its own wet/dry bit. The bit alone is not enough to drive a pump from: a front sweeping past crosses the threshold on its way, and the bit flips as readily for that as for a tube that has genuinely filled.

Recorded on this rig at ~20 Hz, pulling a front back past the coil detector and pushing it out again (`tests/data/a5_front.txt`, 1414 samples, used as the test fixture):

| condition | reading | rolling sd over 0.5 s |
| --- | --- | --- |
| settled liquid | 252 | **0.00** |
| settled air | ~107 | ~0.4 |
| front going past | swinging 43–147 | up to **71** |

A settled detector does not vary at all. So **variance says whether to believe the level, and only then does the level say which it is** — a mean alone cannot tell 252-going-down from 107-going-up at the moment both read 150.

### Which algorithm

The recording contains exactly two real transitions. Anything above two is an algorithm reacting to the journey rather than the destination (`cargo run --example detect_compare`):

| algorithm | changes of mind | first call |
| --- | --- | --- |
| *ideal* | **2** | ~6.1 s |
| bare threshold (what the board does) | 24 | 5.94 s |
| low-pass 3 Hz, then threshold | 18 | 6.06 s |
| low-pass 1 Hz, then threshold | 14 | 6.19 s |
| **low-pass + high-pass gate** | **2** | 7.99 s |
| variance gate | 2 | 8.80 s |

**Low-pass filtering alone never gets there** — even a 1 Hz corner still changes its mind 14 times, because smoothing the level cannot fix a level-based decision taken mid-sweep. What works is refusing to decide while the reading is moving.

The two filters answer different halves of the question, which is why both are here: the **low-pass keeps the level and discards the movement**, the **high-pass keeps the movement and discards the level**. Liquid-or-air is a level; a front or a bubble is movement. A third term, the spread across the window, catches a drift too slow for the high-pass corner — a constant slope gives a constant high-pass output, which sits under the gate if it is gentle enough.

Filters are specified by **cutoff frequency, with the coefficient derived per sample from the interval that actually elapsed**. A bare `alpha` only means something next to a sample rate, and this one's is not constant — the readings share a CAN adapter with the temperature exchanges and arrive when they arrive.

### Stopping a pump on what a detector sees

For stopping, the settled verdict is the wrong signal: it cannot be given until the front has gone past *and* the reading has been quiet again. The right one is **departure from a settled baseline**, which on the recording trips at 5.81 s — earlier than any verdict, and earlier than the board's own bit, because a settled reading varies by nothing and so the first sample that moves is already proof.

`BusCmd::ArmStop` arms it. The sensor thread on the rig's own machine is already reading the board ~20 times a second; when the condition is met it sends the stop itself rather than handing a verdict to something else to act on, because every hop is pump travel. Measured on the rig, commanding a 700-step dispense and arming on a 25-count departure:

```
commanded: dispense 700 from 1219   (would have ended at 519)
came to rest at 1194 — 25 steps, 52 µL, about 66 mm of tube, at 25 rpm
```

That is the achievable precision of stopping on a detector rather than at a position, and it scales with speed: the same stop at 8 rpm travels about a third as far.

> **The air and waste ports are one-way, by plumbing rather than by valve.** Air is drawn *in* through the air port and liquid is pushed *out* through waste; the reverse of either wets the air filter or pulls waste back up into the rig. Nothing downstream enforces that, so [`bus.rs`] refuses a pump move that would do it — on the machine holding the bus, so it applies to every client and every code path.


## The rig as it stands

The schematic draws the rig as designed: six slots, six reagent bottles, every port in use. A rig under commissioning is not that, and **Settings → Rig** carries the difference.

It decides what a port is called, whether it may be switched to, and which slots exist. Ports that are not listed are drawn struck through and greyed out, unfitted slots are drawn faint and empty, and the guard is enforced on the machine holding the bus — so it applies to every screen, not just the one that greys the button out.

As shipped it describes the current rig:

| | Port | Carries |
| --- | --- | --- |
| SV01 | 1 | Wash liquid |
| SV02 | 2 | Reagent line |
| SV02 | 9 | Waste |
| SV02 | 10 | Slot fill |
| SV02 | 16 | Air |
| SV03 | 1 | Slot drain |

One slot is fitted, with its detector on `A2`. Slot numbers follow the diagram, where SV02 port `16 − n` fills slot `n` and SV03 port `7 − n` drains it — so SV02 port 10 and SV03 port 1 are the two ends of **slot 6**.

**Restrict to plumbed ports** turns the guard off without losing the labels, for when commissioning needs to reach a capped line. *Whole diagram* restores the rig as drawn.


## Live monitoring

Each poll cycle sends every device a position query (`0x66` for pumps, `0x3E` for valves) and a motor status query (`0x4A`). Commands are queued on the same worker thread, so they never overlap with polling. A device that stops answering is retried every 2 s, so one missing device does not slow the rest.

## Tests

```sh
cargo test
```

`tests/remote.rs` runs a server and a GUI-side bus against each other on a loopback port, covering the split end to end: a command reaching the devices, state and log coming back, a refused token, a dropped link and two GUIs watching one rig.

To check the server builds as it does on the rig, without any display libraries:

```sh
cargo test --no-default-features
```
