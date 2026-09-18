# Where this was left, 18 September 2026

The rig is powered down and safe: every motor idle, the Peltier PID stopped,
SV02 parked on waste, and no sensor stop armed.

## Two faults to look at before running anything

1. **The slot RTD is faulted** — `RTD_FAULT, RTD_OVUV`, temperature reading
   nothing. It failed once earlier in the session, recovered on its own, and
   failed again at the end. Intermittent, so probably a connector rather than a
   dead sensor. The board refuses to regulate while it is faulted, so nothing
   will heat until it is fixed.

2. **The CAN adapter re-enumerates.** It moved from `/dev/ttyACM0` to
   `ttyACM2` mid-session, taking temperature and the liquid sensors with it.
   The software now resolves it through `/dev/serial/by-id`, which survives
   that, and `temp_port` should be left **empty** so it does. At the very end
   both boards stopped reporting again, which looks like the same thing.

3. **A bubble keeps parking on A4/A5.** One run logged 796 sensor changes on
   channel 4 alone. The detector treats that correctly — flicker means no
   liquid column — but the line itself would benefit from being purged.

## What the rig now is

`tstand_server` runs on the Pi and owns both adapters; the GUI is a view of it
over TCP. Start it with:

```sh
deploy/deploy.sh                 # sync, build on the Pi
ssh rpi@tonespi.local
cd manual_control_gui && ./target/release/tstand_server --port /dev/ttyUSB0 --token <secret>
```

It is **not** a boot service, which is why it did not come back after the Pi
rebooted mid-session. `deploy/deploy.sh --install-service` installs one; the
reason it has not been done is that it would then own the RS485 bus at boot and
controller_v2 could not use it.

The Pi answered on both `192.168.1.114` and `.205` during the session; mDNS
(`tonespi.local`) resolved intermittently from this machine.

## Next steps, in the order they matter

1. Fix the RTD, then heating works with the existing Set/Hold controls.
2. Re-run the three-segment calibration with the line cleared past S1 and the
   slot drained first — see `docs/measurements/README.md` for why the last run
   only produced one trustworthy figure.
3. The prime routine still uses chunked moves. Now that stop-on-detect works
   (`BusCmd::ArmStop`), it should use that instead: fewer moving parts, and it
   stops on the front rather than on a chunk boundary.
4. The scripts driving the last measurements were one-off Python against the
   wire protocol. The two lessons they taught are worth keeping wherever they
   end up: never sleep without draining the socket (the server drops clients
   that fall behind), and wait for a move to *start* before waiting for it to
   finish, or the idle check passes on the previous command.
