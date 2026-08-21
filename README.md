# manual_control_gui

Rust program meant to make it possible to control Tones components manually. Specifically, both pumps and all 3 selector valves through an easy to use interface.

## Status

Is not being actively developed, only maintained.

## Requirements

- Rust (edition 2024) — see [Cargo.toml](Cargo.toml)

## Hardware connections

In order to run protocols, you must connect 1 USB cable

- USB to CAN adapter

## Building

```sh
cargo build --release
```

## Running

```sh
cargo run
```

You must go to settings and select correct COM port. Use Device Manager to find correct COM port. You must then input correct slave IDs.

| Component        | Slave ID |
| ---------------- | -------- |
| `Main pump`      | 1        |
| `Secondary pump` | 2        |
| `Selector 1`     | 3        |
| `Selector 2`     | 5        |
| `Selector 3`     | 4        |

Finally, it may be necessary to home components before you can run any other commands.
