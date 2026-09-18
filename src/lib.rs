//! Tones liquid processing rig: monitor, manual control and the headless
//! control server.
//!
//! The crate splits in two along the [`bus`] module. Everything below it —
//! [`protocol`], [`devices`], [`config`], [`bus`], [`remote`], [`wire`] — is
//! plain `std` and builds without a display, which is what `tstand_server`
//! runs on the rig's Raspberry Pi. Everything above it ([`app`],
//! [`schematic`], [`tracking`]) is the egui front end and is gated behind the
//! default `gui` feature.

pub mod bus;
#[cfg(feature = "can")]
pub mod can;
pub mod config;
pub mod controller_api;
pub mod detect;
pub mod devices;
pub mod fluidics;
pub mod protocol;
pub mod remote;
pub mod rig;
pub mod routine;
pub mod sensors;
pub mod temperature;
pub mod wire;

#[cfg(feature = "gui")]
pub mod app;
#[cfg(feature = "gui")]
pub mod schematic;
#[cfg(feature = "gui")]
pub mod tracking;
