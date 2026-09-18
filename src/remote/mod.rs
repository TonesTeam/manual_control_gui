//! The split between the machine with the adapters and the machine with the
//! screen.
//!
//! [`server`] runs on the rig's computer, owns the RS485 bus and publishes its
//! state; [`client`] runs behind the GUI's `Bus` and makes that state look
//! local. They meet over the line protocol in [`crate::wire`].

pub mod client;
pub mod server;
