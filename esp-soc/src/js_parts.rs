//! Parts whose behaviour lives in the host (JavaScript), for vibe-iot's simulator.
//!
//! The vibe board (`crate::vibe`) models its common parts in Rust. Everything else — e-paper
//! panels, character LCDs, ultrasonic rangers, clocks, keypads — is a JS model the page (or
//! Node) loads, so a new part is a TypeScript file and not a new WebAssembly build. The board
//! talks to those models through [`JsHost`], synchronously, from inside the emulation:
//!
//! - **pads**: a pin the part listens on changed what the MCU does to it (drives it low or
//!   high, or released it to an input). Reported as a drive state, not a level, because a
//!   single-wire sensor's start signal ends with a release that writes no level.
//! - **I2C**: the part is a target on I2C0; the board forwards each address phase, byte and stop.
//! - **SPI**: bytes clocked out on the part's SCK/MOSI; the part gates them by its own CS.
//! - **edges**: what the part puts on its lines, as levels at absolute board cycles (and
//!   releases, handing an open-drain line back to the pull-up). The board asks for the next one
//!   as a deadline and delivers them as board edges, exactly like the DHT model's.
//!
//! The host is process-global: one emulator lives at a time in the WebAssembly module. Native
//! builds (tests, the CLI) install one with [`set_host`]; without one, JS parts are inert.
use std::sync::Mutex;

/// What the MCU does to a pad.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Drive {
    Low = 0,
    High = 1,
    /// Released: an input, high impedance.
    Z = 2,
}

/// One change a JS part puts on a line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JsEdge {
    pub cycle: u64,
    pub pin: u8,
    /// The level, or None: hand the line back to the MCU (its pull decides the level).
    pub level: Option<bool>,
}

/// I2C operations forwarded to a JS target.
pub mod i2c_op {
    /// Address phase for a write. Returns 1 to ACK.
    pub const START_WRITE: u32 = 0;
    /// Address phase for a read. Returns 1 to ACK.
    pub const START_READ: u32 = 1;
    /// A data byte from the master (`arg`). Returns 1 to ACK.
    pub const WRITE: u32 = 2;
    /// A data byte to the master. Returns the byte.
    pub const READ: u32 = 3;
    pub const STOP: u32 = 4;
}

/// The host side of JS parts. `part` is the part's index in the board's config.
pub trait JsHost: Send {
    fn pad(&mut self, part: u32, pin: u8, drive: Drive, cycle: u64);
    fn i2c(&mut self, part: u32, op: u32, arg: u8) -> u32;
    fn spi(&mut self, part: u32, bytes: &[u8]);
    /// The earliest pending edge of any part.
    fn next_edge(&mut self) -> Option<u64>;
    /// Removes and returns the earliest pending edge at or before `upto`.
    fn pop_edge(&mut self, upto: u64) -> Option<JsEdge>;
}

static HOST: Mutex<Option<Box<dyn JsHost>>> = Mutex::new(None);

/// Installs (or, with None, removes) the host every board's JS parts talk to.
pub fn set_host(host: Option<Box<dyn JsHost>>) {
    *HOST.lock().unwrap_or_else(|e| e.into_inner()) = host;
}

/// Runs `f` with the host, or returns `default` when none is installed.
pub fn with_host<R>(default: R, f: impl FnOnce(&mut dyn JsHost) -> R) -> R {
    match HOST.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
        Some(h) => f(h.as_mut()),
        None => default,
    }
}

/// An I2C target answered by a JS part.
pub struct JsI2c {
    pub part: u32,
}

impl esp_periph::i2c::I2cDevice for JsI2c {
    fn start(&mut self, read: bool) -> bool {
        let op = if read { i2c_op::START_READ } else { i2c_op::START_WRITE };
        with_host(0, |h| h.i2c(self.part, op, 0)) != 0
    }
    fn write(&mut self, b: u8) -> bool { with_host(0, |h| h.i2c(self.part, i2c_op::WRITE, b)) != 0 }
    fn read(&mut self) -> u8 { with_host(0xff, |h| h.i2c(self.part, i2c_op::READ, 0)) as u8 }
    fn stop(&mut self) { with_host((), |h| { h.i2c(self.part, i2c_op::STOP, 0); }) }
}

#[cfg(test)]
pub mod test_host {
    //! A host for tests: records what the board sent and plays back scripted edges.
    use super::*;
    use std::sync::{Arc, Mutex};

    #[derive(Default)]
    pub struct Log {
        pub pads: Vec<(u32, u8, Drive, u64)>,
        pub i2c: Vec<(u32, u32, u8)>,
        pub spi: Vec<(u32, Vec<u8>)>,
        pub edges: Vec<JsEdge>,
        /// What READ returns, in turn.
        pub reads: Vec<u8>,
    }

    pub struct TestHost(pub Arc<Mutex<Log>>);

    impl JsHost for TestHost {
        fn pad(&mut self, part: u32, pin: u8, drive: Drive, cycle: u64) { self.0.lock().unwrap().pads.push((part, pin, drive, cycle)); }
        fn i2c(&mut self, part: u32, op: u32, arg: u8) -> u32 {
            let mut l = self.0.lock().unwrap();
            l.i2c.push((part, op, arg));
            if op == i2c_op::READ { if l.reads.is_empty() { 0xff } else { l.reads.remove(0) as u32 } } else { 1 }
        }
        fn spi(&mut self, part: u32, bytes: &[u8]) { self.0.lock().unwrap().spi.push((part, bytes.to_vec())); }
        fn next_edge(&mut self) -> Option<u64> { self.0.lock().unwrap().edges.iter().map(|e| e.cycle).min() }
        fn pop_edge(&mut self, upto: u64) -> Option<JsEdge> {
            let mut l = self.0.lock().unwrap();
            let i = l.edges.iter().enumerate().filter(|(_, e)| e.cycle <= upto).min_by_key(|(_, e)| e.cycle)?.0;
            Some(l.edges.remove(i))
        }
    }

    /// Tests that install the global host run one at a time.
    pub static SERIAL: Mutex<()> = Mutex::new(());
}
