//! The host side of `esp_soc::js_parts` in the browser and Node: each call is an import the
//! page's JS supplies (`env.vibe_js_*`, see vibe-iot's shared/esp32sim.mjs). Cycles cross as f64,
//! exact below 2^53.
use esp_soc::js_parts::{Drive, JsEdge, JsHost};

#[cfg(target_arch = "wasm32")]
#[link(wasm_import_module = "env")]
extern "C" {
    fn vibe_js_pad(part: u32, pin: u32, drive: u32, cycle: f64);
    fn vibe_js_i2c(part: u32, op: u32, arg: u32) -> u32;
    fn vibe_js_spi(part: u32, ptr: *const u8, len: usize);
    /// The earliest pending edge, or a negative number for none.
    fn vibe_js_next_edge() -> f64;
    /// Pops the earliest edge at or before `upto`: `pin | level << 8 | release << 9`, or -1. Its
    /// cycle is then `vibe_js_edge_cycle()`.
    fn vibe_js_pop_edge(upto: f64) -> i32;
    fn vibe_js_edge_cycle() -> f64;
}

pub struct WasmJsHost;

#[cfg(target_arch = "wasm32")]
impl JsHost for WasmJsHost {
    fn pad(&mut self, part: u32, pin: u8, drive: Drive, cycle: u64) {
        // SAFETY: plain values cross to the host; it keeps nothing.
        unsafe { vibe_js_pad(part, pin as u32, drive as u32, cycle as f64) }
    }
    fn i2c(&mut self, part: u32, op: u32, arg: u8) -> u32 {
        // SAFETY: as above.
        unsafe { vibe_js_i2c(part, op, arg as u32) }
    }
    fn spi(&mut self, part: u32, bytes: &[u8]) {
        // SAFETY: the host reads `len` bytes at `ptr` during the call and keeps no reference.
        unsafe { vibe_js_spi(part, bytes.as_ptr(), bytes.len()) }
    }
    fn next_edge(&mut self) -> Option<u64> {
        // SAFETY: as above.
        let c = unsafe { vibe_js_next_edge() };
        (c >= 0.0).then_some(c as u64)
    }
    fn pop_edge(&mut self, upto: u64) -> Option<JsEdge> {
        // SAFETY: as above.
        let packed = unsafe { vibe_js_pop_edge(upto as f64) };
        if packed < 0 { return None; }
        // SAFETY: as above.
        let cycle = unsafe { vibe_js_edge_cycle() } as u64;
        let release = packed >> 9 & 1 == 1;
        Some(JsEdge { cycle, pin: (packed & 0xff) as u8, level: if release { None } else { Some(packed >> 8 & 1 == 1) } })
    }
}

/// Native builds of this crate (its tests) have no page: the parts stay inert.
#[cfg(not(target_arch = "wasm32"))]
impl JsHost for WasmJsHost {
    fn pad(&mut self, _: u32, _: u8, _: Drive, _: u64) {}
    fn i2c(&mut self, _: u32, _: u32, _: u8) -> u32 { 0 }
    fn spi(&mut self, _: u32, _: &[u8]) {}
    fn next_edge(&mut self) -> Option<u64> { None }
    fn pop_edge(&mut self, _: u64) -> Option<JsEdge> { None }
}
