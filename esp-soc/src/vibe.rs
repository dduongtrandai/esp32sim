//! A board assembled at run time from a part list, for vibe-iot's simulator. The ESP32-S3 builds
//! it for `vibe:<json>`, the classic ESP32 for `vibe-esp32:<json>`, the JSON being vibe-iot's SimConfig:
//! `{"parts":[{"kind":"ssd1306","pins":{"sda":8,"scl":9},"addr":60}, {"kind":"dht22","pins":{"data":4}},
//! {"kind":"led","pins":{"signal":2}}, …]}`. Parts are numbered by their place in the list; that
//! number is the `id` the page's controls use.
//!
//! - `ssd1306`: the OLED controller on I2C0 at `addr` (0x3C by default).
//! - `st7789` / `ili9341`: an SPI TFT on `sck`, `mosi`, `dc`, and `cs` and `rst` when wired, `width` x `height`
//!   of its 240 x 320 RAM, 240 x 320 by default (240 x 240 for the 1.3" and 1.54" ST7789 modules).
//!   The first OLED or TFT in the list is the board's display (binary message type 1).
//! - `ws2812`: a chain of `count` LEDs on `data`, fed by RMT frames or decoded from GPIO pulses;
//!   reported with each LED's colour.
//! - `dht22` / `dht11`: the sensor on `data`; the page sets `temperature` and `humidity`.
//! - `hx711`: the load-cell ADC on `dout` and `sck`, `rate` 10 or 80 conversions a second, with a
//!   load cell of `capacity` grams and `sensitivity` mV/V on channel A; the page sets `weight`, and
//!   `capacity`, `sensitivity` and `offset` (counts at no load) to calibrate against, and `noise`
//!   (counts rms, 0 for exact words). A `weight` from the page rings and settles like a real load.
//! - `led`, `buzzer`: outputs on `signal`, reported with their level and any PWM on the pin.
//!   A buzzer toggled by software reports the frequency of its edges.
//! - `button`: an active-low input on `signal`, idle high; the page drives it with `btn`.
//! - `analog`: a voltage on `signal`'s ADC pad; the page sets it with `adc`.
use std::sync::{Arc, Mutex};

use esp_periph::i2c::I2cDevice;

use crate::board::{BoardEdge, BoardModel, SpiPins, VirtualCycle};
use crate::devices::sensors::{self, SensorValues};
use crate::devices::{DcsPanel, Dht, DhtKind, Hx711, Ssd1306, Ssd1306I2c, Ws2812Chain};
use crate::json::{parse_json, Json};

/// An SPI TFT module: an ST77xx/ILI9341 controller (`DcsPanel`) behind its glass. The RAM is
/// 240 x 320; the glass shows the first `width` columns and `height` rows of it. The glass decides
/// how the controller's RAM looks:
/// - its scan direction: an ST7789 module scans RAM mirrored in both axes and an ILI9341 module
///   in x, which is why their drivers draw rotation 0 with MADCTL MX|MY and MX (Adafruit_ST7789,
///   Adafruit_ILI9341) and the picture comes out upright;
/// - its colour order: an ILI9341 module is BGR, an ST7789 one RGB;
/// - whether it is an IPS panel that shows inverted RAM unless the firmware sends INVON (ST7789
///   modules are, which is why their drivers send it).
struct Tft {
    kind: &'static str,
    panel: DcsPanel,
    sck: u8,
    mosi: u8,
    /// None: CS tied low on the module (many 240 x 240 ST7789 boards have no CS pin).
    cs: Option<u8>,
    dc: u8,
    rst: Option<u8>,
    cs_high: bool,
    width: usize,
    height: usize,
    flip_x: bool,
    flip_y: bool,
    bgr_glass: bool,
    inverted_glass: bool,
}

impl Tft {
    fn new(kind: &'static str, p: &Json) -> Option<Self> {
        let (sck, mosi, cs, dc) = (pin(p, "sck")?, pin(p, "mosi")?, pin(p, "cs"), pin(p, "dc")?);
        let st7789 = kind == "st7789";
        let width = (p.i64_or("width", 240).clamp(16, 240)) as usize;
        let height = (p.i64_or("height", 320).clamp(16, 320)) as usize;
        Some(Tft {
            kind,
            panel: if st7789 { DcsPanel::st7789() } else { DcsPanel::new(240, 320) },
            sck, mosi, cs, dc, rst: pin(p, "rst"),
            cs_high: cs.is_some(),
            width, height,
            flip_x: true,
            flip_y: st7789,
            bgr_glass: !st7789,
            inverted_glass: st7789,
        })
    }
    fn on_wires(&self, pins: &SpiPins) -> bool {
        let routed = pins.sclk != 0 || pins.mosi != 0;
        !routed || (pins.sclk >> self.sck & 1 == 1 && pins.mosi >> self.mosi & 1 == 1)
    }
    fn lit(&self) -> bool { self.panel.on && !self.panel.sleeping }
    fn version(&self) -> u64 { self.panel.pixels_written + self.panel.frames + self.panel.resets * 7 + self.lit() as u64 }
    /// What the glass shows, RGB565, row by row.
    fn frame(&self) -> Vec<u16> {
        let p = &self.panel;
        let swap = (p.madctl & 0x08 != 0) != self.bgr_glass;
        let invert = p.inverted != self.inverted_glass;
        let lit = self.lit();
        let mut out = Vec::with_capacity(self.width * self.height);
        for y in 0..self.height {
            let ry = if self.flip_y { self.height - 1 - y } else { y };
            let row = ry.min(p.rows - 1) * p.cols;
            for x in 0..self.width {
                let rx = if self.flip_x { self.width - 1 - x } else { x };
                let mut px = p.gram[row + rx.min(p.cols - 1)];
                if swap { px = (px & 0x07e0) | ((px & 0xf800) >> 11) | ((px & 0x001f) << 11); }
                if invert { px = !px; }
                out.push(if lit { px } else { 0 });
            }
        }
        out
    }
}

enum Part {
    Oled { addr: u8, state: Arc<Mutex<Ssd1306>> },
    /// An I2C sensor (devices::sensors) whose readings the page sets.
    Sensor { kind: &'static str, addr: u8, values: sensors::Shared },
    Tft(Box<Tft>),
    Strip { pin: u8, chain: Ws2812Chain },
    Dht(Dht),
    Hx711(Box<Hx711>),
    Led { pin: u8 },
    Buzzer { pin: u8, rises: [VirtualCycle; 2] },
    Button { pin: u8 },
    Analog { pin: u8 },
}

/// The most LEDs a strip part may have.
const MAX_STRIP: usize = 300;

pub struct VibeBoard {
    parts: Vec<(String, Part)>,
    /// Output levels of driven pins (output AND enable).
    levels: u64,
    now: VirtualCycle,
    gpio_events: u64,
    /// Board-time cycles per second: the SoC bus clock edges and deadlines are counted in.
    cycle_hz: u64,
}

fn pin(p: &Json, role: &str) -> Option<u8> {
    p.get("pins")?.get(role)?.as_i64().filter(|&n| (0..=48).contains(&n)).map(|n| n as u8)
}

impl VibeBoard {
    /// Builds the board from vibe-iot's SimConfig JSON. Parts without the pins they need are
    /// skipped, and so are kinds this board does not know.
    pub fn from_config(config: &str, cycle_hz: u64) -> Result<Self, String> {
        let us = cycle_hz / 1_000_000;
        let json = parse_json(config).map_err(|e| format!("vibe board config: {e}"))?;
        let list = json.get("parts").and_then(Json::as_arr).unwrap_or(&[]);
        let mut parts = Vec::new();
        for (i, p) in list.iter().enumerate() {
            let kind = p.str_or("kind", "");
            let part = match kind {
                "ssd1306" => {
                    let addr = p.i64_or("addr", 0x3c) as u8;
                    let height = p.i64_or("height", 64) as usize;
                    Some(Part::Oled { addr, state: Arc::new(Mutex::new(Ssd1306::new(height))) })
                }
                "bme280" | "bmp280" | "sht31" | "aht20" | "bh1750" | "mpu6050" => {
                    let kind: &'static str = match kind { "bme280" => "bme280", "bmp280" => "bmp280", "sht31" => "sht31", "aht20" => "aht20", "bh1750" => "bh1750", _ => "mpu6050" };
                    let addr = p.get("addr").and_then(Json::as_i64).map(|a| a as u8).or_else(|| sensors::default_address(kind));
                    addr.map(|addr| Part::Sensor { kind, addr, values: Arc::new(Mutex::new(SensorValues::default())) })
                }
                "st7789" => Tft::new("st7789", p).map(|t| Part::Tft(Box::new(t))),
                "ili9341" => Tft::new("ili9341", p).map(|t| Part::Tft(Box::new(t))),
                "ws2812" => pin(p, "data").map(|pin| {
                    let n = p.i64_or("count", 8).clamp(1, MAX_STRIP as i64) as usize;
                    Part::Strip { pin, chain: Ws2812Chain::new(n).with_gpio_clock(cycle_hz as u32) }
                }),
                "dht22" | "dht11" => pin(p, "data").map(|pin| {
                    let mut d = Dht::new(pin, if kind == "dht11" { DhtKind::Dht11 } else { DhtKind::Dht22 }, us);
                    if let Some(t) = p.get("temperature").and_then(Json::as_f64) { d.temperature = t; }
                    if let Some(h) = p.get("humidity").and_then(Json::as_f64) { d.humidity = h; }
                    Part::Dht(d)
                }),
                "hx711" => pin(p, "dout").zip(pin(p, "sck")).filter(|(d, s)| d != s).map(|(dout, sck)| {
                    let mut h = Hx711::new(dout, sck, p.i64_or("rate", 10).clamp(0, 80) as u32, us);
                    let num = |k: &str| p.get(k).and_then(Json::as_f64).filter(|v| v.is_finite());
                    if let Some(v) = num("capacity").filter(|&v| v > 0.0) { h.capacity = v; }
                    if let Some(v) = num("sensitivity").filter(|&v| v > 0.0) { h.sensitivity = v; }
                    if let Some(v) = num("offset") { h.offset = v; }
                    if let Some(v) = num("weight") { h.weight = v; }
                    if let Some(v) = num("noise").filter(|&v| v >= 0.0) { h.noise = v.min(1e6); }
                    Part::Hx711(Box::new(h))
                }),
                "led" => pin(p, "signal").map(|pin| Part::Led { pin }),
                "buzzer" => pin(p, "signal").map(|pin| Part::Buzzer { pin, rises: [0; 2] }),
                "button" => pin(p, "signal").map(|pin| Part::Button { pin }),
                "analog" => pin(p, "signal").map(|pin| Part::Analog { pin }),
                _ => None,
            };
            if let Some(part) = part { parts.push((i.to_string(), part)); }
        }
        Ok(Self { parts, levels: 0, now: 0, gpio_events: 0, cycle_hz })
    }

    /// The board's display: its first OLED or TFT, in the design's order.
    fn screen(&self) -> Option<&Part> {
        self.parts.iter().map(|(_, p)| p).find(|p| matches!(p, Part::Oled { .. } | Part::Tft(_)))
    }

    fn tfts(&mut self) -> impl Iterator<Item = &mut Tft> {
        self.parts.iter_mut().filter_map(|(_, p)| match p { Part::Tft(t) => Some(&mut **t), _ => None })
    }

    fn strips(&mut self) -> impl Iterator<Item = (u8, &mut Ws2812Chain)> {
        self.parts.iter_mut().filter_map(|(_, p)| match p { Part::Strip { pin, chain } => Some((*pin, chain)), _ => None })
    }

    fn dhts(&mut self) -> impl Iterator<Item = &mut Dht> {
        self.parts.iter_mut().filter_map(|(_, p)| match p { Part::Dht(d) => Some(d), _ => None })
    }

    fn hx711s(&mut self) -> impl Iterator<Item = &mut Hx711> {
        self.parts.iter_mut().filter_map(|(_, p)| match p { Part::Hx711(h) => Some(&mut **h), _ => None })
    }

    fn level(&self, pin: u8) -> bool { self.levels >> pin & 1 == 1 }
}

impl BoardModel for VibeBoard {
    fn name(&self) -> &'static str { "vibe" }

    fn gpio_output_at(&mut self, cycle: VirtualCycle, changes: &[(u8, bool)], enabled: u64, output: u64) {
        self.now = self.now.max(cycle);
        self.gpio_events += changes.len() as u64;
        self.levels = enabled & output;
        for &(pin, level) in changes {
            if !level { continue; }
            for (_, p) in self.parts.iter_mut() {
                if let Part::Buzzer { pin: bp, rises } = p {
                    if *bp == pin { *rises = [rises[1], cycle]; }
                }
            }
        }
        for d in self.dhts() { d.host_drive(cycle, enabled, output); }
        for h in self.hx711s() {
            if changes.iter().any(|&(p, _)| p == h.sck) { h.host_drive(cycle, enabled, output); }
        }
        for t in self.tfts() {
            for &(pin, level) in changes {
                if Some(pin) == t.cs { t.cs_high = level; }
                if pin == t.dc { t.panel.dc = level; }
                if Some(pin) == t.rst && !level { t.panel.reset(); }
            }
        }
        // a strip driven by bit-banging rather than RMT: decode its pulses
        for (pin, chain) in self.strips() {
            if changes.iter().any(|&(p, _)| p == pin) {
                chain.gpio_drive(cycle, enabled >> pin & 1 == 1, output >> pin & 1 == 1);
            }
        }
    }

    fn gpio_events(&self) -> u64 { self.gpio_events }

    fn rmt_frame(&mut self, pin: u8, bits: &[bool]) {
        for (p, chain) in self.strips() {
            if p == pin { chain.from_bits(bits); }
        }
    }

    fn uses_spi_pins(&self) -> bool { self.parts.iter().any(|(_, p)| matches!(p, Part::Tft(_))) }

    fn spi_transfer_pins(&mut self, _host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        for t in self.tfts() {
            if !t.cs_high && t.on_wires(&pins) {
                for &b in tx { t.panel.byte(b); }
            }
        }
        vec![0xff; rx_len] // no MISO on these modules
    }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        self.parts.iter().filter_map(|(_, p)| match p {
            Part::Oled { addr, state } => Some((0u8, *addr, Box::new(Ssd1306I2c::new(state.clone())) as Box<dyn I2cDevice>)),
            Part::Sensor { kind, addr, values } => sensors::make(kind, values.clone()).map(|d| (0u8, *addr, d)),
            _ => None,
        }).collect()
    }

    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        match self.screen()? {
            Part::Oled { state, .. } => {
                let s = state.lock().expect("SSD1306 mutex poisoned");
                Some((crate::devices::ssd1306::SSD1306_WIDTH as u32, s.height() as u32, s.frame(), s.version))
            }
            Part::Tft(t) => Some((t.width as u32, t.height as u32, t.frame(), t.version())),
            _ => None,
        }
    }
    fn display_version(&self) -> u64 {
        match self.screen() {
            Some(Part::Oled { state, .. }) => state.lock().expect("SSD1306 mutex poisoned").version,
            Some(Part::Tft(t)) => t.version(),
            _ => 0,
        }
    }
    fn display_frames(&self) -> u64 { self.display_version() }
    /// A TFT frame is 150 KB: push it once drawing settles rather than mid-frame, as esp32sim does
    /// for the other pixel-streaming panels.
    fn display_quiet_push(&self) -> bool { matches!(self.screen(), Some(Part::Tft(_))) }

    /// Buttons idle high (pressed pulls them low); the DHT line idles high on its pull-up; an
    /// HX711 drives DOUT itself.
    fn input_levels(&self) -> Vec<(u8, bool)> {
        self.parts.iter().filter_map(|(_, p)| match p {
            Part::Button { pin } => Some((*pin, true)),
            Part::Dht(d) => Some((d.pin, true)),
            Part::Hx711(h) => Some((h.dout, h.dout_level())),
            _ => None,
        }).collect()
    }

    fn next_deadline(&self) -> Option<VirtualCycle> {
        self.parts.iter().filter_map(|(_, p)| match p {
            Part::Dht(d) => d.next_deadline(),
            Part::Hx711(h) => h.next_deadline(),
            Part::Strip { chain, .. } => chain.gpio_deadline(),
            _ => None,
        }).min()
    }

    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.now = self.now.max(cycle);
        for d in self.dhts() { d.advance_to(cycle); }
        for h in self.hx711s() { h.advance_to(cycle); }
        for (_, chain) in self.strips() { chain.advance_gpio(cycle); }
    }

    fn take_edges(&mut self) -> Vec<BoardEdge> {
        let mut edges: Vec<BoardEdge> = self.dhts().flat_map(|d| d.take_edges()).collect();
        edges.extend(self.hx711s().flat_map(|h| h.take_edges()));
        edges.sort_by_key(|e| e.cycle);
        edges
    }

    fn released_inputs(&mut self) -> Vec<u8> { self.dhts().flat_map(|d| d.take_released()).collect() }

    fn named_pin(&self, name: &str) -> Option<u8> {
        let i: usize = name.parse().ok()?;
        self.parts.iter().find(|(id, _)| id.parse() == Ok(i)).and_then(|(_, p)| match p {
            Part::Button { pin } | Part::Led { pin } | Part::Buzzer { pin, .. } | Part::Analog { pin } => Some(*pin),
            Part::Dht(d) => Some(d.pin),
            Part::Hx711(h) => Some(h.dout),
            Part::Strip { pin, .. } => Some(*pin),
            Part::Oled { .. } | Part::Tft(_) | Part::Sensor { .. } => None,
        })
    }

    fn ui_pwm_pins(&self) -> Vec<u8> {
        self.parts.iter().filter_map(|(_, p)| match p { Part::Led { pin } | Part::Buzzer { pin, .. } => Some(*pin), _ => None }).collect()
    }

    fn ui_state(&mut self, pwm: &[(u8, Option<(f64, u32)>)]) -> Option<String> {
        let pwm_of = |pin: u8| pwm.iter().find(|(p, _)| *p == pin).and_then(|(_, o)| *o);
        let fmt_pwm = |o: Option<(f64, u32)>| o.map_or("null".to_string(), |(hz, duty)| format!("[{:.1},{}]", hz, duty));
        let mut items = Vec::new();
        for (id, p) in &self.parts {
            items.push(match p {
                Part::Oled { addr, state } => {
                    let s = state.lock().expect("SSD1306 mutex poisoned");
                    format!("{{\"id\":\"{id}\",\"kind\":\"ssd1306\",\"addr\":{addr},\"on\":{},\"height\":{}}}", s.is_on(), s.height())
                }
                Part::Sensor { kind, addr, values } => {
                    let v = *values.lock().expect("sensor values mutex poisoned");
                    let fields = match *kind {
                        "bme280" => format!("\"temperature\":{:.2},\"humidity\":{:.1},\"pressure\":{:.2}", v.temperature, v.humidity, v.pressure),
                        "bmp280" => format!("\"temperature\":{:.2},\"pressure\":{:.2}", v.temperature, v.pressure),
                        "sht31" | "aht20" => format!("\"temperature\":{:.2},\"humidity\":{:.1}", v.temperature, v.humidity),
                        "bh1750" => format!("\"lux\":{:.1}", v.lux),
                        _ => format!("\"temperature\":{:.2},\"ax\":{:.3},\"ay\":{:.3},\"az\":{:.3},\"gx\":{:.1},\"gy\":{:.1},\"gz\":{:.1}", v.temperature, v.accel[0], v.accel[1], v.accel[2], v.gyro[0], v.gyro[1], v.gyro[2]),
                    };
                    format!("{{\"id\":\"{id}\",\"kind\":\"{kind}\",\"addr\":{addr},{fields}}}")
                }
                Part::Tft(t) => format!(
                    "{{\"id\":\"{id}\",\"kind\":\"{}\",\"on\":{},\"width\":{},\"height\":{},\"frames\":{}}}",
                    t.kind, t.lit(), t.width, t.height, t.panel.frames),
                Part::Strip { pin, chain } => {
                    let leds: Vec<String> = chain.leds.iter().map(|c| format!("[{},{},{}]", c[0], c[1], c[2])).collect();
                    format!("{{\"id\":\"{id}\",\"kind\":\"ws2812\",\"pin\":{pin},\"updates\":{},\"leds\":[{}]}}", chain.updates, leds.join(","))
                }
                Part::Dht(d) => format!(
                    "{{\"id\":\"{id}\",\"kind\":\"{}\",\"pin\":{},\"temperature\":{:.1},\"humidity\":{:.1},\"reads\":{}}}",
                    if d.kind == DhtKind::Dht11 { "dht11" } else { "dht22" }, d.pin, d.temperature, d.humidity, d.reads),
                Part::Hx711(h) => format!(
                    "{{\"id\":\"{id}\",\"kind\":\"hx711\",\"pin\":{},\"sck\":{},\"weight\":{:.1},\"capacity\":{:.0},\"sensitivity\":{:.3},\"offset\":{:.0},\"gain\":{},\"raw\":{},\"reads\":{},\"noise\":{:.0}}}",
                    h.dout, h.sck, h.weight, h.capacity, h.sensitivity, h.offset, h.gain(), h.last, h.reads, h.noise),
                Part::Led { pin } => format!("{{\"id\":\"{id}\",\"kind\":\"led\",\"pin\":{pin},\"on\":{},\"pwm\":{}}}", self.level(*pin), fmt_pwm(pwm_of(*pin))),
                Part::Buzzer { pin, rises } => {
                    // a software tone: the period between the last two rising edges, while recent
                    let hz = if rises[0] != 0 && rises[1] > rises[0] && self.now.saturating_sub(rises[1]) < self.cycle_hz / 20 {
                        self.cycle_hz as f64 / (rises[1] - rises[0]) as f64
                    } else { 0.0 };
                    format!("{{\"id\":\"{id}\",\"kind\":\"buzzer\",\"pin\":{pin},\"on\":{},\"hz\":{:.0},\"pwm\":{}}}", self.level(*pin), hz, fmt_pwm(pwm_of(*pin)))
                }
                Part::Button { pin } => format!("{{\"id\":\"{id}\",\"kind\":\"button\",\"pin\":{pin}}}"),
                Part::Analog { pin } => format!("{{\"id\":\"{id}\",\"kind\":\"analog\",\"pin\":{pin}}}"),
            });
        }
        Some(format!("\"parts\":[{}]", items.join(",")))
    }

    fn ui_input(&mut self, id: &str, field: &str, value: f64) -> bool {
        for (pid, p) in self.parts.iter_mut() {
            if pid != id { continue; }
            if let Part::Sensor { values, .. } = p {
                return values.lock().expect("sensor values mutex poisoned").set(field, value);
            }
            if let Part::Dht(d) = p {
                match field {
                    "temperature" => { d.temperature = value.clamp(-40.0, 80.0); return true; }
                    "humidity" => { d.humidity = value.clamp(0.0, 100.0); return true; }
                    _ => {}
                }
            }
            if let Part::Hx711(h) = p {
                if !value.is_finite() { return false; }
                match field {
                    "weight" => { h.set_weight(value.clamp(-1e6, 1e6)); return true; }
                    "noise" if value >= 0.0 => { h.noise = value.min(1e6); return true; }
                    "capacity" if value > 0.0 => { h.capacity = value.min(1e7); return true; }
                    "sensitivity" if value > 0.0 => { h.sensitivity = value.min(100.0); return true; }
                    "offset" => { h.offset = value.clamp(-8_388_608.0, 8_388_607.0); return true; }
                    _ => {}
                }
            }
        }
        false
    }

    fn report(&self) -> String { format!("[vibe] {} parts", self.parts.len()) }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HZ: u64 = 240_000_000;
    const US: u64 = HZ / 1_000_000;
    const TONE_TIMEOUT: u64 = HZ / 20;

    const CONFIG: &str = r#"{"parts":[
        {"kind":"ssd1306","label":"OLED","pins":{"sda":8,"scl":9},"addr":60},
        {"kind":"dht22","label":"DHT22","pins":{"data":4}},
        {"kind":"led","label":"LED","pins":{"signal":2}},
        {"kind":"buzzer","label":"Buzzer","pins":{"signal":5}},
        {"kind":"button","label":"Button","pins":{"signal":0}},
        {"kind":"analog","label":"LDR","pins":{"signal":1}},
        {"kind":"servo","label":"unknown"},
        {"kind":"led","label":"no pins"}
    ],"unmodelled":[]}"#;

    #[test]
    fn config_builds_the_known_parts() {
        let mut b = VibeBoard::from_config(CONFIG, HZ).unwrap();
        assert_eq!(b.parts.len(), 6, "the servo and the pinless LED are skipped");
        let i2c = b.i2c_devices();
        assert_eq!(i2c.len(), 1);
        assert_eq!((i2c[0].0, i2c[0].1), (0, 0x3c));
        assert_eq!(b.input_levels(), vec![(4, true), (0, true)]);
        assert_eq!(b.ui_pwm_pins(), vec![2, 5]);
        assert_eq!(b.named_pin("4"), Some(0));
        assert!(b.display().is_some());
    }

    #[test]
    fn levels_tones_and_controls_reach_the_page() {
        let mut b = VibeBoard::from_config(CONFIG, HZ).unwrap();
        b.gpio_output_at(1_000, &[(2, true)], 1 << 2 | 1 << 5, 1 << 2);
        // a software tone on the buzzer: rising edges 1 ms apart = 1 kHz
        let ms = HZ / 1000;
        b.gpio_output_at(2 * ms, &[(5, true)], 1 << 2 | 1 << 5, 1 << 2 | 1 << 5);
        b.gpio_output_at(3 * ms, &[(5, true)], 1 << 2 | 1 << 5, 1 << 2 | 1 << 5);
        assert!(b.ui_input("1", "temperature", 31.5));
        assert!(!b.ui_input("2", "temperature", 1.0), "an LED has no temperature");
        let s = b.ui_state(&[(2, Some((5000.0, 32768))), (5, None)]).unwrap();
        assert!(s.contains(r#""id":"2","kind":"led","pin":2,"on":true,"pwm":[5000.0,32768]"#), "{s}");
        assert!(s.contains(r#""kind":"buzzer","pin":5,"on":true,"hz":1000"#), "{s}");
        assert!(s.contains(r#""temperature":31.5"#), "{s}");
        // the tone stops being reported once it is old
        b.advance_to(3 * ms + TONE_TIMEOUT + 1);
        assert!(b.ui_state(&[]).unwrap().contains(r#""hz":0"#));
    }

    #[test]
    fn dht_start_signal_is_answered_through_the_board() {
        let mut b = VibeBoard::from_config(CONFIG, HZ).unwrap();
        b.gpio_output_at(0, &[(4, false)], 1 << 4, 0);
        b.gpio_output_at(1_100 * US, &[], 0, 0);
        let first = b.next_deadline().expect("the DHT answers");
        b.advance_to(first);
        assert_eq!(b.take_edges(), vec![BoardEdge { cycle: first, pin: 4, level: false }]);
    }

    #[test]
    fn an_hx711_answers_its_clock_through_the_board() {
        let cfg = r#"{"parts":[{"kind":"hx711","pins":{"dout":4,"sck":5},"capacity":5000,"sensitivity":1.0,"offset":-1000,"weight":500,"noise":0},
            {"kind":"hx711","pins":{"dout":6,"sck":6}}]}"#;
        let mut b = VibeBoard::from_config(cfg, HZ).unwrap();
        assert_eq!(b.parts.len(), 1, "DOUT and PD_SCK on one pin is not a wiring");
        assert_eq!(b.input_levels(), vec![(4, true)]);
        assert!(b.ui_input("0", "capacity", 5000.0));
        assert!(!b.ui_input("0", "capacity", -1.0));
        assert!(!b.ui_input("0", "noise", -1.0));
        let ready = b.next_deadline().expect("the first conversion");
        assert_eq!(ready, 400 * HZ / 1000);
        b.advance_to(ready);
        assert_eq!(b.take_edges(), vec![BoardEdge { cycle: ready, pin: 4, level: false }]);
        // 213 748 = 0x0342f4: the first six bits are 0, the seventh 1; DOUT only moves on a change
        let mut bits = Vec::new();
        for n in 0..7u64 {
            let t = ready + n * 2 * US;
            b.gpio_output_at(t, &[(5, true)], 1 << 5, 1 << 5);
            bits.push(b.take_edges().last().map(|e| e.level));
            b.gpio_output_at(t + US, &[(5, false)], 1 << 5, 0);
        }
        assert_eq!(bits, vec![None, None, None, None, None, None, Some(true)]);
        let s = b.ui_state(&[]).unwrap();
        assert!(s.contains(r#""kind":"hx711","pin":4,"sck":5,"weight":500.0,"capacity":5000,"sensitivity":1.000,"offset":-1000,"gain":128,"raw":213748"#), "{s}");
    }

    #[test]
    fn bad_json_is_an_error() {
        assert!(VibeBoard::from_config("{parts", HZ).is_err());
    }

    const TFT: &str = r#"{"parts":[
        {"kind":"st7789","label":"TFT","pins":{"sck":12,"mosi":11,"cs":10,"dc":9,"rst":8},"width":240,"height":320},
        {"kind":"ws2812","label":"Strip","pins":{"data":5},"count":3},
        {"kind":"st7789","label":"no pins","pins":{"sck":12}}
    ]}"#;

    fn routes() -> SpiPins { SpiPins { sclk: 1 << 12, mosi: 1 << 11, miso: None, cs: 0 } }

    /// One command and its parameters, with CS and D/C driven as a GPIO-CS driver does.
    fn cmd(b: &mut VibeBoard, c: u8, params: &[u8]) {
        b.gpio_output_at(0, &[(10, false), (9, false)], 0, 0);
        b.spi_transfer_pins(2, routes(), &[c], 0);
        if !params.is_empty() {
            b.gpio_output_at(0, &[(9, true)], 0, 0);
            b.spi_transfer_pins(2, routes(), params, 0);
        }
        b.gpio_output_at(0, &[(10, true)], 0, 0);
    }

    #[test]
    fn an_st7789_draws_on_its_glass_as_the_adafruit_driver_leaves_it() {
        let mut b = VibeBoard::from_config(TFT, HZ).unwrap();
        assert_eq!(b.parts.len(), 2, "the TFT without its SPI pins is skipped");
        assert!(b.uses_spi_pins());
        assert!(b.display_quiet_push());
        // Adafruit_ST7789: SWRESET, SLPOUT, COLMOD 16-bit, rotation 0 = MADCTL MX|MY|RGB, INVON, DISPON
        for (c, p) in [(0x01, &[][..]), (0x11, &[]), (0x3a, &[0x55]), (0x36, &[0xc0]), (0x21, &[]), (0x29, &[])] { cmd(&mut b, c, p); }
        // a red pixel at (2, 1), and another write while CS is high that must not land; the glass
        // scans RAM mirrored both ways, so the pixel the driver mirrored comes out where it was drawn
        cmd(&mut b, 0x2a, &[0, 2, 0, 2]);
        cmd(&mut b, 0x2b, &[0, 1, 0, 1]);
        cmd(&mut b, 0x2c, &[0xf8, 0x00]);
        b.spi_transfer_pins(2, routes(), &[0x2c, 0x07, 0xe0], 0);
        let (w, h, px, _) = b.display().unwrap();
        assert_eq!((w, h), (240, 320));
        assert_eq!(px[240 + 2], 0xf800, "INVON on an IPS glass shows the RAM as written");
        assert_eq!(px.iter().filter(|&&p| p != 0).count(), 1);
        let s = b.ui_state(&[]).unwrap();
        assert!(s.contains(r#""kind":"st7789","on":true,"width":240,"height":320"#), "{s}");
        // without INVON the IPS glass shows inverted RAM
        cmd(&mut b, 0x20, &[]);
        assert_eq!(b.display().unwrap().2[240 + 2], !0xf800);
    }

    #[test]
    fn an_ili9341_glass_is_bgr() {
        let cfg = r#"{"parts":[{"kind":"ili9341","pins":{"sck":12,"mosi":11,"cs":10,"dc":9}}]}"#;
        let mut b = VibeBoard::from_config(cfg, HZ).unwrap();
        for (c, p) in [(0x11, &[][..]), (0x3a, &[0x55]), (0x36, &[0x48]), (0x29, &[])] { cmd(&mut b, c, p); }
        cmd(&mut b, 0x2a, &[0, 0, 0, 0]);
        cmd(&mut b, 0x2b, &[0, 0, 0, 0]);
        cmd(&mut b, 0x2c, &[0xf8, 0x00]);
        // MADCTL 0x48 (MX | BGR) is how Adafruit_ILI9341 drives its BGR glass, mirrored in x:
        // the pixel drawn at (0, 0) shows at (0, 0), and red stays red
        assert_eq!(b.display().unwrap().2[0], 0xf800);
    }

    #[test]
    fn i2c_sensors_attach_at_their_addresses_and_take_page_readings() {
        let cfg = r#"{"parts":[
            {"kind":"bme280","pins":{"sda":8,"scl":9},"addr":118},
            {"kind":"sht31","pins":{"sda":8,"scl":9}},
            {"kind":"aht20","pins":{"sda":8,"scl":9}},
            {"kind":"bh1750","pins":{"sda":8,"scl":9}},
            {"kind":"mpu6050","pins":{"sda":8,"scl":9}}
        ]}"#;
        let mut b = VibeBoard::from_config(cfg, HZ).unwrap();
        let addrs: Vec<u8> = b.i2c_devices().iter().map(|d| d.1).collect();
        assert_eq!(addrs, vec![0x76, 0x44, 0x38, 0x23, 0x68], "a configured address wins over the driver default");
        assert!(b.ui_input("0", "pressure", 990.5));
        assert!(b.ui_input("3", "lux", 1234.0));
        assert!(b.ui_input("4", "az", -1.0));
        assert!(!b.ui_input("4", "colour", 1.0));
        let s = b.ui_state(&[]).unwrap();
        assert!(s.contains(r#""kind":"bme280","addr":118,"temperature":24.00,"humidity":50.0,"pressure":990.50"#), "{s}");
        assert!(s.contains(r#""kind":"bh1750","addr":35,"lux":1234.0"#), "{s}");
        assert!(s.contains(r#""az":-1.000"#), "{s}");
    }

    #[test]
    fn a_strip_takes_rmt_frames_and_reports_its_colours() {
        let mut b = VibeBoard::from_config(TFT, HZ).unwrap();
        let grb = |g: u8, r: u8, bl: u8| [g, r, bl].into_iter().flat_map(|v| (0..8).map(move |i| v & (0x80 >> i) != 0)).collect::<Vec<bool>>();
        let frame: Vec<bool> = [grb(0, 255, 0), grb(255, 0, 0), grb(0, 0, 255)].concat();
        b.rmt_frame(5, &frame);
        b.rmt_frame(6, &grb(1, 1, 1)); // another pin: not this strip
        let s = b.ui_state(&[]).unwrap();
        assert!(s.contains(r#""kind":"ws2812","pin":5,"updates":1,"leds":[[255,0,0],[0,255,0],[0,0,255]]"#), "{s}");
    }
}
