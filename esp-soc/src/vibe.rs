//! A board assembled at run time from a part list, for vibe-iot's simulator. The ESP32-S3 builds
//! it for `vibe:<json>`, the classic ESP32 for `vibe-esp32:<json>`, the JSON being vibe-iot's SimConfig:
//! `{"parts":[{"kind":"ssd1306","pins":{"sda":8,"scl":9},"addr":60}, {"kind":"dht22","pins":{"data":4}},
//! {"kind":"led","pins":{"signal":2}}, …]}`. Parts are numbered by their place in the list; that
//! number is the `id` the page's controls use.
//!
//! - `ssd1306`: the OLED controller on I2C0 at `addr` (0x3C by default). The first one is the
//!   board's display (`{"t":…}` binary type 1).
//! - `dht22` / `dht11`: the sensor on `data`; the page sets `temperature` and `humidity`.
//! - `led`, `buzzer`: outputs on `signal`, reported with their level and any PWM on the pin.
//!   A buzzer toggled by software reports the frequency of its edges.
//! - `button`: an active-low input on `signal`, idle high; the page drives it with `btn`.
//! - `analog`: a voltage on `signal`'s ADC pad; the page sets it with `adc`.
use std::sync::{Arc, Mutex};

use esp_periph::i2c::I2cDevice;

use crate::board::{BoardEdge, BoardModel, VirtualCycle};
use crate::devices::{Dht, DhtKind, Ssd1306, Ssd1306I2c};
use crate::json::{parse_json, Json};

enum Part {
    Oled { addr: u8, state: Arc<Mutex<Ssd1306>> },
    Dht(Dht),
    Led { pin: u8 },
    Buzzer { pin: u8, rises: [VirtualCycle; 2] },
    Button { pin: u8 },
    Analog { pin: u8 },
}

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
                "dht22" | "dht11" => pin(p, "data").map(|pin| {
                    let mut d = Dht::new(pin, if kind == "dht11" { DhtKind::Dht11 } else { DhtKind::Dht22 }, us);
                    if let Some(t) = p.get("temperature").and_then(Json::as_f64) { d.temperature = t; }
                    if let Some(h) = p.get("humidity").and_then(Json::as_f64) { d.humidity = h; }
                    Part::Dht(d)
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

    fn oled(&self) -> Option<&Arc<Mutex<Ssd1306>>> {
        self.parts.iter().find_map(|(_, p)| match p { Part::Oled { state, .. } => Some(state), _ => None })
    }

    fn dhts(&mut self) -> impl Iterator<Item = &mut Dht> {
        self.parts.iter_mut().filter_map(|(_, p)| match p { Part::Dht(d) => Some(d), _ => None })
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
    }

    fn gpio_events(&self) -> u64 { self.gpio_events }

    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        self.parts.iter().filter_map(|(_, p)| match p {
            Part::Oled { addr, state } => Some((0u8, *addr, Box::new(Ssd1306I2c::new(state.clone())) as Box<dyn I2cDevice>)),
            _ => None,
        }).collect()
    }

    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let s = self.oled()?.lock().expect("SSD1306 mutex poisoned");
        Some((crate::devices::ssd1306::SSD1306_WIDTH as u32, s.height() as u32, s.frame(), s.version))
    }
    fn display_version(&self) -> u64 { self.oled().map_or(0, |s| s.lock().expect("SSD1306 mutex poisoned").version) }
    fn display_frames(&self) -> u64 { self.display_version() }

    /// Buttons idle high (pressed pulls them low); the DHT line idles high on its pull-up.
    fn input_levels(&self) -> Vec<(u8, bool)> {
        self.parts.iter().filter_map(|(_, p)| match p {
            Part::Button { pin } => Some((*pin, true)),
            Part::Dht(d) => Some((d.pin, true)),
            _ => None,
        }).collect()
    }

    fn next_deadline(&self) -> Option<VirtualCycle> {
        self.parts.iter().filter_map(|(_, p)| match p { Part::Dht(d) => d.next_deadline(), _ => None }).min()
    }

    fn advance_to(&mut self, cycle: VirtualCycle) {
        self.now = self.now.max(cycle);
        for d in self.dhts() { d.advance_to(cycle); }
    }

    fn take_edges(&mut self) -> Vec<BoardEdge> {
        let mut edges: Vec<BoardEdge> = self.dhts().flat_map(|d| d.take_edges()).collect();
        edges.sort_by_key(|e| e.cycle);
        edges
    }

    fn released_inputs(&mut self) -> Vec<u8> { self.dhts().flat_map(|d| d.take_released()).collect() }

    fn named_pin(&self, name: &str) -> Option<u8> {
        let i: usize = name.parse().ok()?;
        self.parts.iter().find(|(id, _)| id.parse() == Ok(i)).and_then(|(_, p)| match p {
            Part::Button { pin } | Part::Led { pin } | Part::Buzzer { pin, .. } | Part::Analog { pin } => Some(*pin),
            Part::Dht(d) => Some(d.pin),
            Part::Oled { .. } => None,
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
                Part::Dht(d) => format!(
                    "{{\"id\":\"{id}\",\"kind\":\"{}\",\"pin\":{},\"temperature\":{:.1},\"humidity\":{:.1},\"reads\":{}}}",
                    if d.kind == DhtKind::Dht11 { "dht11" } else { "dht22" }, d.pin, d.temperature, d.humidity, d.reads),
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
            if let Part::Dht(d) = p {
                match field {
                    "temperature" => { d.temperature = value.clamp(-40.0, 80.0); return true; }
                    "humidity" => { d.humidity = value.clamp(0.0, 100.0); return true; }
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
    fn bad_json_is_an_error() {
        assert!(VibeBoard::from_config("{parts", HZ).is_err());
    }
}
