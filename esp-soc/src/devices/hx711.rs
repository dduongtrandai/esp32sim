//! HX711 24-bit load-cell ADC on its two-wire interface, DOUT and PD_SCK, with a load cell on
//! channel A (the usual breakout: E+/E-/A+/A- to the bridge).
//!
//! From the Avia datasheet: DOUT is high while a conversion runs and goes low when a word is
//! ready. Each PD_SCK rising edge then shifts out one bit, MSB first, within 0.1 us; after the 24
//! data bits, 1 to 3 more pulses pull DOUT high again and choose the next conversion: 25 pulses
//! channel A gain 128, 26 channel B gain 32, 27 channel A gain 64. Conversions run continuously,
//! 10 per second with RATE tied low (most breakouts) or 80 with it high. PD_SCK held high for
//! more than 60 us powers the chip down; taking it low again resets it to channel A gain 128, and
//! the first word comes after the settling time (400 ms at 10 SPS, 50 ms at 80 SPS).
//!
//! The bridge is ratiometric, so the word does not depend on the supply: the differential input is
//! `weight / capacity * sensitivity` (mV/V) of the excitation, and the full scale is +-0.5 of the
//! supply over the gain, which gives `code = 2 * gain * 2^23 * weight / capacity * sensitivity / 1000`.
//! `offset` is the bridge's zero error in counts at gain 128. There is no noise: a reading is
//! repeatable, so the firmware's averaging and tare logic see exactly what the page set.
use crate::board::{BoardEdge, VirtualCycle};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Phase {
    /// DOUT high: no word since power-up, reset or the last read.
    Converting,
    /// DOUT low: a word is waiting.
    Ready,
    /// This many data bits shifted out, 1..=24.
    Shifting(u8),
    /// Pulses after the data, 25..=27: DOUT is high and they pick the next conversion.
    Trailing(u8),
    /// PD_SCK was held high past 60 us.
    PoweredDown,
}

#[derive(Clone, Debug)]
pub struct Hx711 {
    pub dout: u8,
    pub sck: u8,
    /// Grams on the load cell.
    pub weight: f64,
    /// The load cell's rated capacity, grams.
    pub capacity: f64,
    /// The load cell's rated output, mV/V.
    pub sensitivity: f64,
    /// Counts at no load, at gain 128.
    pub offset: f64,
    /// Words read out (24 bits clocked), for the page and the tests.
    pub reads: u64,
    /// The word most recently latched, sign-extended.
    pub last: i32,
    cycles_per_us: u64,
    period: u64,
    settle: u64,
    phase: Phase,
    /// Pulses that chose the conversion now running (25, 26 or 27).
    mode: u8,
    word: u32,
    dout_level: bool,
    sck_high: bool,
    sck_rose: VirtualCycle,
    next_ready: Option<VirtualCycle>,
    out: Vec<BoardEdge>,
}

impl Hx711 {
    /// `rate`: conversions per second, 10 or 80.
    pub fn new(dout: u8, sck: u8, rate: u32, cycles_per_us: u64) -> Self {
        let fast = rate >= 80;
        let ms = cycles_per_us * 1000;
        let settle = if fast { 50 * ms } else { 400 * ms };
        Self {
            dout, sck, weight: 0.0, capacity: 5000.0, sensitivity: 1.0, offset: 0.0, reads: 0, last: 0, cycles_per_us,
            period: if fast { 12_500 * cycles_per_us } else { 100 * ms }, settle,
            phase: Phase::Converting, mode: 25, word: 0, dout_level: true, sck_high: false, sck_rose: 0,
            next_ready: Some(settle), out: Vec::new(),
        }
    }

    /// Channel A at gain 128 or 64, or channel B (nothing wired: reads 0).
    pub fn gain(&self) -> u32 { match self.mode { 26 => 32, 27 => 64, _ => 128 } }

    /// The word a conversion in the current mode makes of the page's values.
    pub fn code(&self) -> i32 {
        if self.mode == 26 { return 0; }
        let gain = self.gain() as f64;
        let cap = if self.capacity > 0.0 { self.capacity } else { 1.0 };
        let fraction = self.weight / cap * self.sensitivity / 1000.0;
        let code = 2.0 * gain * 8_388_608.0 * fraction + self.offset * gain / 128.0;
        code.round().clamp(-8_388_608.0, 8_388_607.0) as i32
    }

    pub fn dout_level(&self) -> bool { self.dout_level }
    pub fn ready(&self) -> bool { self.phase == Phase::Ready }

    fn set_dout(&mut self, cycle: VirtualCycle, level: bool) {
        if level != self.dout_level {
            self.dout_level = level;
            self.out.push(BoardEdge { cycle, pin: self.dout, level });
        }
    }

    fn power_down_at(&self) -> Option<VirtualCycle> {
        (self.sck_high && self.phase != Phase::PoweredDown).then(|| self.sck_rose + 60 * self.cycles_per_us)
    }

    /// What the host does with PD_SCK, from the SoC's output and output-enable masks.
    pub fn host_drive(&mut self, cycle: VirtualCycle, enabled: u64, output: u64) {
        let high = enabled >> self.sck & 1 == 1 && output >> self.sck & 1 == 1;
        if high == self.sck_high { return; }
        self.advance_to(cycle);
        self.sck_high = high;
        if high {
            self.sck_rose = cycle;
            self.pulse(cycle);
        } else if self.phase == Phase::PoweredDown {
            // power up: reset to channel A gain 128, first word after settling
            self.phase = Phase::Converting;
            self.mode = 25;
            self.next_ready = Some(cycle + self.settle);
        }
    }

    /// A PD_SCK rising edge: the next bit appears on DOUT at once (datasheet: within 0.1 us).
    fn pulse(&mut self, cycle: VirtualCycle) {
        match self.phase {
            Phase::Ready => self.shift(cycle, 1),
            Phase::Shifting(n) if n < 24 => self.shift(cycle, n + 1),
            Phase::Shifting(_) => {
                self.phase = Phase::Trailing(25);
                self.set_dout(cycle, true);
            }
            Phase::Trailing(n) => self.phase = Phase::Trailing((n + 1).min(27)),
            Phase::Converting | Phase::PoweredDown => {}
        }
    }

    fn shift(&mut self, cycle: VirtualCycle, n: u8) {
        self.phase = Phase::Shifting(n);
        if n == 24 { self.reads += 1; }
        self.set_dout(cycle, self.word >> (24 - n) & 1 == 1);
    }

    fn convert(&mut self, at: VirtualCycle) {
        if let Phase::Trailing(n) = self.phase { self.mode = n; }
        if let Phase::Shifting(_) = self.phase { return; } // the host is reading the last word: it keeps it
        self.last = self.code();
        self.word = self.last as u32 & 0xff_ffff;
        self.phase = Phase::Ready;
        self.set_dout(at, false);
    }

    pub fn next_deadline(&self) -> Option<VirtualCycle> {
        match (self.next_ready, self.power_down_at()) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    pub fn advance_to(&mut self, cycle: VirtualCycle) {
        loop {
            let down = self.power_down_at().filter(|&t| t <= cycle);
            let ready = self.next_ready.filter(|&t| t <= cycle);
            match (down, ready) {
                (Some(d), r) if r.is_none_or(|r| d <= r) => {
                    self.phase = Phase::PoweredDown;
                    self.next_ready = None;
                    self.set_dout(d, true);
                }
                (_, Some(r)) => {
                    self.convert(r);
                    self.next_ready = Some(r + self.period);
                }
                _ => break,
            }
        }
    }

    pub fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.out) }
}

#[cfg(test)]
mod tests {
    use super::*;

    const US: u64 = 240;
    const MS: u64 = 1000 * US;
    const DOUT: u8 = 4;
    const SCK: u8 = 5;

    fn sck(h: &mut Hx711, at: u64, high: bool) { h.host_drive(at, 1 << SCK, if high { 1 << SCK } else { 0 }); }

    /// What bogde/HX711's read() does: wait for DOUT low, 24 pulses sampling DOUT while PD_SCK is
    /// high, then `extra` pulses for the next gain. Returns the word, sign-extended.
    fn read(h: &mut Hx711, mut t: u64, extra: u8) -> (i32, u64) {
        h.advance_to(t);
        assert!(!h.dout_level(), "DOUT is low when a word is ready");
        let mut v = 0u32;
        for _ in 0..24 {
            sck(h, t, true);
            t += US;
            v = v << 1 | h.dout_level() as u32;
            sck(h, t, false);
            t += US;
        }
        for _ in 0..extra {
            sck(h, t, true);
            t += US;
            sck(h, t, false);
            t += US;
        }
        (((v << 8) as i32) >> 8, t)
    }

    #[test]
    fn the_first_word_comes_after_settling() {
        let mut h = Hx711::new(DOUT, SCK, 10, US);
        assert!(h.dout_level());
        assert_eq!(h.next_deadline(), Some(400 * MS));
        h.advance_to(400 * MS - 1);
        assert!(h.take_edges().is_empty());
        h.advance_to(400 * MS);
        assert_eq!(h.take_edges(), vec![BoardEdge { cycle: 400 * MS, pin: DOUT, level: false }]);
        assert!(h.ready());
    }

    #[test]
    fn a_read_returns_the_weight_in_counts() {
        let mut h = Hx711::new(DOUT, SCK, 10, US);
        h.weight = 500.0;
        h.offset = -12_345.0;
        // 500 g of 5 kg at 1 mV/V, gain 128: 2 * 128 * 2^23 * 0.1 * 0.001 = 214 748
        assert_eq!(h.code(), 214_748 - 12_345);
        let (v, t) = read(&mut h, 400 * MS, 1);
        assert_eq!(v, 214_748 - 12_345);
        assert_eq!(h.reads, 1);
        assert!(h.dout_level(), "the 25th pulse pulls DOUT high");
        assert!(!h.ready());
        // the next word comes with the next conversion, 100 ms after the last
        h.weight = 0.0;
        h.advance_to(t);
        assert!(h.dout_level());
        let (v, _) = read(&mut h, 500 * MS, 1);
        assert_eq!(v, -12_345);
    }

    #[test]
    fn negative_words_and_saturation() {
        let mut h = Hx711::new(DOUT, SCK, 10, US);
        h.weight = -100.0;
        let (v, _) = read(&mut h, 400 * MS, 1);
        assert_eq!(v, -42_950);
        h.weight = 1e9;
        assert_eq!(h.code(), 8_388_607);
    }

    #[test]
    fn extra_pulses_choose_the_next_gain_and_channel() {
        let mut h = Hx711::new(DOUT, SCK, 10, US);
        h.weight = 500.0;
        read(&mut h, 400 * MS, 3); // 27 pulses: channel A, gain 64
        let (v, _) = read(&mut h, 500 * MS, 2); // 26: channel B
        assert_eq!(v, 107_374);
        let (v, _) = read(&mut h, 600 * MS, 1);
        assert_eq!(v, 0, "channel B has nothing wired");
        h.advance_to(700 * MS);
        assert_eq!(h.gain(), 128);
    }

    #[test]
    fn eighty_samples_per_second() {
        let mut h = Hx711::new(DOUT, SCK, 80, US);
        assert_eq!(h.next_deadline(), Some(50 * MS));
        let (_, t) = read(&mut h, 50 * MS, 1);
        h.advance_to(t);
        assert_eq!(h.next_deadline(), Some(50 * MS + 12_500 * US));
    }

    #[test]
    fn holding_pd_sck_high_powers_down_and_low_resets() {
        let mut h = Hx711::new(DOUT, SCK, 10, US);
        read(&mut h, 400 * MS, 3); // gain 64 for the next conversion
        sck(&mut h, 450 * MS, true);
        assert_eq!(h.next_deadline(), Some(450 * MS + 60 * US), "power-down after 60 us high");
        h.advance_to(460 * MS);
        assert_eq!(h.next_deadline(), None, "no conversions while powered down");
        sck(&mut h, 900 * MS, false);
        assert_eq!(h.gain(), 128, "reset to channel A gain 128");
        assert_eq!(h.next_deadline(), Some(900 * MS + 400 * MS));
        h.weight = 100.0;
        let (v, _) = read(&mut h, 1300 * MS, 1);
        assert_eq!(v, 42_950);
    }

    #[test]
    fn a_word_being_read_is_not_replaced_by_the_next_conversion() {
        let mut h = Hx711::new(DOUT, SCK, 10, US);
        h.weight = 500.0;
        h.advance_to(400 * MS);
        // 12 bits, then the host stalls past the next conversion (but not 60 us with PD_SCK high)
        let mut t = 400 * MS;
        for _ in 0..12 { sck(&mut h, t, true); sck(&mut h, t + US, false); t += 2 * US; }
        h.weight = 0.0;
        h.advance_to(520 * MS);
        let mut v = 0u32;
        for _ in 0..12 { sck(&mut h, 520 * MS, true); v = v << 1 | h.dout_level() as u32; sck(&mut h, 520 * MS + US, false); }
        assert_eq!(v, 214_748 & 0xfff, "the low 12 bits are still the old word's");
    }
}
