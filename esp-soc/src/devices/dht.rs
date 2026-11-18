//! DHT22 (AM2302) and DHT11 temperature and humidity sensors on their single-wire bus.
//!
//! The host pulls the line low (at least 1 ms for the DHT22, 18 ms for the DHT11; anything from
//! 500 us is accepted here) and releases it. The sensor answers 20-40 us later: 80 us low, 80 us
//! high, then 40 bits, each a 50 us low followed by a high that lasts 26-28 us for a 0 and 70 us
//! for a 1, and a final 50 us low before it lets go. The bits are humidity, temperature and a
//! checksum, in the format of the chosen part. The answer is a list of timed edges the SoC
//! delivers at their cycle, so a driver that times pulses in a busy loop (Adafruit's DHT library)
//! reads the same widths as on hardware.
use crate::board::{BoardEdge, VirtualCycle};
use std::collections::VecDeque;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DhtKind { Dht22, Dht11 }

#[derive(Clone, Debug)]
pub struct Dht {
    pub pin: u8,
    pub kind: DhtKind,
    /// Degrees Celsius.
    pub temperature: f64,
    /// Relative humidity, percent.
    pub humidity: f64,
    /// Answers sent, for the page and the tests.
    pub reads: u64,
    cycles_per_us: u64,
    low_since: Option<VirtualCycle>,
    pending: VecDeque<BoardEdge>,
    release_at: Option<VirtualCycle>,
    out: Vec<BoardEdge>,
    released: Vec<u8>,
}

impl Dht {
    pub fn new(pin: u8, kind: DhtKind, cycles_per_us: u64) -> Self {
        Self {
            pin, kind, temperature: 24.0, humidity: 50.0, reads: 0, cycles_per_us,
            low_since: None, pending: VecDeque::new(), release_at: None, out: Vec::new(), released: Vec::new(),
        }
    }

    /// The five bytes of one answer, checksum last.
    pub fn frame(&self) -> [u8; 5] {
        let (h, t) = (self.humidity.clamp(0.0, 100.0), self.temperature);
        let mut b = match self.kind {
            DhtKind::Dht22 => {
                let h = (h * 10.0).round() as u16;
                let t = ((t.abs().min(80.0) * 10.0).round() as u16) | if t < 0.0 { 0x8000 } else { 0 };
                [(h >> 8) as u8, h as u8, (t >> 8) as u8, t as u8, 0]
            }
            DhtKind::Dht11 => {
                let t = t.clamp(-20.0, 60.0);
                let (ti, tf) = (t.abs().trunc() as u8, ((t.abs().fract() * 10.0).round() as u8).min(9));
                [h.trunc() as u8, 0, ti, tf | if t < 0.0 { 0x80 } else { 0 }, 0]
            }
        };
        b[4] = b[..4].iter().fold(0u8, |s, &x| s.wrapping_add(x));
        b
    }

    /// What the host does with the pin, from the SoC's output and output-enable masks.
    pub fn host_drive(&mut self, cycle: VirtualCycle, enabled: u64, output: u64) {
        let driven_low = enabled >> self.pin & 1 == 1 && output >> self.pin & 1 == 0;
        if driven_low {
            self.low_since.get_or_insert(cycle);
        } else if let Some(start) = self.low_since.take() {
            if cycle.saturating_sub(start) >= 500 * self.cycles_per_us && self.pending.is_empty() && self.release_at.is_none() {
                self.answer(cycle);
            }
        }
    }

    fn answer(&mut self, released: VirtualCycle) {
        let us = self.cycles_per_us;
        let mut t = released + 30 * us;
        let pin = self.pin;
        let edge = |at: VirtualCycle, level: bool, q: &mut VecDeque<BoardEdge>| q.push_back(BoardEdge { cycle: at, pin, level });
        edge(t, false, &mut self.pending);
        t += 80 * us;
        edge(t, true, &mut self.pending);
        t += 80 * us;
        for byte in self.frame() {
            for bit in (0..8).rev() {
                edge(t, false, &mut self.pending);
                t += 50 * us;
                edge(t, true, &mut self.pending);
                t += if byte >> bit & 1 == 1 { 70 } else { 27 } * us;
            }
        }
        edge(t, false, &mut self.pending);
        self.release_at = Some(t + 50 * us);
        self.reads += 1;
    }

    pub fn next_deadline(&self) -> Option<VirtualCycle> {
        match (self.pending.front().map(|e| e.cycle), self.release_at) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    pub fn advance_to(&mut self, cycle: VirtualCycle) {
        while self.pending.front().is_some_and(|e| e.cycle <= cycle) {
            self.out.push(self.pending.pop_front().expect("checked front"));
        }
        if self.pending.is_empty() && self.release_at.is_some_and(|r| r <= cycle) {
            self.release_at = None;
            self.released.push(self.pin);
        }
    }

    pub fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.out) }
    pub fn take_released(&mut self) -> Vec<u8> { std::mem::take(&mut self.released) }
}

#[cfg(test)]
mod tests {
    use super::*;

    const US: u64 = 240;

    /// Decode the edges back into bytes the way a driver does: a high longer than the low before it is a 1.
    fn decode(edges: &[BoardEdge]) -> Vec<u8> {
        let mut bits = Vec::new();
        // skip the 80/80 preamble: edges 0 (low) and 1 (high); bit i is low at 2+2i, high at 3+2i
        for i in 0..40 {
            let low = edges[3 + 2 * i].cycle - edges[2 + 2 * i].cycle;
            let high = edges[4 + 2 * i].cycle - edges[3 + 2 * i].cycle;
            bits.push(high > low);
        }
        bits.chunks(8).map(|b| b.iter().fold(0u8, |a, &x| a << 1 | x as u8)).collect()
    }

    fn start(d: &mut Dht, at: u64, low_us: u64) {
        let pin = 1u64 << d.pin;
        d.host_drive(at, pin, 0);
        d.host_drive(at + low_us * US, 0, 0);
    }

    #[test]
    fn dht22_answers_with_its_reading() {
        let mut d = Dht::new(4, DhtKind::Dht22, US);
        d.temperature = -3.4;
        d.humidity = 61.5;
        start(&mut d, 1_000, 1_100);
        let released = 1_000 + 1_100 * US;
        assert_eq!(d.next_deadline(), Some(released + 30 * US), "the sensor answers 30 us after the release");
        d.advance_to(u64::MAX - 1);
        let edges = d.take_edges();
        assert_eq!(edges.len(), 2 + 80 + 1);
        assert!(!edges[0].level);
        assert_eq!(edges[1].cycle - edges[0].cycle, 80 * US);
        assert_eq!(d.take_released(), vec![4]);
        let bytes = decode(&edges);
        assert_eq!(bytes, d.frame().to_vec());
        assert_eq!(u16::from(bytes[0]) << 8 | u16::from(bytes[1]), 615);
        assert_eq!(u16::from(bytes[2]) << 8 | u16::from(bytes[3]), 0x8000 | 34);
        assert_eq!(bytes[4], bytes[..4].iter().fold(0u8, |s, &x| s.wrapping_add(x)));
        assert_eq!(d.reads, 1);
    }

    #[test]
    fn dht11_format() {
        let mut d = Dht::new(5, DhtKind::Dht11, US);
        d.temperature = 23.6;
        d.humidity = 40.0;
        assert_eq!(d.frame(), [40, 0, 23, 6, 69]);
    }

    #[test]
    fn short_pulses_and_reads_in_progress_are_ignored() {
        let mut d = Dht::new(4, DhtKind::Dht22, US);
        start(&mut d, 0, 100);
        assert_eq!(d.next_deadline(), None, "100 us is not a start signal");
        start(&mut d, 10_000_000, 1_000);
        let first = d.next_deadline();
        start(&mut d, 10_300_000, 1_000);
        assert_eq!(d.next_deadline(), first, "a second start during an answer does not restart it");
        assert_eq!(d.reads, 1);
    }

    #[test]
    fn edges_arrive_at_their_cycle() {
        let mut d = Dht::new(4, DhtKind::Dht22, US);
        start(&mut d, 0, 1_000);
        let first = d.next_deadline().unwrap();
        d.advance_to(first - 1);
        assert!(d.take_edges().is_empty());
        d.advance_to(first);
        assert_eq!(d.take_edges(), vec![BoardEdge { cycle: first, pin: 4, level: false }]);
    }
}
