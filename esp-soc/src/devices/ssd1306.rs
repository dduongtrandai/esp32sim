//! SSD1306 monochrome OLED controller on I2C (128x64 and 128x32 modules).
//!
//! Every I2C write starts with a control byte: bit 6 (D/C) says whether the bytes that follow are
//! commands or display data, bit 7 (Co) that only one byte follows before the next control byte.
//! Data lands in a 128x8-page GDDRAM, one byte per column of 8 vertical pixels, at a pointer that
//! advances by the addressing mode (horizontal, vertical or page). This models the commands the
//! common drivers send (Adafruit_SSD1306, U8g2, ThingPulse): addressing, remap, scan direction,
//! multiplex, start line, display on/off, inversion and entire-display-on. Scrolling and the charge
//! pump, clock and precharge settings are accepted and ignored.
use esp_periph::i2c::I2cDevice;
use std::sync::{Arc, Mutex};

pub const SSD1306_WIDTH: usize = 128;
const PAGES: usize = 8;

/// Lit pixel colour in the RGB565 frame: the white of the common modules.
pub const SSD1306_ON: u16 = 0xffff;

#[derive(Clone, Debug)]
pub struct Ssd1306 {
    ram: Vec<u8>,
    /// Multiplex ratio + 1: 64 or 32 rows, as the driver configures it.
    height: usize,
    mode: u8,
    col_start: u8,
    col_end: u8,
    page_start: u8,
    page_end: u8,
    col: u8,
    page: u8,
    on: bool,
    invert: bool,
    entire_on: bool,
    seg_remap: bool,
    com_rev: bool,
    start_line: u8,
    /// Changes whenever what the glass shows may have changed.
    pub version: u64,
    /// Data bytes written, for tests and reports.
    pub data_bytes: u64,
    cmd: Option<u8>,
    args: Vec<u8>,
}

impl Ssd1306 {
    pub fn new(height: usize) -> Self {
        Self {
            ram: vec![0; SSD1306_WIDTH * PAGES],
            height: if height == 32 { 32 } else { 64 },
            mode: 2, // the controller resets into page addressing
            col_start: 0,
            col_end: 127,
            page_start: 0,
            page_end: 7,
            col: 0,
            page: 0,
            on: false,
            invert: false,
            entire_on: false,
            seg_remap: false,
            com_rev: false,
            start_line: 0,
            version: 1,
            data_bytes: 0,
            cmd: None,
            args: Vec::new(),
        }
    }

    pub fn height(&self) -> usize { self.height }
    pub fn is_on(&self) -> bool { self.on }

    /// Argument bytes that follow a command byte.
    fn arg_count(cmd: u8) -> usize {
        match cmd {
            0x20 | 0x81 | 0x8d | 0xa8 | 0xad | 0xd3 | 0xd5 | 0xd6 | 0xd9 | 0xda | 0xdb => 1,
            0x21 | 0x22 | 0xa3 => 2,
            0x29 | 0x2a => 5,
            0x26 | 0x27 => 6,
            _ => 0,
        }
    }

    pub fn command(&mut self, b: u8) {
        if let Some(cmd) = self.cmd {
            self.args.push(b);
            if self.args.len() >= Self::arg_count(cmd) {
                self.cmd = None;
                let args = std::mem::take(&mut self.args);
                self.apply(cmd, &args);
            }
            return;
        }
        if Self::arg_count(b) > 0 {
            self.cmd = Some(b);
            self.args.clear();
        } else {
            self.apply(b, &[]);
        }
    }

    fn apply(&mut self, cmd: u8, a: &[u8]) {
        match cmd {
            0x00..=0x0f => self.col = (self.col & 0xf0) | (cmd & 0x0f),
            0x10..=0x1f => self.col = ((self.col & 0x0f) | ((cmd & 0x0f) << 4)) & 0x7f,
            0x20 => self.mode = a[0] & 3,
            0x21 => {
                self.col_start = a[0] & 0x7f;
                self.col_end = a[1] & 0x7f;
                self.col = self.col_start;
            }
            0x22 => {
                self.page_start = a[0] & 7;
                self.page_end = a[1] & 7;
                self.page = self.page_start;
            }
            0x40..=0x7f => { self.start_line = cmd & 0x3f; self.version += 1; }
            0xa0 | 0xa1 => { self.seg_remap = cmd == 0xa1; self.version += 1; }
            0xa4 | 0xa5 => { self.entire_on = cmd == 0xa5; self.version += 1; }
            0xa6 | 0xa7 => { self.invert = cmd == 0xa7; self.version += 1; }
            0xa8 => { self.height = if (a[0] & 0x3f) < 32 { 32 } else { 64 }; self.version += 1; }
            0xae | 0xaf => { self.on = cmd == 0xaf; self.version += 1; }
            0xb0..=0xb7 => self.page = cmd & 7,
            0xc0 | 0xc8 => { self.com_rev = cmd == 0xc8; self.version += 1; }
            _ => {}
        }
    }

    pub fn data(&mut self, b: u8) {
        let at = usize::from(self.page) * SSD1306_WIDTH + usize::from(self.col);
        self.ram[at] = b;
        self.data_bytes += 1;
        self.version += 1;
        match self.mode {
            0 => {
                if self.col >= self.col_end {
                    self.col = self.col_start;
                    self.page = if self.page >= self.page_end { self.page_start } else { self.page + 1 };
                } else {
                    self.col += 1;
                }
            }
            1 => {
                if self.page >= self.page_end {
                    self.page = self.page_start;
                    self.col = if self.col >= self.col_end { self.col_start } else { self.col + 1 };
                } else {
                    self.page += 1;
                }
            }
            _ => self.col = (self.col + 1) & 0x7f,
        }
    }

    /// Whether the pixel at screen (x, y) is lit, with the glass mounted the way the common
    /// modules are: A1 (segment remap) with C8 (reversed COM scan) shows GDDRAM upright.
    pub fn pixel(&self, x: usize, y: usize) -> bool {
        if !self.on { return false; }
        if self.entire_on { return true; }
        let col = if self.seg_remap { x } else { SSD1306_WIDTH - 1 - x };
        let row = if self.com_rev { y } else { self.height - 1 - y };
        let row = (row + usize::from(self.start_line)) % (PAGES * 8);
        let bit = self.ram[(row / 8) * SSD1306_WIDTH + col] >> (row % 8) & 1 != 0;
        bit != self.invert
    }

    /// The glass as RGB565, `SSD1306_WIDTH` by `height`.
    pub fn frame(&self) -> Vec<u16> {
        let mut out = Vec::with_capacity(SSD1306_WIDTH * self.height);
        for y in 0..self.height {
            for x in 0..SSD1306_WIDTH {
                out.push(if self.pixel(x, y) { SSD1306_ON } else { 0 });
            }
        }
        out
    }
}

/// The controller as an I2C target. The board keeps the other handle to draw the glass.
pub struct Ssd1306I2c {
    state: Arc<Mutex<Ssd1306>>,
    expect_control: bool,
    data: bool,
    single: bool,
}

impl Ssd1306I2c {
    pub fn new(state: Arc<Mutex<Ssd1306>>) -> Self {
        Self { state, expect_control: true, data: false, single: false }
    }
}

impl I2cDevice for Ssd1306I2c {
    fn start(&mut self, _read: bool) -> bool {
        self.expect_control = true;
        true
    }
    fn write(&mut self, b: u8) -> bool {
        if self.expect_control {
            self.single = b & 0x80 != 0;
            self.data = b & 0x40 != 0;
            self.expect_control = false;
            return true;
        }
        let mut s = self.state.lock().expect("SSD1306 state mutex poisoned");
        if self.data { s.data(b) } else { s.command(b) }
        if self.single { self.expect_control = true; }
        true
    }
    /// Status read: bit 6 is "display off".
    fn read(&mut self) -> u8 {
        if self.state.lock().expect("SSD1306 state mutex poisoned").on { 0 } else { 0x40 }
    }
    fn stop(&mut self) { self.expect_control = true; }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bus(height: usize) -> (Arc<Mutex<Ssd1306>>, Ssd1306I2c) {
        let s = Arc::new(Mutex::new(Ssd1306::new(height)));
        (s.clone(), Ssd1306I2c::new(s))
    }

    fn send(dev: &mut Ssd1306I2c, bytes: &[u8]) {
        dev.start(false);
        for &b in bytes { assert!(dev.write(b)); }
        dev.stop();
    }

    /// The init sequence Adafruit_SSD1306::begin sends for a 128x64 panel.
    const ADAFRUIT_INIT_64: &[u8] = &[
        0x00, 0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0x00, 0x40, 0x8d, 0x14, 0x20, 0x00, 0xa1, 0xc8,
        0xda, 0x12, 0x81, 0xcf, 0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6, 0x2e, 0xaf,
    ];

    #[test]
    fn adafruit_init_then_full_frame_lands_upright() {
        let (s, mut dev) = bus(64);
        send(&mut dev, ADAFRUIT_INIT_64);
        // display(): window over the whole RAM, then 1024 data bytes in 32-byte transactions
        send(&mut dev, &[0x00, 0x22, 0x00, 0xff, 0x21, 0x00, 0x7f]);
        let mut fb = vec![0u8; 1024];
        fb[0] = 0x01; // top-left pixel
        fb[1023] = 0x80; // bottom-right pixel
        for chunk in fb.chunks(31) {
            let mut tx = vec![0x40];
            tx.extend_from_slice(chunk);
            send(&mut dev, &tx);
        }
        let s = s.lock().unwrap();
        assert!(s.is_on());
        assert_eq!(s.height(), 64);
        assert!(s.pixel(0, 0), "RAM column 0, page 0, bit 0 is the top-left pixel");
        assert!(s.pixel(127, 63), "the last byte's bit 7 is the bottom-right pixel");
        assert!(!s.pixel(1, 0));
        assert_eq!(s.frame().iter().filter(|&&p| p == SSD1306_ON).count(), 2);
    }

    #[test]
    fn page_mode_and_single_byte_control() {
        let (s, mut dev) = bus(64);
        // Co=1 control bytes: one command each (U8g2 style)
        send(&mut dev, &[0x80, 0xa1, 0x80, 0xc8, 0x80, 0xaf, 0x80, 0xb2, 0x80, 0x05, 0x80, 0x10]);
        send(&mut dev, &[0x40, 0xff]);
        let s = s.lock().unwrap();
        for y in 16..24 { assert!(s.pixel(5, y), "page 2, column 5 fills rows 16-23 (row {y})"); }
        assert!(!s.pixel(6, 16));
    }

    #[test]
    fn mux_32_inversion_and_display_off() {
        let (s, mut dev) = bus(64);
        send(&mut dev, &[0x00, 0xa8, 0x1f, 0xa1, 0xc8, 0xa7, 0xaf]);
        {
            let s = s.lock().unwrap();
            assert_eq!(s.height(), 32);
            assert!(s.pixel(0, 0), "inverted: an empty RAM shows lit");
            assert_eq!(s.frame().len(), 128 * 32);
        }
        send(&mut dev, &[0x00, 0xae]);
        assert!(!s.lock().unwrap().pixel(0, 0), "off: nothing lit");
    }

    #[test]
    fn unremapped_glass_is_rotated() {
        let (s, mut dev) = bus(64);
        send(&mut dev, &[0x00, 0x20, 0x00, 0xaf]);
        send(&mut dev, &[0x40, 0x01]);
        assert!(s.lock().unwrap().pixel(127, 63), "without A1/C8 the module shows RAM rotated 180 degrees");
    }
}
