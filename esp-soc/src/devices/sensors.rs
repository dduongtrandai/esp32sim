//! I2C sensors whose readings the page sets: BME280 / BMP280, SHT3x, AHT20, BH1750 and MPU-6050.
//!
//! Each model answers the register reads or commands its common Arduino drivers use (Adafruit
//! BME280, BMP280, SHT31, AHTX0 and MPU6050, claws/BH1750) and encodes the values in
//! `SensorValues` the way the part does, so the driver's own conversion gives them back. The
//! values are shared with the board, which changes them from the page.
use esp_periph::i2c::I2cDevice;
use std::sync::{Arc, Mutex};

/// What the sensors measure. SI-ish units as the drivers report them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorValues {
    /// °C
    pub temperature: f64,
    /// %RH
    pub humidity: f64,
    /// hPa
    pub pressure: f64,
    /// lx
    pub lux: f64,
    /// g, x y z
    pub accel: [f64; 3],
    /// °/s, x y z
    pub gyro: [f64; 3],
}

impl Default for SensorValues {
    fn default() -> Self {
        Self { temperature: 24.0, humidity: 50.0, pressure: 1013.25, lux: 300.0, accel: [0.0, 0.0, 1.0], gyro: [0.0; 3] }
    }
}

impl SensorValues {
    /// Sets one field by its page name; false for a name this struct does not have.
    pub fn set(&mut self, field: &str, v: f64) -> bool {
        match field {
            "temperature" => self.temperature = v.clamp(-40.0, 85.0),
            "humidity" => self.humidity = v.clamp(0.0, 100.0),
            "pressure" => self.pressure = v.clamp(300.0, 1100.0),
            "lux" => self.lux = v.clamp(0.0, 65000.0),
            "ax" => self.accel[0] = v.clamp(-2.0, 2.0),
            "ay" => self.accel[1] = v.clamp(-2.0, 2.0),
            "az" => self.accel[2] = v.clamp(-2.0, 2.0),
            "gx" => self.gyro[0] = v.clamp(-250.0, 250.0),
            "gy" => self.gyro[1] = v.clamp(-250.0, 250.0),
            "gz" => self.gyro[2] = v.clamp(-250.0, 250.0),
            _ => return false,
        }
        true
    }
}

pub type Shared = Arc<Mutex<SensorValues>>;

fn values(s: &Shared) -> SensorValues { *s.lock().expect("sensor values mutex poisoned") }

/// The sensor kinds this module models and their default I2C addresses (the drivers' defaults).
pub fn default_address(kind: &str) -> Option<u8> {
    Some(match kind {
        "bme280" | "bmp280" => 0x77,
        "sht31" => 0x44,
        "aht20" => 0x38,
        "bh1750" => 0x23,
        "mpu6050" => 0x68,
        _ => return None,
    })
}

/// A new device of `kind` reading `values`.
pub fn make(kind: &str, values: Shared) -> Option<Box<dyn I2cDevice>> {
    Some(match kind {
        "bme280" => Box::new(Bme280::new(values, true)),
        "bmp280" => Box::new(Bme280::new(values, false)),
        "sht31" => Box::new(Sht3x::new(values)),
        "aht20" => Box::new(Aht20Device::new(values)),
        "bh1750" => Box::new(Bh1750::new(values)),
        "mpu6050" => Box::new(Mpu6050::new(values)),
        _ => return None,
    })
}

// ------------------------------------------------------------------ BME280 / BMP280

/// Bosch's trimming parameters, as a typical part carries them.
pub struct Bme280Calib {
    pub t: (u16, i16, i16),
    pub p: (u16, i16, i16, i16, i16, i16, i16, i16, i16),
    pub h: (u8, i16, u8, i16, i16, i8),
}

pub const BME280_CALIB: Bme280Calib = Bme280Calib {
    t: (27504, 26435, -1000),
    p: (36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000),
    h: (75, 362, 0, 339, 0, 30),
};

/// The datasheet's integer compensation (what Adafruit_BME280 computes): t_fine and °C x 100.
pub fn bme280_temperature(c: &Bme280Calib, adc_t: i32) -> (i32, i32) {
    let (t1, t2, t3) = (c.t.0 as i32, c.t.1 as i32, c.t.2 as i32);
    let var1 = (((adc_t >> 3) - (t1 << 1)) * t2) >> 11;
    let d = (adc_t >> 4) - t1;
    let var2 = (((d * d) >> 12) * t3) >> 14;
    let t_fine = var1 + var2;
    (t_fine, (t_fine * 5 + 128) >> 8)
}

/// Pa x 256.
pub fn bme280_pressure(c: &Bme280Calib, adc_p: i32, t_fine: i32) -> i64 {
    let (p1, p2, p3, p4, p5, p6, p7, p8, p9) =
        (c.p.0 as i64, c.p.1 as i64, c.p.2 as i64, c.p.3 as i64, c.p.4 as i64, c.p.5 as i64, c.p.6 as i64, c.p.7 as i64, c.p.8 as i64);
    let mut var1 = t_fine as i64 - 128000;
    let mut var2 = var1 * var1 * p6;
    var2 += (var1 * p5) << 17;
    var2 += p4 << 35;
    var1 = ((var1 * var1 * p3) >> 8) + ((var1 * p2) << 12);
    var1 = (((1i64 << 47) + var1) * p1) >> 33;
    if var1 == 0 { return 0; }
    let mut p = 1048576 - adc_p as i64;
    p = (((p << 31) - var2) * 3125) / var1;
    let var1 = (p9 * (p >> 13) * (p >> 13)) >> 25;
    let var2 = (p8 * p) >> 19;
    ((p + var1 + var2) >> 8) + (p7 << 4)
}

/// %RH x 1024.
pub fn bme280_humidity(c: &Bme280Calib, adc_h: i32, t_fine: i32) -> i64 {
    let (h1, h2, h3, h4, h5, h6) = (c.h.0 as i64, c.h.1 as i64, c.h.2 as i64, c.h.3 as i64, c.h.4 as i64, c.h.5 as i64);
    let mut v = t_fine as i64 - 76800;
    v = ((((adc_h as i64) << 14) - (h4 << 20) - (h5 * v)) + 16384) >> 15;
    v *= (((((((t_fine as i64 - 76800) * h6) >> 10) * ((((t_fine as i64 - 76800) * h3) >> 11) + 32768)) >> 10) + 2097152) * h2 + 8192) >> 14;
    v -= ((((v >> 15) * (v >> 15)) >> 7) * h1) >> 4;
    v = v.clamp(0, 419430400);
    v >> 12
}

/// The raw reading in [lo, hi] whose compensated value is closest to `target`, the compensation
/// being monotonic (rising or falling) over the range.
fn invert(lo: i32, hi: i32, target: f64, f: impl Fn(i32) -> f64) -> i32 {
    let rising = f(hi) > f(lo);
    let (mut a, mut b) = (lo, hi);
    while b - a > 1 {
        let m = a + (b - a) / 2;
        if (f(m) < target) == rising { a = m } else { b = m }
    }
    if (f(a) - target).abs() <= (f(b) - target).abs() { a } else { b }
}

/// The BME280's raw ADC words for the given values: (adc_T, adc_P, adc_H).
pub fn bme280_raw(c: &Bme280Calib, v: &SensorValues) -> (i32, i32, i32) {
    let adc_t = invert(0, (1 << 20) - 1, v.temperature * 100.0, |r| bme280_temperature(c, r).1 as f64);
    let t_fine = bme280_temperature(c, adc_t).0;
    let adc_p = invert(0, (1 << 20) - 1, v.pressure * 100.0 * 256.0, |r| bme280_pressure(c, r, t_fine) as f64);
    let adc_h = invert(0, (1 << 16) - 1, v.humidity * 1024.0, |r| bme280_humidity(c, r, t_fine) as f64);
    (adc_t, adc_p, adc_h)
}

/// A register-pointer I2C target: the first byte of a write sets the pointer, further bytes are
/// register writes, reads run from the pointer.
pub struct Bme280 {
    values: Shared,
    humidity: bool,
    regs: [u8; 256],
    ptr: u8,
    first: bool,
}

impl Bme280 {
    pub fn new(values: Shared, humidity: bool) -> Self {
        let mut regs = [0u8; 256];
        regs[0xd0] = if humidity { 0x60 } else { 0x58 };
        let c = &BME280_CALIB;
        let calib: [u16; 12] = [c.t.0, c.t.1 as u16, c.t.2 as u16, c.p.0, c.p.1 as u16, c.p.2 as u16, c.p.3 as u16, c.p.4 as u16, c.p.5 as u16, c.p.6 as u16, c.p.7 as u16, c.p.8 as u16];
        for (i, w) in calib.iter().enumerate() {
            regs[0x88 + 2 * i] = *w as u8;
            regs[0x89 + 2 * i] = (*w >> 8) as u8;
        }
        if humidity {
            regs[0xa1] = c.h.0;
            regs[0xe1] = c.h.1 as u8;
            regs[0xe2] = (c.h.1 >> 8) as u8;
            regs[0xe3] = c.h.2;
            regs[0xe4] = (c.h.3 >> 4) as u8;
            regs[0xe5] = ((c.h.3 & 0xf) as u8) | (((c.h.4 & 0xf) as u8) << 4);
            regs[0xe6] = (c.h.4 >> 4) as u8;
            regs[0xe7] = c.h.5 as u8;
        }
        Self { values, humidity, regs, ptr: 0, first: false }
    }

    /// Latch a measurement into 0xF7-0xFE, as the part does at the end of a conversion.
    fn measure(&mut self) {
        let (t, p, h) = bme280_raw(&BME280_CALIB, &values(&self.values));
        let r = &mut self.regs;
        r[0xf7] = (p >> 12) as u8; r[0xf8] = (p >> 4) as u8; r[0xf9] = ((p & 0xf) << 4) as u8;
        r[0xfa] = (t >> 12) as u8; r[0xfb] = (t >> 4) as u8; r[0xfc] = ((t & 0xf) << 4) as u8;
        if self.humidity { r[0xfd] = (h >> 8) as u8; r[0xfe] = h as u8; }
    }
}

impl I2cDevice for Bme280 {
    fn start(&mut self, read: bool) -> bool {
        self.first = !read;
        // a read of the data registers sees a fresh conversion (normal mode keeps converting)
        if read && self.ptr >= 0xf7 { self.measure(); }
        true
    }
    fn write(&mut self, b: u8) -> bool {
        if self.first {
            self.ptr = b;
            self.first = false;
        } else {
            match self.ptr {
                0xe0 if b == 0xb6 => { let keep = Self::new(self.values.clone(), self.humidity); self.regs = keep.regs; }
                0xf2 | 0xf4 | 0xf5 => self.regs[self.ptr as usize] = b,
                _ => {}
            }
            self.ptr = self.ptr.wrapping_add(1);
        }
        true
    }
    fn read(&mut self) -> u8 {
        let v = self.regs[self.ptr as usize];
        self.ptr = self.ptr.wrapping_add(1);
        v
    }
}

// ------------------------------------------------------------------ SHT3x

/// Sensirion's CRC-8: polynomial 0x31, init 0xFF.
pub fn sht_crc(data: &[u8]) -> u8 {
    let mut crc = 0xffu8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 { crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x31 } else { crc << 1 }; }
    }
    crc
}

/// SHT30/31/35: 16-bit commands; a single-shot measurement (0x24xx, 0x2Cxx) leaves six bytes to
/// read: temperature, CRC, humidity, CRC. T = -45 + 175 x raw / 65535, RH = 100 x raw / 65535.
pub struct Sht3x {
    values: Shared,
    cmd: Vec<u8>,
    out: Vec<u8>,
    at: usize,
}

impl Sht3x {
    pub fn new(values: Shared) -> Self { Self { values, cmd: Vec::new(), out: Vec::new(), at: 0 } }
    fn word(v: u16) -> [u8; 3] { let b = v.to_be_bytes(); [b[0], b[1], sht_crc(&b)] }
    fn command(&mut self, c: u16) {
        self.at = 0;
        self.out = match c >> 8 {
            0x24 | 0x2c | 0x20 | 0x21 | 0x22 | 0x23 | 0x27 | 0xe0 => {
                let v = values(&self.values);
                let t = (((v.temperature + 45.0) / 175.0) * 65535.0).round().clamp(0.0, 65535.0) as u16;
                let h = ((v.humidity / 100.0) * 65535.0).round().clamp(0.0, 65535.0) as u16;
                [Self::word(t), Self::word(h)].concat()
            }
            0xf3 if c == 0xf32d => Self::word(0x0000).to_vec(), // status: no alerts
            _ => Vec::new(),                                     // soft reset, heater, clear status
        };
    }
}

impl I2cDevice for Sht3x {
    fn start(&mut self, read: bool) -> bool { if !read { self.cmd.clear(); } true }
    fn write(&mut self, b: u8) -> bool {
        self.cmd.push(b);
        if self.cmd.len() == 2 { let c = u16::from_be_bytes([self.cmd[0], self.cmd[1]]); self.command(c); }
        true
    }
    fn read(&mut self) -> u8 {
        let v = self.out.get(self.at).copied().unwrap_or(0xff);
        self.at += 1;
        v
    }
}

// ------------------------------------------------------------------ AHT20

/// AHT20 (and AHT10/21): 0xAC 0x33 0x00 triggers a measurement; every read returns the status
/// byte (calibrated, not busy) then 20-bit humidity and 20-bit temperature and a CRC.
/// RH = raw / 2^20 x 100, T = raw / 2^20 x 200 - 50.
pub struct Aht20 {
    values: Shared,
    data: [u8; 6],
}

impl Aht20 {
    pub fn new(values: Shared) -> Self { Self { values, data: [0; 6] } }
}

pub struct Aht20Device { inner: Aht20, cmd: Vec<u8>, at: usize, frame: [u8; 7] }

impl Aht20Device {
    fn new(values: Shared) -> Self { Self { inner: Aht20::new(values), cmd: Vec::new(), at: 0, frame: [0x18, 0, 0, 0, 0, 0, 0] } }
}

impl Aht20 {
    fn measure(&mut self) {
        let v = values(&self.values);
        let h = ((v.humidity / 100.0) * 1048576.0).round().clamp(0.0, 1048575.0) as u32;
        let t = (((v.temperature + 50.0) / 200.0) * 1048576.0).round().clamp(0.0, 1048575.0) as u32;
        self.data = [(h >> 12) as u8, (h >> 4) as u8, (((h & 0xf) << 4) | (t >> 16)) as u8, (t >> 8) as u8, t as u8, 0];
    }
}

impl I2cDevice for Aht20Device {
    fn start(&mut self, read: bool) -> bool {
        if read {
            self.at = 0;
            let d = self.inner.data;
            let mut f = [0x18, d[0], d[1], d[2], d[3], d[4], 0];
            f[6] = sht_crc(&f[..6]);
            self.frame = f;
        } else {
            self.cmd.clear();
        }
        true
    }
    fn write(&mut self, b: u8) -> bool {
        self.cmd.push(b);
        if self.cmd.first() == Some(&0xac) && self.cmd.len() == 3 { self.inner.measure(); }
        true
    }
    fn read(&mut self) -> u8 {
        let v = self.frame.get(self.at).copied().unwrap_or(0xff);
        self.at += 1;
        v
    }
}

// ------------------------------------------------------------------ BH1750

/// BH1750: one-byte opcodes for power, reset, measurement mode and measurement time (MTreg);
/// a read returns the 16-bit count. lx = count / 1.2 x 69 / MTreg, halved in the H-resolution
/// mode 2.
pub struct Bh1750 {
    values: Shared,
    mode: u8,
    mtreg: u8,
    out: [u8; 2],
    at: usize,
}

impl Bh1750 {
    pub fn new(values: Shared) -> Self { Self { values, mode: 0x10, mtreg: 69, out: [0; 2], at: 0 } }
}

impl I2cDevice for Bh1750 {
    fn start(&mut self, read: bool) -> bool {
        if read {
            let lux = values(&self.values).lux;
            let mode2 = self.mode & 0x0f == 0x01;
            let count = (lux * 1.2 * self.mtreg as f64 / 69.0 * if mode2 { 2.0 } else { 1.0 }).round().clamp(0.0, 65535.0) as u16;
            self.out = count.to_be_bytes();
            self.at = 0;
        }
        true
    }
    fn write(&mut self, b: u8) -> bool {
        match b {
            0x10 | 0x11 | 0x13 | 0x20 | 0x21 | 0x23 => self.mode = b,
            0x40..=0x47 => self.mtreg = (self.mtreg & 0x1f) | ((b & 0x07) << 5),
            0x60..=0x7f => self.mtreg = (self.mtreg & 0xe0) | (b & 0x1f),
            _ => {} // power down / on / reset
        }
        true
    }
    fn read(&mut self) -> u8 {
        let v = self.out.get(self.at).copied().unwrap_or(0xff);
        self.at += 1;
        v
    }
}

// ------------------------------------------------------------------ MPU-6050

/// MPU-6050: a register file. WHO_AM_I (0x75) is 0x68; writing the reset bit of PWR_MGMT_1
/// (0x6B) returns the part to sleep (0x40); 0x3B-0x48 hold the accelerometer, temperature and
/// gyroscope readings, scaled by the ranges in ACCEL_CONFIG (0x1C) and GYRO_CONFIG (0x1B).
pub struct Mpu6050 {
    values: Shared,
    regs: [u8; 128],
    ptr: u8,
    first: bool,
}

impl Mpu6050 {
    pub fn new(values: Shared) -> Self {
        let mut regs = [0u8; 128];
        regs[0x75] = 0x68;
        regs[0x6b] = 0x40;
        Self { values, regs, ptr: 0, first: false }
    }
    fn sample(&mut self) {
        let v = values(&self.values);
        let a_lsb = 16384.0 / f64::from(1u16 << ((self.regs[0x1c] >> 3) & 3));
        let g_lsb = 131.0 / f64::from(1u16 << ((self.regs[0x1b] >> 3) & 3));
        let word = |x: f64| (x.round().clamp(-32768.0, 32767.0) as i16).to_be_bytes();
        let mut put = |at: usize, w: [u8; 2]| { self.regs[at] = w[0]; self.regs[at + 1] = w[1]; };
        for i in 0..3 { put(0x3b + 2 * i, word(v.accel[i] * a_lsb)); }
        put(0x41, word((v.temperature - 36.53) * 340.0));
        for i in 0..3 { put(0x43 + 2 * i, word(v.gyro[i] * g_lsb)); }
    }
}

impl I2cDevice for Mpu6050 {
    fn start(&mut self, read: bool) -> bool {
        self.first = !read;
        if read { self.sample(); }
        true
    }
    fn write(&mut self, b: u8) -> bool {
        if self.first {
            self.ptr = b & 0x7f;
            self.first = false;
            return true;
        }
        let at = self.ptr as usize;
        match at {
            0x6b if b & 0x80 != 0 => { self.regs = Self::new(self.values.clone()).regs; }
            0x75 | 0x3b..=0x48 => {}
            _ => self.regs[at] = b,
        }
        self.ptr = (self.ptr + 1) & 0x7f;
        true
    }
    fn read(&mut self) -> u8 {
        let v = self.regs[self.ptr as usize];
        self.ptr = (self.ptr + 1) & 0x7f;
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shared(f: impl FnOnce(&mut SensorValues)) -> Shared {
        let mut v = SensorValues::default();
        f(&mut v);
        Arc::new(Mutex::new(v))
    }
    /// Write `reg`, then read `n` bytes: what a register-reading driver does.
    fn read_regs(d: &mut dyn I2cDevice, reg: u8, n: usize) -> Vec<u8> {
        d.start(false); d.write(reg); d.stop();
        d.start(true);
        let out = (0..n).map(|_| d.read()).collect();
        d.stop();
        out
    }

    #[test]
    fn bme280_round_trips_through_the_drivers_compensation() {
        let v = shared(|v| { v.temperature = 21.37; v.pressure = 987.6; v.humidity = 63.2; });
        let mut d = Bme280::new(v, true);
        assert_eq!(read_regs(&mut d, 0xd0, 1), [0x60]);
        let cal = read_regs(&mut d, 0x88, 6);
        assert_eq!(u16::from_le_bytes([cal[0], cal[1]]), 27504);
        let r = read_regs(&mut d, 0xf7, 8);
        let adc_p = ((r[0] as i32) << 12) | ((r[1] as i32) << 4) | (r[2] as i32 >> 4);
        let adc_t = ((r[3] as i32) << 12) | ((r[4] as i32) << 4) | (r[5] as i32 >> 4);
        let adc_h = ((r[6] as i32) << 8) | r[7] as i32;
        let (t_fine, t) = bme280_temperature(&BME280_CALIB, adc_t);
        assert!((t as f64 / 100.0 - 21.37).abs() <= 0.01, "{t}");
        let p = bme280_pressure(&BME280_CALIB, adc_p, t_fine) as f64 / 256.0 / 100.0;
        assert!((p - 987.6).abs() < 0.02, "{p}");
        let h = bme280_humidity(&BME280_CALIB, adc_h, t_fine) as f64 / 1024.0;
        assert!((h - 63.2).abs() < 0.05, "{h}");
    }

    /// The same raw words vibe-iot's TypeScript port (src/sim/devices/sensors.ts) produces, so the
    /// Uno and the ESP32 boards read a BME280 identically.
    #[test]
    fn bme280_raw_words_match_the_typescript_port() {
        assert_eq!(bme280_raw(&BME280_CALIB, &SensorValues::default()), (516424, 410281, 30629));
        let v = SensorValues { temperature: 21.37, pressure: 987.6, humidity: 63.2, ..SensorValues::default() };
        assert_eq!(bme280_raw(&BME280_CALIB, &v), (508032, 422853, 33077));
    }

    #[test]
    fn bmp280_is_its_own_chip() {
        let mut d = Bme280::new(shared(|_| {}), false);
        assert_eq!(read_regs(&mut d, 0xd0, 1), [0x58]);
    }

    #[test]
    fn sht31_single_shot_with_crcs() {
        let mut d = Sht3x::new(shared(|v| { v.temperature = 25.0; v.humidity = 40.0; }));
        d.start(false); d.write(0x24); d.write(0x00); d.stop();
        d.start(true);
        let b: Vec<u8> = (0..6).map(|_| d.read()).collect();
        assert_eq!(sht_crc(&[0xbe, 0xef]), 0x92, "the datasheet's CRC example");
        assert_eq!(b[2], sht_crc(&b[0..2]));
        assert_eq!(b[5], sht_crc(&b[3..5]));
        let t = -45.0 + 175.0 * u16::from_be_bytes([b[0], b[1]]) as f64 / 65535.0;
        let h = 100.0 * u16::from_be_bytes([b[3], b[4]]) as f64 / 65535.0;
        assert!((t - 25.0).abs() < 0.01 && (h - 40.0).abs() < 0.01, "{t} {h}");
    }

    #[test]
    fn aht20_reports_calibrated_and_reads_back() {
        let mut d = Aht20Device::new(shared(|v| { v.temperature = 30.5; v.humidity = 55.0; }));
        d.start(true);
        assert_eq!(d.read() & 0x88, 0x08, "calibrated, not busy");
        d.start(false); for b in [0xac, 0x33, 0x00] { d.write(b); }
        d.start(true);
        let f: Vec<u8> = (0..7).map(|_| d.read()).collect();
        let h = ((f[1] as u32) << 12) | ((f[2] as u32) << 4) | (f[3] as u32 >> 4);
        let t = (((f[3] & 0xf) as u32) << 16) | ((f[4] as u32) << 8) | f[5] as u32;
        assert!((h as f64 / 1048576.0 * 100.0 - 55.0).abs() < 0.01);
        assert!((t as f64 / 1048576.0 * 200.0 - 50.0 - 30.5).abs() < 0.01);
        assert_eq!(f[6], sht_crc(&f[..6]));
    }

    #[test]
    fn bh1750_counts_by_mode_and_mtreg() {
        let mut d = Bh1750::new(shared(|v| v.lux = 500.0));
        d.start(false); d.write(0x10);
        d.start(true);
        assert_eq!(u16::from_be_bytes([d.read(), d.read()]), 600);
        d.start(false); d.write(0x11); // H-resolution mode 2
        d.start(true);
        assert_eq!(u16::from_be_bytes([d.read(), d.read()]), 1200);
    }

    #[test]
    fn mpu6050_scales_by_its_ranges() {
        let mut d = Mpu6050::new(shared(|v| { v.accel = [0.5, -1.0, 1.0]; v.gyro = [10.0, 0.0, -20.0]; v.temperature = 30.0; }));
        assert_eq!(read_regs(&mut d, 0x75, 1), [0x68]);
        d.start(false); d.write(0x6b); d.write(0x80); d.stop();
        assert_eq!(read_regs(&mut d, 0x6b, 1), [0x40], "reset leaves it asleep");
        d.start(false); d.write(0x1c); d.write(0x08); d.stop(); // +-4 g
        let r = read_regs(&mut d, 0x3b, 14);
        let w = |i: usize| i16::from_be_bytes([r[i], r[i + 1]]) as f64;
        assert_eq!((w(0), w(2), w(4)), (4096.0, -8192.0, 8192.0));
        assert!((w(6) / 340.0 + 36.53 - 30.0).abs() < 0.01);
        assert_eq!((w(8), w(12)), (1310.0, -2620.0));
    }
}
