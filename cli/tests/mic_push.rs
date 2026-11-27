// The page's microphone (binary type 4, esp-soc/src/machine/web.rs): its PCM goes into a
// routed source on the mic's I2S pins, made on first use and remade when the rate changes.
use esp_periph::i2s::PcmPins;
use esp_soc::SocBus;

#[test]
fn push_mic_feeds_a_routed_source_on_the_mic_pins() {
    let mut m = esp32s3::machine([0; 6]);
    m.push_mic(5, 4, 6, 16000, &[[100, 100]; 64]).unwrap();
    m.push_mic(5, 4, 6, 16000, &[[200, 200]; 36]).unwrap();
    let bank = m.bus.pcm_sources().expect("a bank once the mic pushed");
    let sources: Vec<_> = bank.inputs.iter().flatten().collect();
    assert_eq!(sources.len(), 1, "one source per wiring");
    assert_eq!(sources[0].pins(), Some(PcmPins::I2s { bclk: 5, ws: 4, data: 6 }));
    assert_eq!(sources[0].rate(), 16000);
    assert_eq!(sources[0].queued_frames(), 100);
}

#[test]
fn push_mic_remakes_the_source_when_the_rate_changes_and_keeps_wirings_apart() {
    let mut m = esp32s3::machine([0; 6]);
    m.push_mic(5, 4, 6, 16000, &[[1, 1]; 10]).unwrap();
    m.push_mic(5, 4, 6, 48000, &[[2, 2]; 3]).unwrap();
    m.push_mic(12, 13, 14, 22050, &[[3, 3]; 5]).unwrap();
    let bank = m.bus.pcm_sources().unwrap();
    let sources: Vec<_> = bank.inputs.iter().flatten().collect();
    assert_eq!(sources.len(), 2);
    assert_eq!((sources[0].rate(), sources[0].queued_frames()), (48000, 3));
    assert_eq!(sources[1].pins(), Some(PcmPins::I2s { bclk: 12, ws: 13, data: 14 }));
}

#[test]
fn push_mic_refuses_a_bad_rate_or_shared_pins() {
    let mut m = esp32s3::machine([0; 6]);
    assert!(m.push_mic(5, 4, 6, 4000, &[[0, 0]]).is_err());
    assert!(m.push_mic(5, 5, 6, 16000, &[[0, 0]]).is_err());
}
