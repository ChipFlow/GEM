// SPDX-FileCopyrightText: Copyright (c) 2026 ChipFlow
// SPDX-License-Identifier: Apache-2.0
//! GPIO peripheral model — port of chipflow-lib `chipflow/common/sim/models.h`
//! template `gpio<pin_count>` (lines 88-139).
//!
//! Behavior:
//! - On a `set` action with a binary-string payload (MSB-first), updates
//!   the internal `input_data` value driving the design's `$i` port.
//! - (TODO) On output-side change of `$o` or `$oe`, emit a `change` event
//!   so input.json `wait` commands can synchronize on it. Output-side
//!   reads need access to `$o`/`$oe` state positions which require
//!   netlist port-mapping support not yet wired through GpioMapping;
//!   tracked as a follow-up.

use crate::sim::input_stim::QueuedAction;
use crate::sim::models::ModelOverrides;
use crate::testbench::GpioConfig;

/// CPU-side state for one GPIO peripheral.
pub struct GpioModel {
    /// Peripheral name (matches chipflow's `gpio_<id>`).
    pub name: String,
    /// State-buffer positions for the `$i` (input) pins, LSB-first.
    /// `pin_input_positions[i]` is the GPU state position for bit `i` of
    /// the input bus.
    pin_input_positions: Vec<u32>,
    /// Current driven value, bit `i` ↔ `pin_input_positions[i]`.
    input_data: u32,
}

impl GpioModel {
    /// Build a model for the given GpioConfig. `gpio_to_input_pos` is a
    /// callback that resolves a GPIO index (from `GpioConfig.pins[i]`)
    /// to its `$i` state-buffer position. Returns `None` if any pin can
    /// not be resolved.
    pub fn new(cfg: &GpioConfig, gpio_to_input_pos: impl Fn(usize) -> Option<u32>) -> Option<Self> {
        let mut pin_input_positions = Vec::with_capacity(cfg.pins.len());
        for &pin in &cfg.pins {
            pin_input_positions.push(gpio_to_input_pos(pin)?);
        }
        Some(Self {
            name: cfg.name.clone(),
            pin_input_positions,
            input_data: 0,
        })
    }

    /// Number of bits in this GPIO bus.
    pub fn pin_count(&self) -> usize {
        self.pin_input_positions.len()
    }

    /// Apply a queued action. Currently handles `set` only; other events
    /// log and are otherwise ignored.
    pub fn apply_action(&mut self, action: &QueuedAction) {
        match action.event.as_str() {
            "set" => {
                let payload_str = action.payload.as_str().unwrap_or_else(|| {
                    panic!(
                        "gpio model `{}`: `set` payload must be a string of '0'/'1'/'Z' chars, got {:?}",
                        self.name, action.payload
                    );
                });
                self.input_data = parse_set_bitstring(payload_str, self.pin_count(), &self.name);
            }
            other => {
                clilog::warn!(
                    "gpio model `{}`: unhandled event `{}` (payload {:?})",
                    self.name,
                    other,
                    action.payload
                );
            }
        }
    }

    /// Contribute this model's driven bit values to the shared override
    /// map, which the cosim loop syncs into the per-edge state_prep ops.
    pub fn contribute_overrides(&self, overrides: &mut ModelOverrides) {
        for (bit_idx, &pos) in self.pin_input_positions.iter().enumerate() {
            let val = ((self.input_data >> bit_idx) & 1) as u8;
            overrides.insert(pos, val);
        }
    }

    /// State positions this model drives. Returned for cosim setup so
    /// placeholder BitOps for these positions can be appended to the
    /// per-edge ops at construction time.
    pub fn driven_positions(&self) -> &[u32] {
        &self.pin_input_positions
    }
}

/// Parse a chipflow-style `set` payload (MSB-first binary string,
/// `'0'`/`'1'` characters; `'Z'` meaning high-impedance is treated here
/// as "drive 0" — without per-pin direction control we have no high-Z
/// representation in the GPU input bits). String length must equal
/// `pin_count`.
fn parse_set_bitstring(payload: &str, pin_count: usize, model_name: &str) -> u32 {
    assert_eq!(
        payload.len(),
        pin_count,
        "gpio model `{}`: `set` payload length {} doesn't match pin_count {}",
        model_name,
        payload.len(),
        pin_count
    );
    let mut value: u32 = 0;
    let bytes = payload.as_bytes();
    // chipflow convention: bin.at((pin_count - 1) - i) = bit i. Equivalently,
    // last char of the string is bit 0 (LSB), first char is the MSB.
    for i in 0..pin_count {
        let c = bytes[(pin_count - 1) - i];
        if c == b'1' {
            value |= 1u32 << i;
        } else if c != b'0' && c != b'Z' && c != b'z' {
            panic!(
                "gpio model `{}`: invalid char `{}` in set payload `{}`",
                model_name, c as char, payload
            );
        }
    }
    value
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn cfg(name: &str, pins: Vec<usize>) -> GpioConfig {
        GpioConfig {
            name: name.to_string(),
            pins,
        }
    }

    #[test]
    fn parses_set_bitstring_msb_first() {
        // chipflow convention: "00111100" => bit 0 = '0', bit 1 = '0', bit 2 = '1', bit 3 = '1',
        // bit 4 = '1', bit 5 = '1', bit 6 = '0', bit 7 = '0'  →  0b00111100 = 0x3C
        assert_eq!(parse_set_bitstring("00111100", 8, "test"), 0x3C);
        assert_eq!(parse_set_bitstring("01010101", 8, "test"), 0x55);
        assert_eq!(parse_set_bitstring("10000000", 8, "test"), 0x80);
        assert_eq!(parse_set_bitstring("00000001", 8, "test"), 0x01);
        assert_eq!(parse_set_bitstring("11", 2, "test"), 0x03);
        assert_eq!(parse_set_bitstring("0", 1, "test"), 0x00);
    }

    #[test]
    #[should_panic(expected = "doesn't match pin_count")]
    fn rejects_wrong_length() {
        parse_set_bitstring("0011", 8, "test");
    }

    #[test]
    fn z_treated_as_zero() {
        // Without per-pin direction control we can't represent high-Z, so 'Z' becomes 0.
        assert_eq!(parse_set_bitstring("ZZZZZZZZ", 8, "test"), 0x00);
        assert_eq!(parse_set_bitstring("Z011Z011", 8, "test"), 0b00110011);
    }

    #[test]
    fn model_resolves_pins_and_applies_set() {
        let g = cfg("gpio_1", vec![10, 11, 12, 13, 14, 15, 16, 17]);
        // Pretend gpio idx N maps to state pos N + 100.
        let model = GpioModel::new(&g, |idx| Some(idx as u32 + 100)).unwrap();
        assert_eq!(model.pin_count(), 8);
        assert_eq!(
            model.driven_positions(),
            &[110, 111, 112, 113, 114, 115, 116, 117]
        );
    }

    #[test]
    fn missing_pin_returns_none() {
        let g = cfg("gpio_x", vec![10, 11, 999]);
        let model = GpioModel::new(&g, |idx| if idx == 999 { None } else { Some(idx as u32) });
        assert!(model.is_none());
    }

    #[test]
    fn apply_set_updates_overrides() {
        let g = cfg("gpio_1", vec![10, 11, 12, 13, 14, 15, 16, 17]);
        let mut model = GpioModel::new(&g, |idx| Some(idx as u32 + 100)).unwrap();
        model.apply_action(&QueuedAction {
            event: "set".to_string(),
            payload: json!("00111100"),
        });
        let mut ov = ModelOverrides::new();
        model.contribute_overrides(&mut ov);
        // 0x3C = bits 2,3,4,5 set
        assert_eq!(ov[&110], 0); // bit 0
        assert_eq!(ov[&111], 0); // bit 1
        assert_eq!(ov[&112], 1); // bit 2
        assert_eq!(ov[&113], 1); // bit 3
        assert_eq!(ov[&114], 1); // bit 4
        assert_eq!(ov[&115], 1); // bit 5
        assert_eq!(ov[&116], 0); // bit 6
        assert_eq!(ov[&117], 0); // bit 7
    }

    #[test]
    fn unknown_event_logs_warning_and_skips() {
        let g = cfg("gpio_1", vec![10]);
        let mut model = GpioModel::new(&g, |idx| Some(idx as u32 + 100)).unwrap();
        // Pre-populate input_data so we can check it doesn't change.
        model.apply_action(&QueuedAction {
            event: "set".to_string(),
            payload: json!("1"),
        });
        let mut ov = ModelOverrides::new();
        model.contribute_overrides(&mut ov);
        assert_eq!(ov[&110], 1);

        // Unknown event should not modify state.
        model.apply_action(&QueuedAction {
            event: "wibble".to_string(),
            payload: json!("anything"),
        });
        ov.clear();
        model.contribute_overrides(&mut ov);
        assert_eq!(ov[&110], 1);
    }
}
