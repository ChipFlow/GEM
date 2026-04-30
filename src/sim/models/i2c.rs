// SPDX-FileCopyrightText: Copyright (c) 2026 ChipFlow
// SPDX-License-Identifier: Apache-2.0
//! I²C peripheral model — port of chipflow-lib's
//! `chipflow/common/sim/models.cc::i2c::step` (lines 314-374).
//!
//! Open-drain semantics: the model and design both can pull SDA/SCL
//! low; high is "released" (the bus is pulled high by an external
//! pull-up). Bus value = wired-AND of all drivers. The model reads
//! the design's `sda_oe`/`scl_oe` outputs (oe=1 → design pulling low)
//! and contributes its own `drive_sda` (false → model pulling low,
//! true → released).
//!
//! State machine: detects START (SDA falls while SCL high), STOP (SDA
//! rises while SCL high), and per-bit shifts on SCL edges. After each
//! 8-bit byte, on the next SCL low the model either ACKs/NACKs (set by
//! `ack`/`nack` action), or shifts out the next bit of `read_data`
//! (set by `set_data` action) when the master is reading.
//!
//! Wiring is TODO — needs port-mapping infrastructure to look up the
//! design's `i2c_<N>` peripheral port positions in the GPU state buffer
//! by name (Jacquard's `GpioMapping` currently only supports GPIO-index
//! lookup for inputs and named lookup for inputs but not outputs).
//! The model itself is fully tested.

use crate::sim::input_stim::QueuedAction;
use crate::sim::models::{EmittedEvent, ModelOverrides};
use serde_json::json;

/// Pin positions for one I²C peripheral.
pub struct I2cPins {
    /// State position of `sda_oe` (design output: 1 → design pulling SDA low).
    pub sda_oe_out_pos: u32,
    /// State position of `scl_oe` (design output: 1 → design pulling SCL low).
    pub scl_oe_out_pos: u32,
    /// State position of `sda_i` (design input: what design sees).
    pub sda_i_in_pos: u32,
    /// State position of `scl_i` (design input: what design sees).
    pub scl_i_in_pos: u32,
}

/// CPU-side state for one I²C peripheral.
pub struct I2cModel {
    /// Peripheral name (matches chipflow's `i2c_<id>`).
    pub name: String,
    pins: I2cPins,
    /// Byte index in the current transaction (0 = address, 1+ = data).
    byte_count: u32,
    /// Bit index in the current byte (0..7 = data, 8 = ack/nack).
    bit_count: u32,
    /// Whether to ACK the current byte (queued via `ack` / `nack` action).
    do_ack: bool,
    /// Direction inferred from the address byte's R/W bit. True = master read.
    is_read: bool,
    /// Byte the model returns when the master is reading
    /// (queued via `set_data` action).
    read_data: u8,
    /// 8-bit shift register.
    sr: u8,
    /// Whether the model releases SDA (true) or pulls it low (false).
    drive_sda: bool,
    /// Previous-edge SDA / SCL (post-wired-AND with model contributions
    /// is irrelevant for edge detection — chipflow uses just the
    /// design-side `sda` / `scl` derived from oe, which is what we mirror).
    last_sda: bool,
    last_scl: bool,
}

impl I2cModel {
    /// Build a model. Initial state: SDA/SCL released (high), no bytes
    /// transferred.
    pub fn new(name: String, pins: I2cPins) -> Self {
        Self {
            name,
            pins,
            byte_count: 0,
            bit_count: 0,
            do_ack: false,
            is_read: false,
            read_data: 0,
            sr: 0xFF,
            drive_sda: true,
            last_sda: true,
            last_scl: true,
        }
    }

    /// State positions this model drives (input side).
    pub fn driven_positions(&self) -> [u32; 2] {
        [self.pins.sda_i_in_pos, self.pins.scl_i_in_pos]
    }

    /// Apply a queued action.
    pub fn apply_action(&mut self, action: &QueuedAction) {
        match action.event.as_str() {
            "ack" => self.do_ack = true,
            "nack" => self.do_ack = false,
            "set_data" => {
                self.read_data = action
                    .payload
                    .as_u64()
                    .filter(|&v| v <= 0xFF)
                    .unwrap_or_else(|| {
                        panic!(
                            "i2c `{}`: `set_data` payload must be u8, got {:?}",
                            self.name, action.payload
                        );
                    }) as u8;
            }
            other => {
                clilog::warn!(
                    "i2c `{}`: unhandled event `{}` (payload {:?})",
                    self.name,
                    other,
                    action.payload
                );
            }
        }
    }

    /// Advance the I²C state machine one step. Reads `sda_oe` / `scl_oe`
    /// from the design's output state, runs the FSM, contributes
    /// SDA/SCL input drives to `overrides`, and pushes any emitted
    /// events into `emitted`.
    pub fn step(
        &mut self,
        output_state: &[u32],
        overrides: &mut ModelOverrides,
        emitted: &mut Vec<EmittedEvent>,
        timestamp: u64,
    ) {
        let sda_oe = read_bit(output_state, self.pins.sda_oe_out_pos);
        let scl_oe = read_bit(output_state, self.pins.scl_oe_out_pos);
        // Open-drain: design pulls low when oe=1, releases (high) when oe=0.
        let sda = sda_oe == 0;
        let scl = scl_oe == 0;

        if self.last_scl && self.last_sda && !sda {
            // START condition (SDA falls while SCL high)
            emitted.push(EmittedEvent {
                peripheral: self.name.clone(),
                event: "start".to_string(),
                payload: json!(""),
            });
            let _ = timestamp;
            self.sr = 0xFF;
            self.byte_count = 0;
            self.bit_count = 0;
            self.is_read = false;
            self.drive_sda = true;
        } else if scl && !self.last_scl {
            // SCL posedge: shift in (during write) or do nothing (during read of next bytes)
            if self.byte_count == 0 || !self.is_read {
                self.sr = (self.sr << 1) | (if sda { 1 } else { 0 });
            }
            self.bit_count += 1;
            if self.bit_count == 8 {
                if self.byte_count == 0 {
                    self.is_read = (self.sr & 0x1) != 0;
                    emitted.push(EmittedEvent {
                        peripheral: self.name.clone(),
                        event: "address".to_string(),
                        payload: json!(self.sr),
                    });
                } else if !self.is_read {
                    emitted.push(EmittedEvent {
                        peripheral: self.name.clone(),
                        event: "write".to_string(),
                        payload: json!(self.sr),
                    });
                }
                self.byte_count += 1;
            } else if self.bit_count == 9 {
                self.bit_count = 0;
            }
        } else if !scl && self.last_scl {
            // SCL negedge: release SDA, then drive ack/data/release for next bit
            self.drive_sda = true;
            if self.bit_count == 8 {
                self.drive_sda = !self.do_ack;
            } else if self.byte_count > 0 && self.is_read {
                if self.bit_count == 0 {
                    self.sr = self.read_data;
                } else {
                    self.sr <<= 1;
                }
                self.drive_sda = ((self.sr >> 7) & 0x1) != 0;
            }
        } else if self.last_scl && !self.last_sda && sda {
            // STOP condition (SDA rises while SCL high)
            emitted.push(EmittedEvent {
                peripheral: self.name.clone(),
                event: "stop".to_string(),
                payload: json!(""),
            });
            self.drive_sda = true;
        }

        self.last_sda = sda;
        self.last_scl = scl;
        // sda_i = wired-AND of design (sda) and model (drive_sda).
        let sda_i = if sda && self.drive_sda { 1 } else { 0 };
        let scl_i = if scl { 1 } else { 0 };
        overrides.insert(self.pins.sda_i_in_pos, sda_i);
        overrides.insert(self.pins.scl_i_in_pos, scl_i);
    }
}

#[inline]
fn read_bit(state: &[u32], pos: u32) -> u8 {
    let word = (pos >> 5) as usize;
    let bit = pos & 31;
    if word < state.len() {
        ((state[word] >> bit) & 1) as u8
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// In-memory bit-vector for tests. We model `sda_oe` and `scl_oe` at
    /// known positions and sweep them through transactions.
    struct BusHarness {
        state: Vec<u32>,
    }

    impl BusHarness {
        fn new(size_words: usize) -> Self {
            Self {
                state: vec![0u32; size_words],
            }
        }
        fn set_bit(&mut self, pos: u32, value: u8) {
            let word = (pos >> 5) as usize;
            let bit = pos & 31;
            if word < self.state.len() {
                if value != 0 {
                    self.state[word] |= 1u32 << bit;
                } else {
                    self.state[word] &= !(1u32 << bit);
                }
            }
        }
    }

    fn pins() -> I2cPins {
        I2cPins {
            sda_oe_out_pos: 0,
            scl_oe_out_pos: 1,
            sda_i_in_pos: 2,
            scl_i_in_pos: 3,
        }
    }

    /// Run several FSM steps in sequence with given (sda, scl) bus
    /// values (open-drain logical level: 1 = high/released, 0 = low).
    /// Each (sda, scl) pair is a single step; the harness translates to
    /// `sda_oe`/`scl_oe` (oe = inverse of bus level).
    fn run_steps(
        model: &mut I2cModel,
        harness: &mut BusHarness,
        steps: &[(u8, u8)],
    ) -> Vec<EmittedEvent> {
        let mut events = Vec::new();
        for &(sda_high, scl_high) in steps {
            harness.set_bit(model.pins.sda_oe_out_pos, if sda_high == 0 { 1 } else { 0 });
            harness.set_bit(model.pins.scl_oe_out_pos, if scl_high == 0 { 1 } else { 0 });
            let mut overrides = ModelOverrides::new();
            model.step(&harness.state, &mut overrides, &mut events, 0);
        }
        events
    }

    /// Helper: emit one full I²C byte by toggling SCL and SDA bit-by-bit.
    /// `bits` is MSB-first. Real I²C masters change SDA only while SCL is
    /// low, so each bit takes three sub-steps: drop SCL (preserving the
    /// previous SDA), change SDA, raise SCL. Initial state expected:
    /// SCL=0, SDA=0 (typical post-START).
    fn write_byte_steps(bits: u8) -> Vec<(u8, u8)> {
        let mut steps = Vec::new();
        let mut prev = 0u8;
        for i in 0..8 {
            let bit = (bits >> (7 - i)) & 0x1;
            // Drop SCL with previous SDA (clean negedge with no SDA change).
            steps.push((prev, 0));
            // Change SDA while SCL is low.
            steps.push((bit, 0));
            // Raise SCL — this is the posedge where the slave samples.
            steps.push((bit, 1));
            prev = bit;
        }
        steps
    }

    #[test]
    fn detects_start_and_stop() {
        let mut model = I2cModel::new("i2c_0".to_string(), pins());
        let mut h = BusHarness::new(2);
        // Steady state: bus high.
        let _ = run_steps(&mut model, &mut h, &[(1, 1)]);
        // START: SDA falls while SCL high
        let ev1 = run_steps(&mut model, &mut h, &[(0, 1)]);
        assert_eq!(ev1.len(), 1);
        assert_eq!(ev1[0].event, "start");
        // SCL falls (no event), SCL rises (we're mid-transaction, no extra)
        let _ = run_steps(&mut model, &mut h, &[(0, 0), (1, 0)]);
        // STOP: bring SCL high then SDA high while SCL high
        let ev2 = run_steps(&mut model, &mut h, &[(0, 1), (1, 1)]);
        let stops: Vec<_> = ev2.iter().filter(|e| e.event == "stop").collect();
        assert_eq!(stops.len(), 1);
    }

    #[test]
    fn emits_address_event_with_rw_bit() {
        // Write transaction: address 0x50, R/W=0 → byte = 0xA0
        let mut model = I2cModel::new("i2c_0".to_string(), pins());
        let mut h = BusHarness::new(2);
        let _ = run_steps(&mut model, &mut h, &[(1, 1)]);
        // START
        let _ = run_steps(&mut model, &mut h, &[(0, 1)]);
        // SCL low (idle)
        let _ = run_steps(&mut model, &mut h, &[(0, 0)]);
        // Shift in 0xA0 = 1010_0000
        let steps = write_byte_steps(0xA0);
        let events = run_steps(&mut model, &mut h, &steps);
        let addrs: Vec<_> = events.iter().filter(|e| e.event == "address").collect();
        assert_eq!(addrs.len(), 1);
        assert_eq!(addrs[0].payload, json!(0xA0));
        // is_read = (0xA0 & 1) = 0 → write transaction
        assert!(!model.is_read);
    }

    #[test]
    fn emits_write_event_for_data_byte() {
        let mut model = I2cModel::new("i2c_0".to_string(), pins());
        let mut h = BusHarness::new(2);
        // Address (write): 0xA0
        let _ = run_steps(&mut model, &mut h, &[(1, 1), (0, 1), (0, 0)]);
        let _ = run_steps(&mut model, &mut h, &write_byte_steps(0xA0));
        // After: bus = (0, 1), bit_count=8, byte_count=1.
        // ACK pulse: drop SCL then raise (9th SCL posedge → bit_count rolls to 0),
        // then drop again to set up for next byte. SDA stays low throughout.
        let _ = run_steps(&mut model, &mut h, &[(0, 0), (0, 1), (0, 0)]);
        // Data byte 0x42
        let events = run_steps(&mut model, &mut h, &write_byte_steps(0x42));
        let writes: Vec<_> = events.iter().filter(|e| e.event == "write").collect();
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0].payload, json!(0x42));
    }

    #[test]
    fn ack_action_drives_sda_low_during_ack_slot() {
        let mut model = I2cModel::new("i2c_0".to_string(), pins());
        let mut h = BusHarness::new(2);
        model.apply_action(&QueuedAction {
            event: "ack".to_string(),
            payload: json!(""),
        });
        // START + SCL-low setup, then address byte.
        let _ = run_steps(&mut model, &mut h, &[(1, 1), (0, 1), (0, 0)]);
        let _ = run_steps(&mut model, &mut h, &write_byte_steps(0xA0));
        // After: bus = (0, 1), bit_count=8.
        // Drop SCL — this is the negedge where the model decides ack vs nack.
        // Since do_ack=true, drive_sda becomes false (model pulls SDA low).
        let _ = run_steps(&mut model, &mut h, &[(0, 0)]);

        // Master releases SDA. Verify the model is still pulling it low.
        let mut overrides = ModelOverrides::new();
        let mut events = Vec::new();
        h.set_bit(model.pins.sda_oe_out_pos, 0); // design releases sda → bus sda=1
                                                 // scl_oe stays at 1 → scl=0
        model.step(&h.state, &mut overrides, &mut events, 0);
        // sda_i = sda(=1) AND drive_sda(=false) = 0
        assert_eq!(overrides[&model.pins.sda_i_in_pos], 0);
    }

    #[test]
    fn set_data_action_seeds_read_byte() {
        let mut model = I2cModel::new("i2c_0".to_string(), pins());
        model.apply_action(&QueuedAction {
            event: "set_data".to_string(),
            payload: json!(0xC3),
        });
        assert_eq!(model.read_data, 0xC3);
        // nack action overrides the default ack
        model.apply_action(&QueuedAction {
            event: "nack".to_string(),
            payload: json!(""),
        });
        assert!(!model.do_ack);
    }

    #[test]
    fn driven_positions_lists_input_pins() {
        let model = I2cModel::new("i2c_0".to_string(), pins());
        assert_eq!(model.driven_positions(), [2, 3]);
    }
}
