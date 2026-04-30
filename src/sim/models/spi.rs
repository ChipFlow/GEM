// SPDX-FileCopyrightText: Copyright (c) 2026 ChipFlow
// SPDX-License-Identifier: Apache-2.0
//! Generic SPI peripheral (slave) model — port of chipflow-lib's
//! `chipflow/common/sim/models.cc::spi::step`.
//!
//! The design is the SPI master; this model is the slave it talks to.
//! Wires:
//! - `clk`  : design drives, model reads (SCK)
//! - `csn`  : design drives, model reads (active-low chip select)
//! - `copi` : design drives, model reads (controller-out / peripheral-in)
//! - `cipo` : model drives, design reads (controller-in / peripheral-out)
//!
//! Actions:
//! - `set_data` (u32 payload): load the byte/word the model will return
//!   on the next CSN-low transaction.
//! - `set_width` (u32 payload): set bit-width per frame (default 8).
//!
//! Events emitted:
//! - `select` on CSN falling edge.
//! - `deselect` on CSN rising edge.
//! - `data` (u32 payload of received bits) every `width` SCK posedges.
//!
//! Wiring is TODO — needs port-mapping infrastructure to look up the
//! design's `user_spi_<N>` peripheral port positions in the GPU state
//! buffer by name. The model itself is fully tested.

use crate::sim::input_stim::QueuedAction;
use crate::sim::models::{
    payload_u32_bounded, read_bit, warn_unhandled, Edge, EdgeDetector, EmittedEvent,
    ModelOverrides, PeripheralModel,
};
use serde_json::json;

/// Pin positions for one SPI peripheral.
pub struct SpiPins {
    /// State position of `clk` (design output).
    pub clk_out_pos: u32,
    /// State position of `csn` (design output).
    pub csn_out_pos: u32,
    /// State position of `copi` (design output).
    pub copi_out_pos: u32,
    /// State position of `cipo` (design input — model drives).
    pub cipo_in_pos: u32,
}

/// CPU-side state for one SPI peripheral.
pub struct SpiModel {
    pub name: String,
    pins: SpiPins,
    driven_positions_arr: [u32; 1],
    clk_edge: EdgeDetector,
    csn_edge: EdgeDetector,
    bit_count: u32,
    send_data: u32,
    width: u32,
    in_buffer: u32,
    out_buffer: u32,
    cipo_value: u8,
}

impl SpiModel {
    pub fn new(name: String, pins: SpiPins) -> Self {
        let driven_positions_arr = [pins.cipo_in_pos];
        Self {
            name,
            pins,
            driven_positions_arr,
            clk_edge: EdgeDetector::new(false),
            csn_edge: EdgeDetector::new(false),
            bit_count: 0,
            send_data: 0,
            width: 8,
            in_buffer: 0,
            out_buffer: 0,
            cipo_value: 0,
        }
    }
}

impl PeripheralModel for SpiModel {
    fn name(&self) -> &str {
        &self.name
    }

    fn driven_positions(&self) -> &[u32] {
        &self.driven_positions_arr
    }

    fn apply_action(&mut self, action: &QueuedAction) {
        let model = format!("spi `{}`", self.name);
        match action.event.as_str() {
            "set_data" => {
                let v = payload_u32_bounded(action, &model, u32::MAX);
                self.send_data = v;
                self.out_buffer = v;
            }
            "set_width" => {
                self.width = payload_u32_bounded(action, &model, 32);
                if self.width == 0 {
                    panic!("{}: `set_width` payload must be 1..=32, got 0", model);
                }
            }
            _ => warn_unhandled(&model, action),
        }
    }

    fn step_edge(
        &mut self,
        output_state: &[u32],
        overrides: &mut ModelOverrides,
        emitted: &mut Vec<EmittedEvent>,
    ) {
        let clk = read_bit(output_state, self.pins.clk_out_pos) != 0;
        let csn = read_bit(output_state, self.pins.csn_out_pos) != 0;
        let copi = read_bit(output_state, self.pins.copi_out_pos) as u32;

        let csn_e = self.csn_edge.update(csn);
        let clk_e = self.clk_edge.update(clk);

        match csn_e {
            Edge::Rising => {
                // CSN rising: deselect
                self.bit_count = 0;
                self.in_buffer = 0;
                self.out_buffer = self.send_data;
                emitted.push(EmittedEvent::new(&self.name, "deselect", json!("")));
            }
            Edge::Falling => {
                emitted.push(EmittedEvent::new(&self.name, "select", json!("")));
            }
            Edge::None => match clk_e {
                Edge::Rising if !csn => {
                    // CLK posedge while selected: shift in copi, shift out_buffer, count.
                    self.in_buffer = (self.in_buffer << 1) | (copi & 0x1);
                    self.out_buffer <<= 1;
                    self.bit_count += 1;
                    if self.bit_count == self.width {
                        emitted.push(EmittedEvent::new(&self.name, "data", json!(self.in_buffer)));
                        self.bit_count = 0;
                    }
                }
                Edge::Falling if !csn => {
                    // CLK negedge while selected: drive cipo with MSB of out_buffer.
                    self.cipo_value = ((self.out_buffer >> (self.width - 1)) & 0x1) as u8;
                }
                _ => {}
            },
        }

        self.contribute_overrides(overrides);
    }

    fn contribute_overrides(&self, overrides: &mut ModelOverrides) {
        overrides.insert(self.pins.cipo_in_pos, self.cipo_value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
            if value != 0 {
                self.state[word] |= 1u32 << bit;
            } else {
                self.state[word] &= !(1u32 << bit);
            }
        }
    }

    fn pins() -> SpiPins {
        SpiPins {
            clk_out_pos: 0,
            csn_out_pos: 1,
            copi_out_pos: 2,
            cipo_in_pos: 3,
        }
    }

    fn step(
        model: &mut SpiModel,
        h: &mut BusHarness,
        clk: u8,
        csn: u8,
        copi: u8,
    ) -> (Vec<EmittedEvent>, u8) {
        h.set_bit(model.pins.clk_out_pos, clk);
        h.set_bit(model.pins.csn_out_pos, csn);
        h.set_bit(model.pins.copi_out_pos, copi);
        let mut overrides = ModelOverrides::new();
        let mut events = Vec::new();
        model.step_edge(&h.state, &mut overrides, &mut events);
        let cipo = overrides[&model.pins.cipo_in_pos];
        (events, cipo)
    }

    #[test]
    fn select_and_deselect_events() {
        let mut model = SpiModel::new("user_spi_0".to_string(), pins());
        let mut h = BusHarness::new(1);
        let (ev, _) = step(&mut model, &mut h, 0, 1, 0);
        assert_eq!(ev.len(), 1);
        assert_eq!(ev[0].event, "deselect");
        let (ev, _) = step(&mut model, &mut h, 0, 0, 0);
        assert_eq!(ev.len(), 1);
        assert_eq!(ev[0].event, "select");
        let (ev, _) = step(&mut model, &mut h, 0, 0, 0);
        assert_eq!(ev.len(), 0);
        let (ev, _) = step(&mut model, &mut h, 0, 1, 0);
        assert_eq!(ev.len(), 1);
        assert_eq!(ev[0].event, "deselect");
    }

    #[test]
    fn shifts_in_a_byte_msb_first() {
        let mut model = SpiModel::new("user_spi_0".to_string(), pins());
        let mut h = BusHarness::new(1);
        let _ = step(&mut model, &mut h, 0, 1, 0); // initial deselect
        let _ = step(&mut model, &mut h, 0, 0, 0); // select

        let mut all_events = Vec::new();
        for i in 0..8 {
            let bit = (0xA5u8 >> (7 - i)) & 0x1;
            let _ = step(&mut model, &mut h, 0, 0, bit);
            let (ev, _) = step(&mut model, &mut h, 1, 0, bit);
            all_events.extend(ev);
            let _ = step(&mut model, &mut h, 0, 0, bit);
        }
        let datas: Vec<_> = all_events.iter().filter(|e| e.event == "data").collect();
        assert_eq!(datas.len(), 1);
        assert_eq!(datas[0].payload, json!(0xA5));
    }

    #[test]
    fn shifts_out_send_data_msb_first() {
        let mut model = SpiModel::new("user_spi_0".to_string(), pins());
        let mut h = BusHarness::new(1);
        model.apply_action(&QueuedAction {
            event: "set_data".to_string(),
            payload: json!(0x3C),
        });
        let _ = step(&mut model, &mut h, 0, 1, 0); // initial deselect (reload)
        let _ = step(&mut model, &mut h, 0, 0, 0); // select

        let mut shifted = 0u8;
        for _ in 0..8 {
            let (_, cipo_low) = step(&mut model, &mut h, 0, 0, 0);
            let (_, _) = step(&mut model, &mut h, 1, 0, 0);
            shifted = (shifted << 1) | cipo_low;
        }
        assert_eq!(shifted, 0x3C);
    }

    #[test]
    fn set_width_changes_frame_size() {
        let mut model = SpiModel::new("user_spi_0".to_string(), pins());
        let mut h = BusHarness::new(1);
        model.apply_action(&QueuedAction {
            event: "set_width".to_string(),
            payload: json!(4),
        });
        let _ = step(&mut model, &mut h, 0, 1, 0);
        let _ = step(&mut model, &mut h, 0, 0, 0);

        let mut all_events = Vec::new();
        for i in 0..4 {
            let bit = (0xBu8 >> (3 - i)) & 0x1;
            let _ = step(&mut model, &mut h, 0, 0, bit);
            let (ev, _) = step(&mut model, &mut h, 1, 0, bit);
            all_events.extend(ev);
            let _ = step(&mut model, &mut h, 0, 0, bit);
        }
        let datas: Vec<_> = all_events.iter().filter(|e| e.event == "data").collect();
        assert_eq!(datas.len(), 1);
        assert_eq!(datas[0].payload, json!(0xB));
    }

    #[test]
    #[should_panic(expected = "must fit in 0..=4294967295")]
    fn invalid_set_data_panics() {
        let mut model = SpiModel::new("user_spi_0".to_string(), pins());
        model.apply_action(&QueuedAction {
            event: "set_data".to_string(),
            payload: json!(u64::MAX),
        });
    }

    #[test]
    #[should_panic(expected = "must be 1..=32")]
    fn invalid_set_width_panics() {
        let mut model = SpiModel::new("user_spi_0".to_string(), pins());
        model.apply_action(&QueuedAction {
            event: "set_width".to_string(),
            payload: json!(0),
        });
    }

    #[test]
    fn driven_positions_lists_cipo() {
        let model = SpiModel::new("user_spi_0".to_string(), pins());
        assert_eq!(model.driven_positions(), [3]);
    }
}
