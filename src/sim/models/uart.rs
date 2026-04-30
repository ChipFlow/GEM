// SPDX-FileCopyrightText: Copyright (c) 2026 ChipFlow
// SPDX-License-Identifier: Apache-2.0
//! UART RX-driver peripheral model — port of the TX-side of chipflow-lib's
//! `chipflow/common/sim/models.cc::uart::step` (lines around `s.tx_active`).
//!
//! The RX-decoder side (decoding the design's TX line into bytes) is
//! already implemented on the GPU as `gpu_io_step` in
//! `csrc/kernel_v1.metal`, with events surfaced through `run_cosim`'s
//! UART drain. This module only provides the **driver** that consumes
//! `tx` actions from the input dispatcher and shifts the byte's bits
//! out onto the design's RX line over time.
//!
//! Frame: 1 start bit (0), 8 data bits LSB-first, 1 stop bit (1). Idle
//! line is high. One bit lasts `baud_div_edges` cosim edges
//! (= `clock_hz / baud_rate * edges_per_cycle`).

use crate::sim::input_stim::QueuedAction;
use crate::sim::models::ModelOverrides;

/// CPU-side state for one UART peripheral (RX-driver only).
pub struct UartRxDriver {
    /// Peripheral name (matches chipflow's `uart_<id>`).
    pub name: String,
    /// State-buffer position for the design's `rx` input pin.
    rx_pin_position: u32,
    /// Edges per UART bit (= clock_hz / baud * edges_per_cycle).
    baud_div_edges: u32,
    /// Whether a byte is currently being shifted out.
    tx_active: bool,
    /// Byte being transmitted.
    tx_data: u8,
    /// Edge counter within the active byte (counts from 0 at start bit).
    tx_counter: u32,
    /// Value currently driven on rx (idle = 1).
    rx_value: u8,
}

impl UartRxDriver {
    /// Build a UART RX driver. `rx_pin_position` is the GPU state-buffer
    /// position of the design's RX input. `baud_div_edges` is the number
    /// of cosim edges per UART bit.
    pub fn new(name: String, rx_pin_position: u32, baud_div_edges: u32) -> Self {
        assert!(
            baud_div_edges > 0,
            "uart {}: baud_div_edges must be > 0",
            name
        );
        Self {
            name,
            rx_pin_position,
            baud_div_edges,
            tx_active: false,
            tx_data: 0,
            tx_counter: 0,
            rx_value: 1, // idle high
        }
    }

    /// Apply a queued action. Currently handles `tx` only.
    pub fn apply_action(&mut self, action: &QueuedAction) {
        match action.event.as_str() {
            "tx" => {
                let byte = action
                    .payload
                    .as_u64()
                    .filter(|&v| v <= 0xFF)
                    .unwrap_or_else(|| {
                        panic!(
                            "uart `{}`: `tx` payload must be a u8 (0..=255), got {:?}",
                            self.name, action.payload
                        );
                    }) as u8;
                if self.tx_active {
                    clilog::warn!(
                        "uart `{}`: queued `tx` 0x{:02X} while previous byte still in flight; dropping new byte",
                        self.name,
                        byte
                    );
                } else {
                    self.tx_active = true;
                    self.tx_data = byte;
                    self.tx_counter = 0;
                }
            }
            other => {
                clilog::warn!(
                    "uart `{}`: unhandled event `{}` (payload {:?})",
                    self.name,
                    other,
                    action.payload
                );
            }
        }
    }

    /// Advance the RX driver one edge. Updates the line value driven on
    /// the design's RX pin. Should be called once per cosim edge.
    pub fn step_edge(&mut self) {
        if !self.tx_active {
            self.tx_counter = 0;
            self.rx_value = 1; // idle high
            return;
        }
        // Bit index: 0 = start, 1..=8 = data, 9 = stop, 10+ = post-byte idle.
        let bit = self.tx_counter / self.baud_div_edges;
        self.rx_value = match bit {
            0 => 0, // start bit
            b if (1..=8).contains(&b) => (self.tx_data >> (b - 1)) & 0x01,
            9 => 1, // stop bit
            _ => {
                self.tx_active = false;
                self.tx_counter = 0;
                1 // idle high
            }
        };
        self.tx_counter += 1;
    }

    /// State position this driver controls.
    pub fn driven_position(&self) -> u32 {
        self.rx_pin_position
    }

    /// Contribute the current rx line value to the shared override map.
    pub fn contribute_overrides(&self, overrides: &mut ModelOverrides) {
        overrides.insert(self.rx_pin_position, self.rx_value);
    }

    /// True if mid-byte transmission.
    pub fn is_active(&self) -> bool {
        self.tx_active
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn drain_until_idle(driver: &mut UartRxDriver, max_edges: u32) -> Vec<u8> {
        let mut out = Vec::new();
        for _ in 0..max_edges {
            driver.step_edge();
            out.push(driver.rx_value);
            if !driver.tx_active && !out.is_empty() && *out.last().unwrap() == 1 {
                // settle one extra edge after going idle
            }
        }
        out
    }

    /// Decode a captured rx-line trace as a UART receiver would: sample at
    /// the middle of each bit. The trace must begin with the start bit
    /// (rx already pulled low). Returns the recovered byte.
    fn decode_rx_trace(trace: &[u8], baud_div_edges: u32) -> u8 {
        let start_idx = trace
            .iter()
            .position(|&v| v == 0)
            .expect("no start bit found");
        let mut byte = 0u8;
        for bit in 0..8 {
            // Center of data bit `bit`: start_idx + (1 + bit) * baud_div + baud_div/2
            // (start bit takes [start_idx, start_idx+baud_div); data bits follow.)
            let center =
                start_idx + ((1 + bit) * baud_div_edges as usize) + (baud_div_edges as usize) / 2;
            byte |= (trace[center] & 1) << bit;
        }
        byte
    }

    #[test]
    fn idle_holds_high() {
        let mut d = UartRxDriver::new("uart_0".to_string(), 100, 4);
        for _ in 0..20 {
            d.step_edge();
            assert_eq!(d.rx_value, 1);
        }
        let mut ov = ModelOverrides::new();
        d.contribute_overrides(&mut ov);
        assert_eq!(ov[&100], 1);
    }

    #[test]
    fn tx_action_drives_serial_frame() {
        // baud_div_edges = 4 means each bit lasts 4 edges.
        let mut d = UartRxDriver::new("uart_0".to_string(), 100, 4);
        d.apply_action(&QueuedAction {
            event: "tx".to_string(),
            payload: json!(0x55), // 0b01010101
        });
        assert!(d.is_active());

        // Drain a full frame (1 start + 8 data + 1 stop = 10 bits = 40 edges, plus a few idle).
        let trace = drain_until_idle(&mut d, 50);
        assert_eq!(decode_rx_trace(&trace, 4), 0x55);
        assert!(!d.is_active());
        // After the byte, line returns to idle high.
        assert_eq!(d.rx_value, 1);
    }

    #[test]
    fn second_tx_after_first_completes() {
        let mut d = UartRxDriver::new("uart_0".to_string(), 100, 4);
        d.apply_action(&QueuedAction {
            event: "tx".to_string(),
            payload: json!(0xA3),
        });
        // Drain first byte
        let trace1 = drain_until_idle(&mut d, 50);
        assert_eq!(decode_rx_trace(&trace1, 4), 0xA3);
        assert!(!d.is_active());

        // Queue and drain second byte
        d.apply_action(&QueuedAction {
            event: "tx".to_string(),
            payload: json!(0x42),
        });
        let trace2 = drain_until_idle(&mut d, 50);
        assert_eq!(decode_rx_trace(&trace2, 4), 0x42);
    }

    #[test]
    fn second_tx_during_first_is_dropped() {
        let mut d = UartRxDriver::new("uart_0".to_string(), 100, 4);
        d.apply_action(&QueuedAction {
            event: "tx".to_string(),
            payload: json!(0xAA),
        });
        // Step one edge so we're mid-byte.
        d.step_edge();
        assert!(d.is_active());

        // Try to queue another byte mid-flight — should warn and drop.
        d.apply_action(&QueuedAction {
            event: "tx".to_string(),
            payload: json!(0x55),
        });
        // tx_data should still be the original byte.
        assert_eq!(d.tx_data, 0xAA);
    }

    #[test]
    #[should_panic(expected = "must be a u8")]
    fn invalid_tx_payload_panics() {
        let mut d = UartRxDriver::new("uart_0".to_string(), 100, 4);
        d.apply_action(&QueuedAction {
            event: "tx".to_string(),
            payload: json!(999),
        });
    }

    #[test]
    fn unknown_event_skips() {
        let mut d = UartRxDriver::new("uart_0".to_string(), 100, 4);
        d.apply_action(&QueuedAction {
            event: "wibble".to_string(),
            payload: json!("ignored"),
        });
        assert!(!d.is_active());
        assert_eq!(d.rx_value, 1);
    }
}
