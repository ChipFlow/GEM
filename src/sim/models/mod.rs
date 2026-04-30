// SPDX-FileCopyrightText: Copyright (c) 2026 ChipFlow
// SPDX-License-Identifier: Apache-2.0
//! CPU-side peripheral models for cosim.
//!
//! These models drain queued input actions from the [`InputDispatcher`]
//! and either drive design input bits (via the per-edge state_prep ops)
//! or observe design output bits to emit `wait`-able events.
//!
//! Ports of chipflow-lib's C++ peripheral models in
//! `chipflow/common/sim/models.h` and `chipflow/models/*.cc`. Same
//! protocol semantics so a chipflow `input.json` is portable to Jacquard.

pub mod gpio;
pub mod i2c;
pub mod spi;
pub mod uart;

use std::collections::HashMap;

use crate::sim::input_stim::QueuedAction;

/// Cumulative bit-overrides that peripheral models contribute. Each entry
/// is `state_position -> bit_value (0 or 1)`. The cosim main loop applies
/// these to the per-edge state_prep ops buffers so the GPU sees the
/// driven inputs at the next dispatch.
pub type ModelOverrides = HashMap<u32, u8>;

/// An output event emitted by a peripheral model. The cosim loop forwards
/// these to the [`crate::sim::input_stim::InputDispatcher`] so `wait`
/// commands can synchronize on them.
#[derive(Debug, Clone)]
pub struct EmittedEvent {
    pub peripheral: String,
    pub event: String,
    pub payload: serde_json::Value,
}

impl EmittedEvent {
    /// Build an event from a peripheral name + event-type str + payload.
    pub fn new(peripheral: &str, event: &str, payload: serde_json::Value) -> Self {
        Self {
            peripheral: peripheral.to_string(),
            event: event.to_string(),
            payload,
        }
    }
}

/// Common interface for all CPU-side peripheral models in cosim.
///
/// The cosim main loop drives this trait uniformly: drain pending
/// actions per name, advance one edge of state machine state, contribute
/// driven values to the override map, push any emitted events.
pub trait PeripheralModel {
    /// Peripheral name used by `input.json` (e.g. `gpio_0`, `uart_0`).
    fn name(&self) -> &str;

    /// State-buffer positions this model drives. Used at construction time
    /// to register placeholder BitOps in the per-edge ops buffers.
    fn driven_positions(&self) -> &[u32];

    /// Apply a queued action from the input dispatcher.
    fn apply_action(&mut self, action: &QueuedAction);

    /// Advance one cosim edge. Reads the design's output state if the
    /// model needs to (I²C/SPI bus observation), updates internal state,
    /// contributes current input drives to `overrides`, and pushes any
    /// emitted output events.
    ///
    /// The default implementation just calls `contribute_overrides` —
    /// suitable for models with no per-edge state machine (GPIO).
    /// Override for stateful models (UART RX, I²C, SPI).
    fn step_edge(
        &mut self,
        _output_state: &[u32],
        overrides: &mut ModelOverrides,
        _emitted: &mut Vec<EmittedEvent>,
    ) {
        self.contribute_overrides(overrides);
    }

    /// Write the model's currently-driven input values into the shared
    /// override map. Used both during normal stepping and at startup
    /// to seed initial idle values.
    fn contribute_overrides(&self, overrides: &mut ModelOverrides);

    /// True iff the model is mid-transmission and bit timing depends on
    /// per-edge granularity. The cosim loop uses this to decide whether
    /// to force single-edge batches.
    fn is_active(&self) -> bool {
        false
    }
}

// ── Shared helpers ──────────────────────────────────────────────────────────

/// Read one bit from a packed u32 state buffer at the given bit position.
/// Returns 0 if the position is out of range.
#[inline]
pub(crate) fn read_bit(state: &[u32], pos: u32) -> u8 {
    let word = (pos >> 5) as usize;
    let bit = pos & 31;
    if word < state.len() {
        ((state[word] >> bit) & 1) as u8
    } else {
        0
    }
}

/// Extract a u8 payload from a queued action. Panics with a clear
/// per-model error message if the payload isn't a u8 (0..=255).
pub(crate) fn payload_u8(action: &QueuedAction, model: &str) -> u8 {
    action
        .payload
        .as_u64()
        .filter(|&v| v <= 0xFF)
        .unwrap_or_else(|| {
            panic!(
                "{}: `{}` payload must be a u8 (0..=255), got {:?}",
                model, action.event, action.payload
            )
        }) as u8
}

/// Extract a bounded u32 payload (0..=`max`) from a queued action.
pub(crate) fn payload_u32_bounded(action: &QueuedAction, model: &str, max: u32) -> u32 {
    action
        .payload
        .as_u64()
        .filter(|&v| v <= max as u64)
        .unwrap_or_else(|| {
            panic!(
                "{}: `{}` payload must fit in 0..={}, got {:?}",
                model, action.event, max, action.payload
            )
        }) as u32
}

/// Extract a string payload from a queued action.
pub(crate) fn payload_str<'a>(action: &'a QueuedAction, model: &str) -> &'a str {
    action.payload.as_str().unwrap_or_else(|| {
        panic!(
            "{}: `{}` payload must be a string, got {:?}",
            model, action.event, action.payload
        )
    })
}

/// Log a warning for an unhandled action event. Used as the default
/// branch in models' `apply_action` `match` blocks.
#[inline]
pub(crate) fn warn_unhandled(model: &str, action: &QueuedAction) {
    clilog::warn!(
        "{}: unhandled event `{}` (payload {:?})",
        model,
        action.event,
        action.payload
    );
}

/// Edge classification for a single bool signal, given prior and current
/// values. Returned by [`EdgeDetector::update`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Edge {
    None,
    Rising,
    Falling,
}

/// Tracks the previous value of a single bool signal so per-step code
/// can ask "did this signal just rise/fall?". Initialized at the
/// signal's idle level.
pub(crate) struct EdgeDetector {
    prev: bool,
}

impl EdgeDetector {
    pub fn new(initial: bool) -> Self {
        Self { prev: initial }
    }

    /// Update with the current value, returning the edge type.
    pub fn update(&mut self, cur: bool) -> Edge {
        let edge = match (self.prev, cur) {
            (false, true) => Edge::Rising,
            (true, false) => Edge::Falling,
            _ => Edge::None,
        };
        self.prev = cur;
        edge
    }

    pub fn prev(&self) -> bool {
        self.prev
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_bit_packs_word_aligned() {
        let state = vec![0b0000_0101u32, 0b1000_0000u32];
        assert_eq!(read_bit(&state, 0), 1);
        assert_eq!(read_bit(&state, 1), 0);
        assert_eq!(read_bit(&state, 2), 1);
        assert_eq!(read_bit(&state, 31), 0);
        assert_eq!(read_bit(&state, 32), 0);
        assert_eq!(read_bit(&state, 39), 1); // bit 7 of word 1
                                             // out-of-range
        assert_eq!(read_bit(&state, 64), 0);
    }

    #[test]
    fn edge_detector_classifies_transitions() {
        let mut e = EdgeDetector::new(false);
        assert_eq!(e.update(false), Edge::None);
        assert_eq!(e.update(true), Edge::Rising);
        assert_eq!(e.update(true), Edge::None);
        assert_eq!(e.update(false), Edge::Falling);
        assert_eq!(e.prev(), false);
    }
}
