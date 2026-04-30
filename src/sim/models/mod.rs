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
