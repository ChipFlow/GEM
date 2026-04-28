// SPDX-License-Identifier: Apache-2.0
//! Load and own a Jacquard timing-IR (.jtir) buffer.
//!
//! `TimingIrFile` reads a `.jtir` file into memory and exposes a
//! `timing_ir::TimingIR<'_>` view whose lifetime is tied to the buffer.
//! Callers keep the file alive while iterating the view.
//!
//! See `docs/plans/ws3-delete-sdf-parser.md` for the consumer cutover
//! plan.

use std::path::Path;

/// In-memory timing-IR document. Owns the underlying buffer; produces
/// `timing_ir::TimingIR` views via `view()`.
pub struct TimingIrFile {
    buf: Vec<u8>,
}

impl TimingIrFile {
    /// Read a `.jtir` file from disk.
    pub fn from_path(path: &Path) -> Result<Self, String> {
        let buf = std::fs::read(path).map_err(|e| format!("reading {}: {e}", path.display()))?;
        Self::from_bytes(buf)
    }

    /// Wrap an existing buffer. Verifies the FlatBuffers root parses.
    pub fn from_bytes(buf: Vec<u8>) -> Result<Self, String> {
        timing_ir::root_as_timing_ir(&buf).map_err(|e| format!("invalid timing IR: {e}"))?;
        Ok(Self { buf })
    }

    /// Borrow a typed view of the IR. Cheap (no copy).
    pub fn view(&self) -> timing_ir::TimingIR<'_> {
        // Already verified in `from_bytes`; safe to unwrap.
        timing_ir::root_as_timing_ir(&self.buf).expect("verified at construction")
    }
}
