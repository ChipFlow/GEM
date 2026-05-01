//! Jacquard timing intermediate representation.
//!
//! SDF-equivalent timing annotation data in a schema-versioned, zero-copy
//! FlatBuffers format. See [`docs/adr/0002-timing-ir.md`] for design
//! rationale and [`docs/timing-correctness.md`] R1 for requirements.
//!
//! [`docs/adr/0002-timing-ir.md`]: https://github.com/chipflow/Jacquard/blob/main/docs/adr/0002-timing-ir.md
//! [`docs/timing-correctness.md`]: https://github.com/chipflow/Jacquard/blob/main/docs/timing-correctness.md
//!
//! The IR schema lives in `schemas/timing_ir.fbs` (source of truth).
//! Rust bindings are generated with `flatc` and checked in at
//! `src/timing_ir_generated.rs`; see the crate README for the
//! regeneration procedure.

// The generated file has its own allow-list of lints relevant to
// machine-generated code; we also suppress clippy entirely on it.
#[path = "timing_ir_generated.rs"]
#[allow(unused_imports, dead_code, clippy::all, non_camel_case_types)]
mod schema;

// Re-export everything in the `jacquard.timing_ir` namespace so consumers
// use a flat `timing_ir::TimingArc` path instead of
// `timing_ir::schema::jacquard::timing_ir::TimingArc`.
pub use schema::jacquard::timing_ir::*;

// Re-export the FlatBuffers Vector type so consumers can use it in
// function signatures without depending on the `flatbuffers` crate
// directly.
pub use flatbuffers::Vector;

/// Structured diff between two IR documents. Used by the `timing-ir-diff`
/// binary and by integration tests.
pub mod diff;

/// Schema version encoded in this crate.
///
/// - `MAJOR` — incremented for breaking wire-format changes.
/// - `MINOR` — incremented for backwards-compatible additions.
/// - `PATCH` — incremented for editorial fixes that do not change the wire format.
pub const SCHEMA_MAJOR: u16 = 0;
/// See [`SCHEMA_MAJOR`].
pub const SCHEMA_MINOR: u16 = 2;
/// See [`SCHEMA_MAJOR`].
pub const SCHEMA_PATCH: u16 = 0;

/// File identifier magic for timing IR binaries. Matches
/// `file_identifier "JTIR";` in `schemas/timing_ir.fbs`.
pub const FILE_IDENTIFIER: &str = "JTIR";
