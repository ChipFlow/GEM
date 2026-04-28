//! `opensta-to-ir` — drive OpenSTA and emit Jacquard timing IR.
//!
//! Phase 0 WS2 implementation. See `docs/plans/ws2-opensta-to-ir.md` for the
//! design and `docs/adr/0006-sdf-preprocessing-model.md` for the rationale.
//!
//! This crate ships a binary (`opensta-to-ir`) and a library exposing the
//! pieces (dump-format parser, IR builder, OpenSTA subprocess driver) so
//! that integration tests, CI tooling, and future producers can reuse them
//! without invoking the binary.

pub mod builder;
pub mod dump;
