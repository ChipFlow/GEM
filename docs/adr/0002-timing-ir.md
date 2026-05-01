# ADR 0002 — Timing intermediate representation

**Status:** Accepted.

## Context

Jacquard currently parses SDF directly in `src/sdf_parser.rs`, a hand-rolled parser that has accumulated reactive fixes (empty `()` delays, `(COND …)` pin specs, backslash escapes, edge-qualified timing checks, TIMINGCHECK stripping workarounds for OpenLane2 output). Each new production failure has been a one-off patch.

Commercial tool output adds dialect variation (Cadence, Synopsys extensions). Future parser paths (Liberty, SPEF) and future reference tools (OpenSTA, OpenTimer) each carry their own data models. A format-per-consumer coupling structure will continue to spread parser complexity into the simulator.

The project needs:

- A stable format we consume, with parser complexity isolated from simulator complexity.
- A format that can be diffed between producers (two parsers of the same file must agree).
- A format that supports multi-corner PVT values natively — commercial flows require this; single-corner shortcuts become retrofit pain.
- Preservation of vendor-specific annotations so information is not silently discarded.
- Fast consumption at sim startup (SDF parsing is currently on the critical path).

## Decision

Introduce a timing intermediate representation (timing IR) for SDF-equivalent annotation data.

- **Binary format:** FlatBuffers. Zero-copy reads, schema evolution, cross-language (Rust, C++ for OpenTimer adapter, Python for tooling).
- **Text sidecar:** JSON, produced via FlatBuffers' JSON round-trip, for CI diffs and human inspection.
- **Schema versioning:** explicit version field, compatible-evolution rules stated in schema comments. Breaking changes require a major version bump and migration notes.
- **Multi-corner native:** timing values are min / typ / max across a declared set of PVT corners. Single-corner designs are represented as a single-element corner set.
- **Vendor extension passthrough:** typed `VendorExtension` variants (`VendorCadence`, `VendorSynopsys`, `VendorOther`) carry unrecognised annotations as byte-typed blobs with source labels. Consumers opt in to understanding them; the IR never silently drops them.
- **Per-arc provenance:** each timing arc records source tool, source file, and origin category — `asserted` (from SDF / input), `computed` (derived by an STA tool), `defaulted` (fallback because no better value was available). Provenance is inspectable at consumer side.
- **Scope boundary:** the IR represents *timing annotation data only*. It is **not** a netlist representation, **not** a timing graph, **not** cell characterization. Attempts to extend it toward those adjacent formats are rejected — they become separate IRs if needed.

## Consequences

- A new schema and format to maintain. Scope discipline is load-bearing: if the IR creeps toward being a full STA framework, it becomes duplicate work with OpenSTA/OpenTimer.
- Parser complexity moves out of `src/sdf_parser.rs` (and its future rewrite, per ADR covering #3) into a focused converter crate. Unit-testable in isolation.
- A diff-based test corpus becomes natural: multiple converters on the same input must produce equivalent IR. This is the enforcement mechanism for ADR 0001's oracle pattern.
- Vendor extensions do not require Jacquard code changes — only converter updates.
- Startup parse cost drops: reading binary IR is near-instant. SDF-to-IR conversion becomes a one-time preprocessing step, not repeated per sim.
- Adopting FlatBuffers adds a code-generation step to the build, via `flatc`. Build hygiene (checked-in generated code, pinned `flatc` version, or a build-script integration) is required.
- If the IR is ever shared across other tooling beyond Jacquard, its stability contract tightens. Flagged in open questions on `timing-correctness.md`; not resolved here.

## Links

- `../project-scope.md` — validation and permissive-license constraints.
- `../timing-correctness.md` — requirement R1, principle P5 (multi-corner).
- ADR 0001 — OpenSTA oracle (IR is the diff format).
- ADR 0003 — **Superseded.** OpenTimer was the proposed in-process reference STA; spike Q2 fail moved Jacquard to OpenSTA-only via `opensta-to-ir`. See `../spikes/opentimer-sky130.md`.
- ADR 0004 — private PDK testing (IR enables portable fixtures without leaking PDK data).
