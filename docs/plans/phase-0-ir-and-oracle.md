# Plan — Phase 0: Timing IR and OpenSTA oracle

**Status:** Proposed. Work may begin once the requirements doc and ADRs 0001–0002 are accepted. Phase 0 is independent of the OpenTimer spike (which runs in parallel and informs phase 1).

## Goal

Deliver the minimum viable infrastructure to enforce Jacquard's timing correctness contract:

1. A stable timing intermediate representation (IR) for SDF-equivalent annotations.
2. An OpenSTA-driven subprocess converter that produces IR from the same inputs Jacquard consumes.
3. A converter that produces IR from Jacquard's existing SDF parser output.
4. A CI diff harness that fails loud on converter disagreement.
5. Parser-success assertions on the SDF and Liberty paths.

After phase 0, Jacquard's timing pipeline has an enforced external reference. Silent failures (zero-match SDF, mis-scoped hierarchical prefixes, unexpected cell drops) surface as CI failures rather than correctness regressions detected in the field.

## Prerequisites

- Requirements doc (`../timing-correctness.md`) accepted.
- ADR 0001 (OpenSTA oracle) accepted.
- ADR 0002 (timing IR) accepted.
- A representative test design committed to the repo with inputs needed for both Jacquard and OpenSTA (`.v` + `.lib` + `.sdf` minimum; `.spef` if available). Candidate: `tests/timing_test/inv_chain_pnr` or the MCU SoC subset, whichever is smaller for first-pass iteration.
- OpenSTA available on developer machines and CI runners (installation documented).

## Work breakdown

### WS1 — IR schema

Produce the FlatBuffers schema (`schemas/timing_ir.fbs`) and generated Rust bindings.

Fields (minimum viable; extend only with written justification):

- `SchemaVersion { major, minor, patch }`.
- `Corner { name, process, voltage, temperature }`; IR holds a list of corners.
- `CornerValue { corner_index, min, typ, max }` for multi-corner floats.
- `TimingArc { driver_pin, load_pin, rise_delay: [CornerValue], fall_delay: [CornerValue], condition, provenance }`.
- `InterconnectDelay { net, from_pin, to_pin, delay: [CornerValue], provenance }`.
- `SetupHoldCheck { d_pin, clk_pin, edge, setup: [CornerValue], hold: [CornerValue], condition, provenance }`.
- `Provenance { source_tool, source_file, origin: Asserted | Computed | Defaulted }`.
- `VendorExtension { source_tool, kind: CadenceX | SynopsysY | Other, raw_bytes }` — untyped passthrough for unrecognised annotations.
- Root table `TimingIR { schema_version, corners, cell_instances, timing_arcs, interconnect_delays, setup_hold_checks, vendor_extensions }`.

Deliverables:

- `schemas/timing_ir.fbs` checked in.
- `build.rs` integration for code generation (or checked-in generated Rust with a `flatc` pin).
- A tiny `timing-ir` crate exposing read/write helpers.
- JSON round-trip via `flatc --json` verified in a unit test.

Scope guard: if you find yourself adding fields that represent computed timing graphs, cell electrical characterisation, or netlist structure, stop and re-read ADR 0002.

### WS2 — `opensta-to-ir` production tool

Per ADR 0006, `opensta-to-ir` is a shipped preprocessing tool, not merely a validation helper. Post-release it remains as an alternative preprocessing path for users who want OpenSTA-computed timing.

Deliverables:

- A Tcl script runnable by OpenSTA that loads Liberty + Verilog + SDF + (optionally) SPEF + SDC, then emits a machine-readable dump of timing annotations.
- A production-quality standalone Rust binary `opensta-to-ir` that parses OpenSTA's dump and emits timing IR (binary + JSON sidecar). Stable CLI, documented exit codes, clear diagnostics, man-page-worthy `--help`.
- Invocation wrapper handling OpenSTA subprocess lifecycle, stderr capture, exit-code checking, and error propagation up through `opensta-to-ir`'s own exit code.
- Assertion: if OpenSTA reports < expected-count cells, exit non-zero with a clear diagnostic.
- Ships as part of Jacquard's release artefacts (binary distributable, documented in user-facing docs).

### WS3 — Remove hand-rolled SDF parser; wire interim runtime hook

Per ADR 0006, Jacquard's hand-rolled SDF parser is deleted in Phase 0 rather than maintained through later phases. The runtime gains a new IR input path; the old SDF input path becomes an interim convenience wrapper over WS2.

Deliverables:

- Delete `src/sdf_parser.rs` and the SDF→Jacquard-internal-types code path. Remove all direct consumers.
- Add `jacquard sim --timing-ir <path>` as the canonical post-release timing input. Loads a pre-converted timing IR file, consumes it into the simulator's internal structures.
- Retarget the existing `--timing-sdf` / `--enable-timing` CLI behaviour: when SDF is provided, `jacquard sim` subprocesses `opensta-to-ir` internally to produce IR on the fly, then consumes it. Code site tagged **"INTERIM per ADR 0006; removed before first release."**
- Verify no remaining imports of the deleted module. Verify all existing tests that previously used the hand-rolled parser now pass via the interim hook or via checked-in IR fixtures.
- No runtime behaviour regression on Jacquard's timing-related regression suite; any design that currently works must still work after WS3.

### WS4 — Diff harness and CI integration

Deliverables:

- A test binary `timing-ir-diff` that reads two IR files and produces a structured diff (missing arcs, mismatched delays past tolerance, mismatched provenance).
- OpenSTA vendored as a git submodule at `vendor/opensta/`. Not built from Jacquard's build; present for CI version pinning and stress-corpus access (see ADR 0005).
- A primary regression corpus at `tests/timing_ir/corpus/` — Jacquard-specific designs with expected golden IR (initial entry: `inv_chain_pnr`; MCU SoC subset follows). Run on every CI execution.
- A stress corpus at `tests/timing_ir/stress/` — a manifest file listing paths into `vendor/opensta/<test-tree-subdir>/`. Run nightly or pre-release. Exit criterion: no crashes, no hangs, no malformed IR; numerical agreement with OpenSTA not required.
- A CI job that:
  1. Runs WS2 (OpenSTA) and WS3 (Jacquard) on each corpus design.
  2. Runs `timing-ir-diff` on the two outputs.
  3. Fails if diffs exceed declared tolerance or if either converter exits non-zero.

### WS5 — Parser-success assertions

Deliverables:

- Assertions in Jacquard's Liberty parsing code: non-zero cells parsed on non-empty input.
- Assertions in `opensta-to-ir` (WS2): non-zero IOPATHs / timing arcs resolved on non-trivial SDF input. Exit non-zero with a clear diagnostic when below threshold.
- A way to override thresholds for intentionally-empty test inputs (e.g., `--allow-empty-parse` flag, used only in specific tests).

(Original-plan assertions for Jacquard's SDF parser are obsolete — WS3 deletes the parser they were to guard.)

## Test plan

Tests live in `tests/timing_ir/`.

1. **Schema round-trip** (WS1). Construct a small IR in Rust, serialize to binary, deserialize, assert equality. Same for JSON.
2. **OpenSTA converter unit tests** (WS2). For a hand-crafted tiny design, invoke the converter, assert IR contents match expectation.
3. **Jacquard converter unit tests** (WS3). Same, on the same tiny design, through Jacquard's parser.
4. **Corpus diff** (WS4). For each design in the corpus, WS2 vs WS3 outputs diff clean within tolerance.
5. **Parser-success assertion tests** (WS5). Feed empty Liberty, empty SDF, and non-empty-but-no-match Liberty. Each should fail loud with a clear diagnostic, not proceed silently.

Tolerances:

- Delay values: ±5% or ±5 ps absolute floor, whichever is larger. Rationale: matches the existing `timing-validation.md` convention, at least until phase 2 refines tolerances with OpenTimer data.
- Missing arcs: zero tolerance. Every arc in OpenSTA's output must appear in Jacquard's or be explained by an annotation-type that Jacquard deliberately does not consume.

## Exit criteria

Phase 0 is complete when **all** of the following hold:

1. `schemas/timing_ir.fbs` is checked in and IR read/write works with a round-trip unit test.
2. `opensta-to-ir` binary exists, is production-quality (stable CLI, documented exit codes), and produces IR from the primary regression corpus.
3. `src/sdf_parser.rs` is deleted. All remaining imports are cleaned up. `jacquard sim --timing-ir <path>` is the canonical timing input path; `--timing-sdf` acts as an interim subprocess wrapper over `opensta-to-ir`, tagged for pre-release removal per ADR 0006.
4. OpenSTA is vendored as a git submodule at `vendor/opensta/` (per ADR 0005).
5. `timing-ir-diff` runs in CI on the primary corpus, passes cleanly, and fails loud on artificial regressions (verified by a mutation test).
6. Parser-success assertions in place on the Liberty parser and on `opensta-to-ir`. Failure-mode tests demonstrate they fire.
7. No existing Jacquard timing-related regression test regresses as a consequence of the WS3 cutover.
8. The section of `timing-validation.md` that sets the current "±5% tolerance" convention is updated to point to this plan's tolerance specification, or flagged for removal in phase 1.

## Out of scope (deferred to later phases)

- Native Rust SDF→IR converter. The hand-rolled parser is **removed** in Phase 0 WS3 (per ADR 0006); the native Rust replacement is Phase 3 work, landing before first release. Interim SDF input is handled via the `opensta-to-ir` subprocess wrapper.
- OpenTimer integration. Depends on the spike; tracked in `../spikes/opentimer-sky130.md` and its resulting phase-1 plan.
- Private PDK (GF130) test track. Tracked in ADR 0004; plumbing deferred to its own phase.
- SPEF IR. Separate from timing-annotation IR per ADR 0002.
- Runtime violation reporting improvements (R4 critical-path refinement JSON). Phase 1 or 2.

## Risks

- **Licensing verification on vendored OpenSTA corpus.** Per-file check needed before inclusion. May reduce corpus size if restrictive; acceptable.
- **FlatBuffers build integration friction.** If `build.rs` codegen causes cross-compilation or CI issues, fall back to checked-in generated code with a documented `flatc` version. Pick one approach and stick to it; flip-flopping is worse than either option.
- **Tolerance tuning.** Initial ±5% may prove too loose (hides bugs) or too tight (false positives from numerical differences). Plan to re-tune after first real-design data arrives.
- **WS3 cutover risk.** Deleting the hand-rolled SDF parser risks regressing designs that depend on behaviour it currently provides. Exit criterion 7 requires a clean regression run before WS3 is considered complete. If coverage gaps emerge, walk-back options per ADR 0006 apply: add dialect shims to `opensta-to-ir`, or postpone deletion to Phase 3.
- **OpenSTA dialect coverage.** OpenSTA may not accept every SDF dialect Jacquard's hand-rolled parser has been patched to handle. Such cases are tracked as either `opensta-to-ir` post-processing fixes or upstream OpenSTA contributions. Under no condition is the fix to reinstate the hand-rolled parser unless walk-back per ADR 0006 is formally triggered.

## Links

- `../project-scope.md`
- `../timing-correctness.md` — acceptance criteria this plan satisfies.
- `../adr/0001-opensta-as-oracle.md`
- `../adr/0002-timing-ir.md`
- `../spikes/opentimer-sky130.md` — runs in parallel; no dependency either way.
