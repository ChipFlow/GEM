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

Detailed design and phased implementation: [`ws2-opensta-to-ir.md`](ws2-opensta-to-ir.md).

Deliverables:

- A Tcl script runnable by OpenSTA that loads Liberty + Verilog + SDF + (optionally) SPEF + SDC, then emits a machine-readable dump of timing annotations.
- A production-quality standalone Rust binary `opensta-to-ir` that parses OpenSTA's dump and emits timing IR (binary + JSON sidecar). Stable CLI, documented exit codes, clear diagnostics, man-page-worthy `--help`.
- Invocation wrapper handling OpenSTA subprocess lifecycle, stderr capture, exit-code checking, and error propagation up through `opensta-to-ir`'s own exit code.
- Assertion: if OpenSTA reports < expected-count cells, exit non-zero with a clear diagnostic.
- Ships as part of Jacquard's release artefacts (binary distributable, documented in user-facing docs).

#### WS2.4 — Multi-corner CLI flag (~3 days, open)

The IR schema (`crates/timing-ir/schemas/timing_ir.fbs`) supports per-corner `TimingValue` vectors today, but every record lands in the IR with a single `TimingValue` keyed at `corner_index = 0`. Both producer (`opensta-to-ir`) and consumer (`flatten.rs`) treat the world as single-corner. Multi-corner support has three pieces:

**Producer (Tcl + Rust binary):**
- `crates/opensta-to-ir/tcl/dump_timing.tcl`: replace single `read_liberty` + hardcoded `CORNER 0 default tt 1.0 25.0` with OpenSTA's `define_corners` + per-corner `read_liberty -corner $name`. The existing arc / setup-hold / wire / clock-arrival walks already key by `(cell, …)`; wrap each in a per-corner loop and call `[edge arc_delays $arc -corner $c]`. Verify the exact `-corner` syntax against the locally built OpenSTA before relying on it (similar to the `vertex_worst_arrival_path` probe done for clock arrival in commit `c403cc8`).
- `crates/opensta-to-ir/src/main.rs`: rework `--liberty PATH` to accept `--corner NAME=PATH[,V=…,T=…,P=…]` repeats. Validate at least one corner.
- `crates/opensta-to-ir/src/builder.rs`: today each ARC / SETUP_HOLD / INTERCONNECT / CLOCK_ARRIVAL line lands as one IR record with one `TimingValue`. Multi-corner emits multiple lines per `(cell, driver, load, corner_index)` from Tcl; the builder dedupes them into one IR record carrying a `[TimingValue]` vector. Mechanical.

**Consumer (jacquard root):**
- Add `--timing-corner <NAME>` to `SimArgs` / `CosimArgs` in `src/bin/jacquard.rs`; resolve to an index by walking `ir.corners()`.
- Replace `flatten.rs::ir_corner0_max(...)` (used in ~5 sites) with `ir_corner_max(idx)`. Thread the resolved index through `load_timing_from_ir`.

**Fixture:** sky130 ships multi-corner Liberty (`tt_025C_1v80`, `ss_-40C_1v62`, `ff_125C_1v95`) on disk via volare under `~/.volare/...` on dev machines that have run the cosim work. Wire two corners against the existing DFF / chain integration tests for a synthetic-but-real fixture; no external decision is needed before starting.

Land in this order: fixture probe (~hour, verifies the OpenSTA Tcl `-corner` flag works as expected) → producer (Tcl + binary + builder) → consumer (CLI + flatten plumbing) → integration test exercising both corners. The risk concentrates in the first hour; everything after that is mechanical.

### WS3 — Remove hand-rolled SDF parser; wire interim runtime hook

Per ADR 0006, Jacquard's hand-rolled SDF parser is deleted in Phase 0 rather than maintained through later phases. The runtime gains a new IR input path; the old SDF input path becomes an interim convenience wrapper over WS2.

Detailed design and phased implementation: [`ws3-delete-sdf-parser.md`](ws3-delete-sdf-parser.md).

Deliverables:

- Delete `src/sdf_parser.rs` and the SDF→Jacquard-internal-types code path. Remove all direct consumers.
- Add `jacquard sim --timing-ir <path>` as the canonical post-release timing input. Loads a pre-converted timing IR file, consumes it into the simulator's internal structures.
- Retarget the existing `--timing-sdf` / `--enable-timing` CLI behaviour: when SDF is provided, `jacquard sim` subprocesses `opensta-to-ir` internally to produce IR on the fly, then consumes it. Code site tagged **"INTERIM per ADR 0006; removed before first release."**
- Verify no remaining imports of the deleted module. Verify all existing tests that previously used the hand-rolled parser now pass via the interim hook or via checked-in IR fixtures.
- No runtime behaviour regression on Jacquard's timing-related regression suite; any design that currently works must still work after WS3.

### WS4 — Diff harness and CI integration

> **Reframed 2026-05-02; corpus + runner shipped 2026-05-02.** The original WS4 was framed as "WS2 vs WS3 IR diff" (OpenSTA-derived against Jacquard's hand-rolled SDF parser-derived). WS3 deleted that parser; the diff has only one side now. Three reframings were considered (see `post-cosim-models-handoff.md` § 1) — Option A (golden-IR regression corpus for `opensta-to-ir`) was chosen as the Phase 0 closure; Option B (end-to-end behavioural diff cxxrtl/CVC vs Jacquard cosim event traces) belongs in `timing-validation.md` as a Phase 1+ extension; Option C (cross-tool diff vs a future native Rust SDF→IR parser) is Phase 3 work per ADR 0006.

Deliverables:

- A test binary `timing-ir-diff` that reads two IR files and produces a structured diff (missing arcs, mismatched delays past tolerance, mismatched provenance). **Shipped** in `crates/timing-ir/src/bin/timing-ir-diff.rs`.
- OpenSTA vendored as a git submodule at `vendor/opensta/`. Not built from Jacquard's build at runtime; present for CI version pinning, the `opensta-to-ir` integration tests, and stress-corpus access (see ADR 0005). **Shipped.**
- A primary regression corpus at `tests/timing_ir/corpus/` — Jacquard-specific designs with checked-in `expected.jtir` (and a `expected.json` sidecar via `flatc --json` for human-readable diffs). **Shipped 2026-05-02** with the seed entry `aigpdk_dff_chain` (a minimal aigpdk DFF + AND with back-annotated wire delay; covers ARC + SETUP_HOLD + CLOCK_ARRIVAL + INTERCONNECT in a self-contained fixture). Sky130 entries (`inv_chain_pnr`, mcu_soc subset) remain to be added — the inputs exist under `tests/timing_test/`, but a CI strategy for installing the sky130 Liberty (likely volare) lands with them.
- A stress corpus at `tests/timing_ir/stress/` — a manifest file listing paths into `vendor/opensta/<test-tree-subdir>/`. Run nightly or pre-release. Exit criterion: no crashes, no hangs, no malformed IR; numerical agreement with OpenSTA not required. **Manifest format specced in `tests/timing_ir/stress/README.md`; entries pending.**
- A regression test that, for each design in the primary corpus, runs `opensta-to-ir` on its inputs and diffs against `expected.jtir` via `timing_ir::diff::diff_irs` with the per-design tolerance from `manifest.toml`. **Shipped** as `crates/opensta-to-ir/tests/corpus.rs::corpus_designs_match_golden_ir`. Skips gracefully when OpenSTA isn't built; fails loud with a structured diff when there's a mismatch.
- A `regenerate-goldens` helper for the OpenSTA-pin-bump workflow: bump submodule, run regen, review the diff, commit golden + submodule together. **Shipped** as `scripts/regenerate-corpus-goldens.sh`. Iterates `tests/timing_ir/corpus/*/manifest.toml`, runs `opensta-to-ir` per entry with the manifest-specified flags, refreshes both `expected.jtir` and the `expected.json` sidecar via `flatc --json`. Accepts entry names as positional args for targeted regen.
- A diff-machinery mutation test that perturbs a known-good IR and asserts `timing-ir-diff` flags it. **Shipped** in `crates/timing-ir/tests/diff.rs`: `delay_mismatch_past_tolerance_detected`, `delay_mismatch_within_tolerance_is_clean`, `arc_only_in_a_detected`, `arc_only_in_b_detected`.

**CI hookup landed 2026-05-02.** The `opensta-to-ir-tests` job in `.github/workflows/ci.yml` builds CUDD (cached), builds OpenSTA via `scripts/build-opensta.sh` (cached on the submodule SHA), and runs `cargo test` inside `crates/opensta-to-ir` — covering the corpus regression test, the CLI tests, and the OpenSTA-driven integration tests on every PR. `scripts/build-opensta.sh` was extended to honour a `CUDD_DIR` env var so the CI job can hand it the source-built CUDD location without bypassing the script.

What this catches: OpenSTA upstream regressions, dump-format / Tcl-driver regressions, accidental schema-breaking changes in `timing_ir.fbs`, builder bugs in `opensta-to-ir/src/builder.rs`, and the diff machinery itself (via the mutation tests that perturb an IR and assert `timing-ir-diff` flags the perturbation).

What this **doesn't** catch: behavioural divergence between Jacquard and a reference simulator. That's `timing-validation.md`'s job (CVC/iverilog event-trace comparison) — the mcu_soc/sky130 90/90 reference match is the current one-design instance, generalisable in Phase 1+.

### WS5 — Parser-success assertions

> **Done.** Both halves shipped pre-this-section being marked.

Deliverables (all live):

- Assertions in Jacquard's Liberty parsing code: non-zero cells parsed on non-empty input. Implemented as `TimingLibrary::parse` (`src/liberty_parser.rs:297-309`); rejects with a clear diagnostic naming the input byte count and pointing at the explicit override.
- Assertions in `opensta-to-ir` (WS2): non-zero IOPATHs / timing arcs resolved on non-trivial SDF input. Implemented as the `--min-arcs N` CLI flag (default 1) in the binary (`crates/opensta-to-ir/src/main.rs:71-77, :112-121`); exits with code `EXIT_MIN_ARCS_FAILED = 3` (see `:17`) and a diagnostic naming the produced count, the threshold, and the override flag.
- A way to override thresholds for intentionally-empty test inputs: `TimingLibrary::parse_unchecked` (`src/liberty_parser.rs:316`) for the Liberty path, `--allow-empty-parse` flag for the `opensta-to-ir` path.

Tests covering both halves: `liberty_parser::parse_rejects_library_input_with_zero_cells` and `parse_unchecked_accepts_zero_cell_library`; `opensta-to-ir::cli::cli_min_arcs_failure_exit_3` (covers both the failure and the `--allow-empty-parse` override).

(Original-plan assertions for Jacquard's SDF parser are obsolete — WS3 deleted the parser they were to guard.)

## Test plan

Tests live in `tests/timing_ir/`.

1. **Schema round-trip** (WS1). Construct a small IR in Rust, serialize to binary, deserialize, assert equality. Same for JSON.
2. **OpenSTA converter unit tests** (WS2). For a hand-crafted tiny design, invoke the converter, assert IR contents match expectation.
3. **Jacquard converter unit tests** (WS3). Same, on the same tiny design, through Jacquard's parser.
4. **Corpus diff** (WS4). For each design in the primary corpus, freshly produced `opensta-to-ir` output diffs clean against the checked-in golden `expected.jtir` within per-design tolerance.
5. **Parser-success assertion tests** (WS5). Feed empty Liberty, empty SDF, and non-empty-but-no-match Liberty. Each should fail loud with a clear diagnostic, not proceed silently.

Tolerances:

- Delay values: ±5% or ±5 ps absolute floor, whichever is larger. Rationale: matches the existing `timing-validation.md` convention; per-design overrides allowed via `manifest.toml`.
- Missing arcs: zero tolerance. Every arc in the golden IR must appear in the freshly produced one (and vice versa).

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
