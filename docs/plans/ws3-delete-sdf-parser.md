# Plan — WS3: delete SDF parser, wire IR consumer + interim runtime hook

**Status:** Implemented — kept as historical record. Note: the "interim" / "pre-release-only" framing throughout this document describes the original ADR 0006 model. Per ADR 0006 § Amendment (2026-05-02), the runtime subprocess wrapper is now the shipping mechanism — Phase 3 (native Rust SDF→IR) is no longer release-gating. This document is preserved for the implementation phasing record; for current shipping intent see ADR 0006 § Amendment and `post-phase-0-roadmap.md` § Phase 3.

**Phase:** 0 (executes WS3 from `phase-0-ir-and-oracle.md`).
**Predecessors:** WS2 phases 2.1 + 2.3-minimum (delay arcs + setup/hold checks landed). Sufficient IR coverage for runtime cutover.
**ADRs:** 0002 (IR), 0006 (SDF preprocessing model + interim cutover; amended 2026-05-02).

## Goal

Delete `src/sdf_parser.rs` and migrate `src/flatten.rs`'s timing-loading to consume the timing IR directly. Wire `jacquard sim --timing-ir <PATH>` as the canonical input path, and (per ADR 0006) keep `--timing-sdf <PATH>` working pre-release as a contributor-ergonomics convenience that internally subprocesses `opensta-to-ir`.

End state:

- No hand-rolled SDF parsing in the Jacquard codebase.
- Runtime SDF input still works (via internal subprocess) until first release.
- `flatten.rs` consumes `timing_ir::TimingIR<'_>` for arc / setup / hold loading.
- All `flatten.rs` tests that previously hand-built SDF strings are migrated to build IR fixtures via the `timing-ir` crate's FlatBuffers builders.

## Surface analysis

`src/sdf_parser.rs` (1099 lines) defines `SdfFile`, `SdfDelay`, `SdfCorner`, `TimingCheckType`, and parses SDF text. Consumers:

- `src/flatten.rs` — `load_timing_from_sdf(...)` is the only non-test consumer; iterates `SdfFile.get_cell(path)`, uses `SdfDelay` for wire delays, `TimingCheckType::Setup/Hold` for check identification. ~200 lines of integration plus 7+ test fixtures that build SDF strings inline.
- `src/sim/setup.rs` — translates `--sdf-corner` CLI string into `SdfCorner` and calls `SdfFile::parse_file`.
- `src/aig.rs` — test imports only.
- `src/lib.rs` — module declaration only.

## Architecture changes

### New: `src/sim/timing_ir_loader.rs`

Thin module that owns the IR file buffer (so consumers can borrow `TimingIR<'_>` views from it):

```rust
pub struct TimingIrFile {
    buf: Vec<u8>,
}

impl TimingIrFile {
    pub fn from_path(path: &Path) -> Result<Self, ...> { ... }
    pub fn from_bytes(buf: Vec<u8>) -> Result<Self, ...> { ... }
    pub fn view(&self) -> Result<timing_ir::TimingIR<'_>, ...> {
        timing_ir::root_as_timing_ir(&self.buf)
    }
}
```

The `TimingIR` view holds a lifetime tied to the buffer. Callers keep the `TimingIrFile` alive while iterating the view.

### Modified: `src/flatten.rs`

Replace `load_timing_from_sdf` with `load_timing_from_ir`:

```rust
pub fn load_timing_from_ir(
    &mut self,
    aig: &AIG,
    netlistdb: &NetlistDB,
    ir: &timing_ir::TimingIR<'_>,
    clock_period_ps: u64,
    liberty_fallback: Option<&TimingLibrary>,
    debug: bool,
) { ... }
```

Logic translation table:

| Old (`SdfFile`) | New (`TimingIR<'_>`) |
|---|---|
| `sdf.get_cell(path)` | Index `ir.timing_arcs()` / `ir.setup_hold_checks()` by `cell_instance` (build a `HashMap<&str, _>` once). |
| `cell.iopaths` | Filter timing arcs by `cell_instance == path`. |
| `cell.timing_checks` | Filter setup/hold checks by `cell_instance == path`. |
| `SdfDelay { rise, fall, ... }` | `TimingArc.rise_delay()` / `.fall_delay()` (per-corner); take corner 0 max for now. |
| `TimingCheckType::Setup` / `::Hold` | `SetupHoldCheck.setup()` / `.hold()` per record. |
| `cell.interconnect_delays` | `ir.interconnect_delays()` — empty until WS2.2 lands; tolerate. |

The hierarchy-prefix detection (lines 1793-1820 of current flatten.rs) is independent of source format — same logic applies, just use IR's `cell_instance` strings instead of SDF's. Keep the heuristic.

### Modified: CLI surface (`src/bin/jacquard.rs`, `src/sim/setup.rs`)

- Add `--timing-ir <PATH>` flag that loads IR directly via `TimingIrFile::from_path`.
- Retarget `--timing-sdf <PATH>` (and the existing `--sdf-corner`) to: spawn `opensta-to-ir` as a subprocess, capture its IR output, call `load_timing_from_ir`. Mark the code site `INTERIM per ADR 0006`.
- The interim hook needs Liberty + Verilog paths to feed `opensta-to-ir`; the `jacquard sim` CLI already takes those, so plumb them through.
- Keep `--sdf-corner` for backward compat — the interim wrapper passes it as `--corner` to `opensta-to-ir`.

### Deletions

- `src/sdf_parser.rs` — entire file.
- `src/lib.rs` — `pub mod sdf_parser` line.
- `src/aig.rs` — `use crate::sdf_parser::{SdfCorner, SdfFile}` test imports; rewrite or delete the affected tests.
- `src/flatten.rs` — `use crate::sdf_parser::SdfFile`; rewrite test fixtures.

### Test migration strategy

Test fixtures in `flatten.rs` currently look like:

```rust
let sdf_content = r#"(DELAYFILE ... )"#;
let sdf = SdfFile::parse_str(sdf_content, SdfCorner::Typ).expect("...");
flat.load_timing_from_sdf(&aig, &netlistdb, &sdf, ...);
```

After cutover:

```rust
let ir_buf = build_test_ir(&TestIrSpec {
    arcs: vec![ /* (cell, from, to, rise_max, fall_max) */ ],
    setup_hold: vec![ /* (cell, d, clk, edge, setup, hold) */ ],
});
let ir = root_as_timing_ir(&ir_buf).unwrap();
flat.load_timing_from_ir(&aig, &netlistdb, &ir, ...);
```

A `build_test_ir` helper in `flatten.rs::tests` mirrors `build_ir_with_arcs` from `crates/timing-ir/tests/diff.rs`. Single source of truth would be nicer; for now duplicate it (deduplication is a future cleanup).

## Phased implementation

| Phase | Scope | Exit signal |
|---|---|---|
| 3.1 | Add `src/sim/timing_ir_loader.rs` and `flatten.rs::load_timing_from_ir` (parallel to `_from_sdf`). No CLI surface, no deletions. Unit-test the new function with a small synthetic IR. | New function compiles + passes unit test; existing `_from_sdf` path still works. |
| 3.2 | Add `jacquard sim --timing-ir <PATH>` CLI flag wired to `load_timing_from_ir`. End-to-end test: pre-generate IR via `opensta-to-ir`, run `jacquard sim --timing-ir`, compare against the existing `--timing-sdf` baseline. | A representative timing test (e.g., one of the existing `tests/timing_test/`) produces matching VCD output via both paths. |
| 3.3 | Retarget `--timing-sdf` to subprocess `opensta-to-ir` internally, then consume IR. Tag the code site `INTERIM per ADR 0006`. | Existing `--timing-sdf` regression tests pass through the new path. |
| 3.4 | Delete `src/sdf_parser.rs`. Migrate flatten.rs test fixtures from SDF strings to IR builders. Migrate aig.rs test imports. | All `cargo test --lib` tests pass; `src/sdf_parser.rs` is gone; the only `crate::sdf_parser::` reference is `git log`. |

Each phase exits cleanly. Phase 3.4 is the irreversible deletion — gates on phases 3.1-3.3 having green CI on the migration tests.

## Open questions

- **Hierarchy separator**: SDF uses `.`, OpenSTA's default divider is `/`. Our IR's `cell_instance` strings come from OpenSTA so use `/`. The flatten.rs hierarchy-prefix detection logic uses `.`. After cutover, the logic needs to use `/`. Verify by running on a hierarchical design (MCU SoC) before declaring 3.4 ready.
- **`--sdf-corner` semantics under IR**: today this picks one of `Min/Typ/Max` from SDF triples. The IR has min/typ/max per `TimingValue` already; the corner selection becomes "pick which of the three to use" applied per-arc rather than per-file. Document the mapping.
- **Default-corner consistency**: WS2 emits `default` as the corner name. Pre-existing Jacquard tests may not look at corner names — need to spot-check.
- **`liberty_fallback` semantics**: today, for cells absent from SDF, we fall back to Liberty-computed delays. Under IR, OpenSTA-computed values are already in the IR's arcs (as `Origin::Computed`). So `liberty_fallback` is potentially dead. Decide whether to drop it in 3.4 or keep as safety net.
- **Multi-corner (post-WS2.4)**: when WS2.4 lands, the IR will have multiple corners. flatten.rs currently picks one. Define the per-corner selection contract — explicit corner-name CLI flag, or default to a named corner.

## Risks

- **flatten.rs test churn**: 7+ test fixtures need rewrites. Each is a focused mechanical change but the bulk adds up. Mitigation: a `build_test_ir` helper standardizes the pattern.
- **Hidden-bug exposure**: the existing SDF parser had quirks. The IR parser has different ones (or none). Migration may surface bugs that were latent. Treat any test failure during 3.4 as a real bug, not "just adjust the test."
- **Hierarchy-separator regression**: if not caught in phase 3.2 testing (which tests on a single design), it could land in 3.4 and break a hierarchical design that wasn't previously regression-tested. Mitigation: include a hierarchical design in the 3.2 verification matrix.
- **Cutover timing**: WS3 lands while WS2.2 (interconnects) and WS2.4 (multi-corner) are still pending. flatten.rs's cutover assumes those will land later — test fixtures should not depend on interconnect delays or multi-corner behaviour for at-least-3.4 to pass.

## Walk-back

If 3.4 surfaces blocking issues, ADR 0006 already permits deferring deletion: keep `src/sdf_parser.rs` alive but tagged `LEGACY — superseded by IR consumer; remove before first release`, and ship preprocessing-only for the interim. The runtime SDF subprocess wrapper covers the contributor ergonomics. The native Rust SDF parser rewrite (Phase 3 in the original phasing) is the durable replacement.

## Non-goals

- A native Rust SDF parser. (Original ADR 0006 Phase 3; not part of WS3.)
- Validating SDF round-trip equivalence between the old parser and OpenSTA. (CI corpus test in WS4/WS2.5 covers this when fixtures exist.)
- Refactoring the broader `flatten.rs` structure beyond what migration requires.

## References

- `../adr/0002-timing-ir.md` — IR contract.
- `../adr/0006-sdf-preprocessing-model.md` — interim runtime subprocess + release-time cutover.
- `phase-0-ir-and-oracle.md` — WS3 row.
- `ws2-opensta-to-ir.md` — produces the IR this consumer reads.
- `crates/opensta-to-ir/` — subprocess target for the interim `--timing-sdf` hook.
- `crates/timing-ir/` — IR library + builders for test fixtures.

---

**Last updated:** 2026-04-28
