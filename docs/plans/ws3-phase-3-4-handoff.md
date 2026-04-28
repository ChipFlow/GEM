# Handoff — WS3 Phase 3.4: delete SDF parser + migrate test fixtures

**Created:** 2026-04-28
**Resumes:** [`ws3-delete-sdf-parser.md`](ws3-delete-sdf-parser.md), Phase 3.4
**Predecessor commits in `main`:** `93bbdd3` (3.1), `d51c583` (3.2), `52229b2` (3.3)

## Where the work stands

- **Phases 3.1, 3.2, 3.3 landed** (all in `main`).
- **`src/sim/timing_ir_loader.rs`** owns `.jtir` buffers; `flatten.rs::load_timing_from_ir` consumes IR; `--timing-ir` CLI flag works; `--sdf` CLI now subprocesses `opensta-to-ir` via the new `setup::load_sdf_via_opensta_to_ir` helper.
- **`src/sdf_parser.rs` is still present** (1099 lines) but no longer called from the sim CLI path. The only remaining caller of the hand-rolled parser is **`src/sim/setup.rs::load_sdf`** (used by `src/sim/cosim_metal.rs:2333`) and **`src/flatten.rs::load_timing_from_sdf`** (used by 7+ tests in `flatten.rs::sdf_delay_tests`).

Tests at handoff: 111 root + 14 timing-ir + 16 opensta-to-ir = **141 green**.

## What Phase 3.4 must do

Delete the hand-rolled SDF parser, migrate or delete every consumer, and end with `src/sdf_parser.rs` gone and all tests passing.

Concrete steps (suggested order — earlier steps unblock later ones):

1. **Migrate `cosim_metal.rs::load_sdf` call** (`src/sim/cosim_metal.rs:2333`).
   - cosim's `--sdf` currently goes through `setup::load_sdf` → hand-rolled parser. Replace with `setup::load_sdf_via_opensta_to_ir`.
   - cosim does not currently take `--liberty`. Two options:
     - **Add `--liberty` to `CosimArgs`** and thread it through (preferred — matches sim's surface).
     - **Drop cosim `--sdf` support** and require pre-converted IR via `--timing-ir`. Smaller change but a CLI regression.
   - The cosim path also has a config-file fallback (`config.timing.sdf_file`). The Liberty equivalent (config field) may need adding too.
2. **Delete `setup::load_sdf`** (`src/sim/setup.rs:209-238`). After step 1 there are no callers.
3. **Delete `flatten.rs::load_timing_from_sdf`** (`src/flatten.rs:1764-2018`, ~250 lines). Currently called only by the sdf_delay_tests in the same file.
4. **Migrate or delete `flatten.rs::sdf_delay_tests`** (lines ~2047-2566). Each test currently builds an inline SDF string, parses it via `SdfFile::parse_str`, and calls `load_timing_from_sdf`. Two options per test:
   - **Migrate**: build an IR fixture programmatically via `timing_ir`'s FlatBuffers builders, call `load_timing_from_ir`, assert same outcome. A `build_test_ir` helper modelled on `crates/timing-ir/tests/diff.rs::build_ir_with_arcs` is a sensible refactor target.
   - **Delete**: if the test is redundant with the OpenSTA integration coverage in `crates/opensta-to-ir/tests/`, drop it.

   Tests to handle (search `crate::sdf_parser` in `src/flatten.rs`):
   - `test_accumulated_delay_analytical`
   - `test_sdf_delay_application`
   - `test_internal_and_gates_zero_delay`
   - `test_sdf_dff_constraints_loaded`
   - Plus the `use crate::sdf_parser::*;` blocks at the top of multiple `mod` definitions.
5. **Migrate `aig.rs` test imports** (`src/aig.rs` has `use crate::sdf_parser::{SdfCorner, SdfFile};` in test code). Same migrate-or-delete decision per test.
6. **Delete `src/sdf_parser.rs`** entirely.
7. **Remove `pub mod sdf_parser;`** from `src/lib.rs`.
8. **Run `cargo build && cargo test --lib && cargo clippy --lib --no-deps -- -D warnings`** after each step. The test count should remain steady (or shrink, if some sdf-tests are deleted as redundant).

## Critical context to know

- **Hierarchy separator changed**: SDF used `.`, IR uses `/`. `load_timing_from_ir` already handles the `/` path with the same prefix-mismatch heuristic. Test fixtures need to use `/` if they hand-build IR with hierarchical paths.
- **`load_sdf_via_opensta_to_ir` requires `--liberty`**: when the SDF path goes through this helper, `--liberty` is mandatory. cosim must comply.
- **The `flatbuffers::Vector` re-export** lives at `timing_ir::Vector` (added in 3.1) so test fixtures don't need their own `flatbuffers` dep.
- **`opensta-to-ir` is a path dep on root**: added in 3.3. Builders are at `opensta_to_ir::builder::build_ir`. `dump::DumpDocument` and `DumpRecord` enum are public — these may help build IR programmatically without going through OpenSTA.
- **`opensta_to_ir` crate has a `--from-dump` escape hatch**: useful for fixture-driven workflows. Tests could build a dump string in code, write it to a tempfile, run `opensta-to-ir --from-dump`, and consume the produced IR. But cleaner: skip the dump format entirely and build IR via `flatbuffers::FlatBufferBuilder` directly, the way `crates/timing-ir/tests/roundtrip.rs` does.
- **DFF constraint handling**: `load_timing_from_ir` picks the *first* matching `SetupHoldCheck` per cell (matches existing SDF semantics). If the IR has both Posedge and Negedge entries for the same cell (as AIGPDK DFF does), the order is dict-iteration-undefined in Tcl — the test might want to be tolerant or filter explicitly.

## Gotchas surfaced during 3.1–3.3

- **`flatbuffers::Vector` lifetime**: when iterating `ir.timing_arcs()`, each `arc.cell_instance()` returns `Option<&str>`. The `&str` is borrowed from the IR buffer; storing arcs in a `HashMap<&str, Vec<TimingArc<'_>>>` works but tied to the IR's lifetime. Don't try to outlive it.
- **`f64 → u64` ps conversion**: clamp non-negative before rounding. `ir_corner0_max` in `flatten.rs` does this; reuse it.
- **`set_cmd_units -time ps` does NOT affect `arc_delays`** — that's the bug we hit in 2.1-followup-B. Values come back in seconds; multiply by 1e12. Already handled in the Tcl driver.
- **OpenSTA's setup/hold edges point CLK→D**, opposite of SDF. The Tcl driver already swaps for the dump format, so the IR has the SDF convention (d_pin first). Fixture builders should follow the SDF convention.
- **`pre-commit` hooks are usually fine**; but in two cases this session, `cargo fmt --check` flagged a multi-line `assert_eq!` — fmt auto-fix resolves cleanly.

## Test fixture migration template

A minimal pattern for replacing an SDF-based test fixture with an IR builder:

```rust
fn build_test_ir(arcs: &[(&str, &str, &str, f64, f64)]) -> Vec<u8> {
    use timing_ir::*;
    let mut b = flatbuffers::FlatBufferBuilder::with_capacity(1024);

    // Default corner.
    let cn = b.create_string("default");
    let cp = b.create_string("tt");
    let corner = Corner::create(&mut b, &CornerArgs {
        name: Some(cn), process: Some(cp), voltage: 1.0, temperature: 25.0,
    });
    let corners_vec = b.create_vector(&[corner]);

    let prov_t = b.create_string("test");
    let prov_f = b.create_string("synthetic");
    let prov = Provenance::create(&mut b, &ProvenanceArgs {
        source_tool: Some(prov_t), source_file: Some(prov_f), origin: Origin::Asserted,
    });

    let mut arcs_offsets = Vec::new();
    for (cell, from_pin, to_pin, rise_ps, fall_ps) in arcs {
        let ci = b.create_string(cell);
        let dp = b.create_string(from_pin);
        let lp = b.create_string(to_pin);
        let cd = b.create_string("");
        let r = [TimingValue::new(0, *rise_ps, *rise_ps, *rise_ps)];
        let f = [TimingValue::new(0, *fall_ps, *fall_ps, *fall_ps)];
        let rv = b.create_vector(&r);
        let fv = b.create_vector(&f);
        arcs_offsets.push(TimingArc::create(&mut b, &TimingArcArgs {
            cell_instance: Some(ci), driver_pin: Some(dp), load_pin: Some(lp),
            rise_delay: Some(rv), fall_delay: Some(fv),
            condition: Some(cd), provenance: Some(prov),
        }));
    }
    let arcs_vec = b.create_vector(&arcs_offsets);

    // Empty vectors for SETUP_HOLD / INTERCONNECT / VENDOR_EXT.
    let sh_vec = b.create_vector::<flatbuffers::WIPOffset<SetupHoldCheck>>(&[]);
    let ic_vec = b.create_vector::<flatbuffers::WIPOffset<InterconnectDelay>>(&[]);
    let ve_vec = b.create_vector::<flatbuffers::WIPOffset<VendorExtension>>(&[]);

    let gt = b.create_string("test 0.0");
    let gv = b.create_string("0.0");
    let if_vec = b.create_vector::<flatbuffers::WIPOffset<&str>>(&[]);
    let sv = SchemaVersion::new(SCHEMA_MAJOR, SCHEMA_MINOR, SCHEMA_PATCH);

    let ir = TimingIR::create(&mut b, &TimingIRArgs {
        schema_version: Some(&sv),
        corners: Some(corners_vec),
        timing_arcs: Some(arcs_vec),
        interconnect_delays: Some(ic_vec),
        setup_hold_checks: Some(sh_vec),
        vendor_extensions: Some(ve_vec),
        generator_tool: Some(gt),
        generator_version: Some(gv),
        input_files: Some(if_vec),
    });
    finish_timing_ir_buffer(&mut b, ir);
    b.finished_data().to_vec()
}
```

For tests with setup/hold checks, extend with a `(cell, d_pin, clk_pin, edge, setup, hold)` parameter list; pattern follows `crates/opensta-to-ir/src/builder.rs::build_setup_hold`.

## Verification

After every step:

```sh
cargo build 2>&1 | tail
cargo test --lib 2>&1 | grep "test result"
cargo clippy --lib --no-deps -- -D warnings
```

Aim for **111 root-crate tests still passing** at the end, possibly minus deletions of redundant sdf-only tests. clippy/fmt clean.

## Walk-back

If 3.4 surfaces issues beyond expected churn (e.g., a test fixture exercises behaviour neither `opensta-to-ir` nor `load_timing_from_ir` covers, or hierarchy-separator drift breaks an existing design), the walk-back per ADR 0006 is: keep `src/sdf_parser.rs` alive but tagged `LEGACY — superseded by IR consumer; remove before first release`. Keep cosim's hand-rolled path as the legacy fallback; document the limitation. Pick up the work in the dedicated Phase 3 native-Rust SDF parser later.

## References

- [`ws3-delete-sdf-parser.md`](ws3-delete-sdf-parser.md) — the design plan this handoff executes.
- [`../adr/0006-sdf-preprocessing-model.md`](../adr/0006-sdf-preprocessing-model.md) — the SDF preprocessing model + cutover philosophy.
- [`../adr/0002-timing-ir.md`](../adr/0002-timing-ir.md) — IR contract.
- [`crates/timing-ir/tests/roundtrip.rs`](../../crates/timing-ir/tests/roundtrip.rs) — reference for FlatBuffers-builder patterns in tests.
- [`crates/opensta-to-ir/src/builder.rs`](../../crates/opensta-to-ir/src/builder.rs) — production IR builder; informs test-fixture builder shape.

---

**Ready for resume.** Open this doc in the new session, then proceed with step 1 (cosim migration). If the cosim `--liberty` plumbing turns out to be a much bigger lift than expected, fall back to step-1-as-CLI-regression (drop cosim's `--sdf`) and document the choice in the eventual commit message.
