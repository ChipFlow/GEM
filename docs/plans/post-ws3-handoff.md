# Handoff — Post-WS3 (Phase 0 IR & Oracle)

**Created:** 2026-04-29
**Predecessor:** [`ws3-phase-3-4-handoff.md`](ws3-phase-3-4-handoff.md) (resolved)
**Phase 0 plan:** [`phase-0-ir-and-oracle.md`](phase-0-ir-and-oracle.md)
**Working tree:** clean. `main` is 22 commits ahead of `origin/main`; nothing pushed yet.

## Where the work stands

WS3 (delete hand-rolled SDF parser) is **complete**. The runtime now consumes timing
data exclusively through the Jacquard timing IR (`timing-ir` crate). `jacquard sim`
accepts both `--timing-ir` (canonical) and `--sdf` (interim wrapper that
subprocesses `opensta-to-ir`). `jacquard cosim` accepts `--timing-ir` only —
restoring `--sdf` ergonomics is documented as Option A in
[`ws3-cosim-sdf-followup.md`](ws3-cosim-sdf-followup.md).

End-to-end verification on sky130 mcu_soc passed: all 67 captured UART payloads
match the cxxrtl reference exactly (zero divergence). The raw event-count gap
between Jacquard and cxxrtl is fully explained by `--max-cycles` counting
half-cycles vs cxxrtl's `num_steps` counting full cycles.

## Phase 0 status snapshot

| WS | Title | Status | Notes |
|---|---|---|---|
| WS1 | IR schema | ✓ Complete | Round-trip test landed in `508baaf` |
| WS2.1 | OpenSTA subprocess + scaffold | ✓ Complete | `dc3db4a`, `3997e06`, `50b8600` |
| WS2.2 | Interconnect delays in IR | ✗ Open | Consumer in `flatten.rs:1853-1873` already wired; producer (`opensta-to-ir`) doesn't emit them yet |
| WS2.3 | Setup/hold checks | ✓ Complete | `8343b14` |
| WS2.4 | Multi-corner IR | ✗ Open | IR schema supports it; `flatten.rs::ir_corner0_max` picks corner 0 unconditionally; CLI flag for corner selection not yet defined |
| WS3 | Delete hand-rolled SDF parser | ✓ Complete | Today |
| WS4 | Diff harness + CI corpus | ⚠ Needs rescoping | Original WS4 compared WS2 (OpenSTA-derived IR) vs WS3 (Jacquard's hand-rolled-parser IR). With WS3's parser deleted, there's only one path; WS4 likely becomes a pure `opensta-to-ir` self-consistency / regression corpus. See **Open question 1** below. |
| WS5 | Parser-success assertions | ✗ Open | Sub-task: assertion in `opensta-to-ir` that non-trivial input produces non-zero arc count |

## Open follow-ups (priority-ordered, with size estimates)

### 1. WS2.2 — emit `interconnect_delays` from `opensta-to-ir` (medium)

The IR schema already has the `InterconnectDelay` table; `flatten.rs::load_timing_from_ir`
already reads `ir.interconnect_delays()` and threads them into `wire_delays_per_cell`.
Currently `opensta-to-ir` builds an empty interconnect vector
(`crates/opensta-to-ir/src/builder.rs:81`). The work is on the producer side.

OpenSTA's Tcl driver in `crates/opensta-to-ir/scripts/` would need to query OpenSTA
for per-net wire delays and emit them in the dump format. The IR builder then
populates `InterconnectDelayArgs` per dump record.

**Why this matters next**: post-PnR designs lose ~30-50 % of their delay budget
to wire delays. The mcu_soc verification today reported `0 wire delays` — the
IR-timed sim is currently effectively gate-only. Filling this in is the biggest
remaining accuracy lever before first release.

**Tests**: extend `crates/opensta-to-ir/tests/build_ir.rs` with an inv_chain
fixture that has SDF INTERCONNECT entries; assert non-zero `interconnect_delays`
in the produced IR. The sky130 inv_chain fixtures from
`tests/timing_test/inv_chain_pnr/inv_chain_test.sdf` already have the data.

### 2. Cosim Option A — restore `--sdf` ergonomics (small)

Spec is fully documented in [`ws3-cosim-sdf-followup.md`](ws3-cosim-sdf-followup.md).
The implementation is mechanical:

1. Add `--liberty`, `--sdf`, `--sdf-corner`, `--sdf-debug` to `CosimArgs` in
   `src/bin/jacquard.rs`.
2. Plumb them through `DesignArgs` (currently hardcoded `liberty: None`).
3. Re-add `sdf_file` / `sdf_corner` / `liberty_file` (optional) to
   `TimingSimConfig` in `src/testbench.rs`.
4. Re-add the config-file SDF fallback in `src/sim/cosim_metal.rs::run_cosim`,
   routed through `setup::load_sdf_via_opensta_to_ir`.

`tests/mcu_soc/sim_config_sky130.json` would get back the SDF fields it carried
pre-3.4, plus a new `liberty_file` pointing at the volare-installed sky130 lib.

**Test**: rerun the mcu_soc cosim flow without the manual `opensta-to-ir`
pre-step; result should match today's output byte-for-byte.

### 3. `--max-clock-cycles` CLI alias (small)

Footgun documented in [`ws3-cosim-sdf-followup.md`](ws3-cosim-sdf-followup.md):
`--max-cycles` actually counts `MultiClockScheduler` ticks (= half-cycles for a
single-domain design). cxxrtl users comparing budgets will trip over this.

Add `--max-clock-cycles N` that resolves internally to `N * schedule_len` ticks.
Either flag works; document the half-cycle one as legacy or rename it.

Touches `src/bin/jacquard.rs` (`SimArgs`, `CosimArgs`) and the cosim/sim setup
path that currently reads `args.max_cycles`.

### 4. WS5 — parser-success assertions (small-to-medium)

Per Phase 0 plan:
> Assertions in `opensta-to-ir` (WS2): non-zero IOPATHs / timing arcs resolved
> on non-trivial SDF input. Exit non-zero with a clear diagnostic when below
> threshold.

Currently `opensta-to-ir` will silently produce a 0-arc IR if Liberty/SDF
combination doesn't link. The mcu_soc run today produced 79896 arcs — a healthy
signal — but a smaller broken setup would just produce nothing without
complaining. Add a threshold check + fail-loud diagnostic. Threshold could be
`arcs >= 1` for the simplest case, or proportional to the cell count in the
linked netlist.

### 5. WS4 — corpus diff harness (open question, see below)

### 6. WS2.4 — multi-corner CLI flag (small, blocked on no current consumer)

The IR carries multiple corners; `flatten.rs::ir_corner0_max` always reads
corner 0. When a multi-corner IR appears in practice, add a `--timing-corner
<NAME>` CLI flag that resolves to the corresponding `Corner` index. Today's
single-corner producer (`opensta-to-ir` with default `--corner default`) makes
this low-priority until WS2.2 lands and we have producer-side diversity.

## Open questions

### 1. What does WS4 mean post-WS3?

The original WS4 ("Diff harness and CI integration") was scoped as comparing
two IR-producing paths: WS2's `opensta-to-ir` vs WS3's hand-rolled Jacquard
parser, on a corpus of designs. With WS3 deleting Jacquard's parser, WS4's
"diff" no longer has two parties.

Plausible reframings:
- **`opensta-to-ir` regression corpus**: a corpus of `(Liberty, Verilog, SDF) →
  expected-IR-fingerprint` triples. Catches regressions in OpenSTA upgrades or
  in the dump-format parser.
- **Cross-tool diff**: `opensta-to-ir` vs a future native Rust SDF→IR
  converter. This is naturally Phase 3 work, not Phase 0.
- **End-to-end behavioural diff**: cxxrtl vs Jacquard cosim on the corpus,
  comparing event traces. Today's mcu_soc verification is essentially a
  one-design instance of this; generalizing it is large but high-value.

Worth deciding before starting WS4 work. The phase plan
(`docs/plans/phase-0-ir-and-oracle.md` § WS4) should be updated either way.

### 2. mcu_soc/sky130 vs cxxrtl rate gap (~14 % residual)

After correcting for the half-cycle factor, Jacquard's per-byte rate on
mcu_soc is ~14 % slower than the cxxrtl reference's apparent rate. The
firmware is ~95 % blocked on SPI flash reads at this point in boot, so the
gap likely reflects flash-model latency differences. Not a phase-0 blocker;
worth a focused investigation when timing-arrival accuracy matters
(e.g. validating critical-path setup violations against an SDF-replay golden).

### 3. mcu_soc `events_reference.json` — coverage end-to-end?

Today's run captured 67 of 155 expected UART events at `--max-cycles 3000000`
(= 1.5 M clock cycles). The reference goes to ~3 M clock cycles' worth of
output. A full-coverage run takes `--max-cycles ≈ 6000000` here, ≈ 9 minutes
wall time. We confirmed all 67 captured payloads match. Worth re-running once
WS2.2 lands so the comparison covers both the reference's full sequence and
realistic wire-delay-aware timing.

## Critical context to know (carried forward)

These facts surfaced during WS2/WS3 and remain relevant:

- **Hierarchy separator**: SDF used `.`, IR uses `/` (OpenSTA's default
  `divider`). `flatten.rs::load_timing_from_ir` has a prefix-stripping
  heuristic (lines 1810-1848) that handled the mcu_soc wrapper-vs-`top`
  mismatch automatically.
- **`flatbuffers::Vector` is re-exported as `timing_ir::Vector`** so consumers
  don't need a direct flatbuffers dep — except for tests that build IR
  fixtures, which need `flatbuffers` as a `[dev-dependencies]` (added to
  jacquard's root `Cargo.toml` in `9b2eb00`).
- **`f64 → u64 ps` conversion**: clamp non-negative before rounding;
  `flatten.rs::ir_corner0_max` already does this.
- **OpenSTA returns delays in seconds** — the Tcl driver multiplies by 1e12.
- **`set_cmd_units -time ps` does NOT affect `arc_delays`** in OpenSTA Tcl —
  values come back in seconds regardless. (Bug hit during WS2.1-followup-B.)
- **OpenSTA's setup/hold edges point CLK→D** (opposite of SDF). The Tcl
  driver swaps for the dump format, so the IR has SDF convention (d_pin first).
  Fixture builders should follow.
- **Verilog parser limitations**: OpenSTA's parser rejects RTL-style assigns
  (`~`, concatenations). For mcu_soc, feed `top_synth.v` + `--top top` to
  `opensta-to-ir` — not `6_final.v` (the wrapper). See
  [`ws3-cosim-sdf-followup.md`](ws3-cosim-sdf-followup.md) for the full
  recipe.
- **`--max-cycles` half-cycle gotcha**: `MultiClockScheduler` produces
  `lcm/gcd` ticks per period (= 2 for single-domain). `--max-cycles N`
  simulates `N / schedule_len` clock cycles. Re-validate any
  cycle-budget number from cxxrtl/comparable tools.

## Verification

After every change in this thread:

```sh
cargo build 2>&1 | tail
cargo test --lib 2>&1 | grep "test result"   # expect 99 passed
cargo clippy --lib --no-deps -- -D warnings
cargo fmt --check
```

Optional smoke test on mcu_soc/sky130 (~5 minutes wall time, requires
`opensta-to-ir` built):

```sh
crates/opensta-to-ir/target/debug/opensta-to-ir \
    --liberty /Users/roberttaylor/.volare/volare/sky130/versions/c6d73a35f524070e85faff4a6a9eef49553ebc2b/sky130B/libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib \
    --verilog tests/mcu_soc/data/top_synth.v \
    --sdf tests/mcu_soc/data/6_final.sdf \
    --top top \
    --output tests/mcu_soc/data/6_final.jtir

target/release/jacquard cosim \
    tests/mcu_soc/data/6_final.v \
    --config tests/mcu_soc/sim_config_sky130.json \
    --top-module openframe_project_wrapper \
    --timing-ir tests/mcu_soc/data/6_final.jtir \
    --max-cycles 3000000
```

Expected: `Ticks simulated: 3000000` / 67 UART bytes captured / 67 of 155
events match (FAIL on count, no payload mismatches reported).

## References

- [`phase-0-ir-and-oracle.md`](phase-0-ir-and-oracle.md) — Phase 0 work breakdown
- [`ws3-cosim-sdf-followup.md`](ws3-cosim-sdf-followup.md) — cosim Option A spec + the wrapper/top + max-cycles gotchas
- [`ws3-delete-sdf-parser.md`](ws3-delete-sdf-parser.md) — original WS3 design (now executed)
- [`ws3-phase-3-4-handoff.md`](ws3-phase-3-4-handoff.md) — predecessor handoff (resolved)
- [`../adr/0002-timing-ir.md`](../adr/0002-timing-ir.md) — IR contract
- [`../adr/0006-sdf-preprocessing-model.md`](../adr/0006-sdf-preprocessing-model.md) — preprocessing-model + cutover philosophy

---

**Suggested first move next session**: pick one of the four small follow-ups
(Cosim Option A, `--max-clock-cycles` alias, WS5 assertions, or WS2.4 corner
flag) for a low-friction warm-up; or commit to WS2.2 (interconnect_delays) as
the next substantive piece.
