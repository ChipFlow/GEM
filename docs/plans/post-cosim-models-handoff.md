# Handoff — Post-Cosim-Models (Phase 0 follow-ups)

> **STATUS: RESOLVED 2026-05-02.** All open follow-ups in this doc were
> picked up. Pillar B Stages 1+2 landed (commits `c403cc8`/`6767c3e`),
> WS4 was reframed and implemented end-to-end with CI hookup
> (`89f7aad`/`90558bb`/`6997096`/`9e25bc2`), WS5 was retroactively
> marked done (`6183bee`), and post-supersedure docs were reconciled
> across 9 files (`4aee1cc`). See [`post-phase-0-closure-handoff.md`](post-phase-0-closure-handoff.md)
> for the current state and the remaining WS2.4 + sky130-corpus + Phase 1
> trajectory. The body of this document is preserved for historical
> context.

**Created:** 2026-04-30
**Predecessor:** [`post-ws3-handoff.md`](post-ws3-handoff.md) (now resolved)
**Successor:** [`post-phase-0-closure-handoff.md`](post-phase-0-closure-handoff.md)
**Working tree:** clean. `main` matches `origin/main` (12 commits pushed this session).

## Goal & next-up

**Goal of this session:** Resolve the post-WS3 follow-ups — fix the
`--max-cycles` half-cycle misdiagnosis, switch cosim to edge-granularity,
land the input.json dispatcher + chipflow peripheral models, get
mcu_soc/sky130 cosim matching the chipflow reference 1:1, and close
WS2.2 (interconnect_delays in opensta-to-ir).

**Next session should pick up:** any of the small remaining items —
**WS2.4** (multi-corner CLI flag), **WS4 reframing decision** (still
the open question from post-ws3), or **peripheral wiring** for I²C/SPI
when a fuller mcu_soc fixture lands. The cosim/peripheral framework is
done; the gating issue for full chipflow-reference parity is netlist-
level (Jacquard's mcu_soc fixture is a stripped synthesis, missing
gpio_1 / uart_1 / i2c_0 / user_spi_*).

**Verification command:**
```sh
cargo test --lib && (cd crates/opensta-to-ir && cargo test) && \
target/release/jacquard cosim tests/mcu_soc/data/6_final.v \
    --config <see Verification section> \
    --top-module openframe_project_wrapper \
    --timing-ir tests/mcu_soc/data/6_final.jtir \
    --max-clock-edges 6000000
# Expect: 133 lib + 16 opensta-to-ir tests; cosim PASS 90/90 events.
```

## Done this session

12 commits, all pushed to `origin/main`:

| Commit | Subject | Notes |
|---|---|---|
| `2dd5889` | gitignore housekeeping | inherited from prior session |
| `46433e5` | docs: handoff cleanup — mark WS3 phase 3.4 resolved | inherited |
| `ca0c9d6` | docs: retract --max-cycles half-cycle misdiagnosis | corrected `91147dc`'s wrong claim |
| `46b5c28` | cosim: switch internal granularity to scheduler edges; rename `--max-cycles` → `--max-clock-edges` | -366 lines net (collapsed pair layer) |
| `4a1a989` | cosim: input-stimulus dispatcher (input.json, wait/action/stop) | new module `src/sim/input_stim.rs` |
| `aaa0efc` | cosim: port chipflow GPIO peripheral model | new module `src/sim/models/gpio.rs` |
| `44654fb` | cosim: port chipflow UART RX-driver model | `src/sim/models/uart.rs` |
| `5ad351b` | cosim: port chipflow I²C and SPI peripheral models | `src/sim/models/i2c.rs`, `spi.rs`; +EmittedEvent |
| `13562d8` | cosim: cut-down events_reference + skip force-single-edge when idle | `tests/mcu_soc/events_reference_cutdown.json` |
| `50fdf00` | mcu_soc: trim firmware to fit stripped netlist; cosim now matches reference 1:1 | new `tests/mcu_soc/software/main.c`; idle-init bugfix |
| `f413f58` | cosim: extract `PeripheralModel` trait + shared helpers across models | unified `Vec<Box<dyn PeripheralModel>>` |
| `67210c0` | opensta-to-ir: emit INTERCONNECT records (WS2.2) | both Tcl + Rust + tests |

## Phase 0 status snapshot

| WS | Title | Status | Notes |
|---|---|---|---|
| WS1 | IR schema | ✓ Complete | `508baaf` |
| WS2.1 | OpenSTA subprocess + scaffold | ✓ Complete | `dc3db4a`, `3997e06`, `50b8600` |
| WS2.2 | Interconnect delays in IR | **✓ Complete this session** | `67210c0`. Producer + consumer wired; tested end-to-end with hand-written SDF fixture. |
| WS2.3 | Setup/hold checks | ✓ Complete | `8343b14` |
| WS2.4 | Multi-corner IR | ✗ Open | Schema supports it; `flatten.rs::ir_corner0_max` reads corner 0; CLI flag for selection still TODO |
| WS3 | Delete hand-rolled SDF parser | ✓ Complete | prior session |
| WS4 | Diff harness + CI corpus | ⚠ Needs reframing | Same open question as before — see post-ws3-handoff.md "Open question 1" |
| WS5 | Parser-success assertions | ✗ Open | Add `--min-arcs` threshold + fail-loud diagnostic in opensta-to-ir |
| Cosim peripheral models | **✓ Complete this session** | All 4 chipflow models (GPIO, UART RX, I²C, SPI) ported to Rust + unit-tested via `PeripheralModel` trait |
| Input-stimulus dispatcher | **✓ Complete this session** | Loads chipflow input.json schema; supports wait/action/stop; idle-init handled |
| Edge-granularity refactor | **✓ Complete this session** | `--max-clock-edges` semantically aligned with chipflow's edge-counted timestamps |

## Cosim end-to-end validation

mcu_soc/sky130 with the trimmed firmware reproduces chipflow's
cut-down reference 1:1:

```
$ jacquard cosim --max-clock-edges 6000000 ...
....: nyaa~!
SoC type: CA7F100F
Flash ID: CA7CA7FF
Quad mode
GPIO0: 00000000
Echo:
Reference: 90 events  Actual: 90 events
PASS: all 90 event payloads match
SIMULATION: PASSED
```

Plus VCD-verified peripheral driving: GPIO `set "10101010"` → 0xAA on
`gpio_in[27..34]`; UART RX `tx 0x55` → 115200 baud serial frame on
`gpio_in[26]`; UART echo round-trip ('A' → echoed back).

133 jacquard root tests + 16 opensta-to-ir tests = 149 passing.

## Open follow-ups (priority-ordered)

### 1. WS4 reframing — resolved 2026-05-02

Decision: **Option A — golden-IR regression corpus for `opensta-to-ir`.** Cheap, mechanical, builds on the corpus + manifest infra already specced under `tests/timing_ir/{corpus,stress}/` and on the existing `timing-ir-diff` binary. § WS4 in `phase-0-ir-and-oracle.md` was updated to match.

Option B (end-to-end cxxrtl/CVC vs Jacquard cosim event-trace diff) was deferred to `timing-validation.md`'s scope as a Phase 1+ extension; the mcu_soc/sky130 90/90 reference match is the current one-design instance, generalisable later. Option C (cross-tool diff vs a native Rust SDF→IR parser) is Phase 3 per ADR 0006 — the parser doesn't exist yet.

What remains under WS4 (scheduled as Phase-0-carryover in `post-phase-0-roadmap.md` WS-P1.2): populate `tests/timing_ir/corpus/` with `inv_chain_pnr` and an mcu_soc subset, write a CI job that runs `opensta-to-ir` + `timing-ir-diff` per entry, and a `regenerate-goldens` helper for the OpenSTA-pin-bump workflow.

### 2. WS5 — parser-success assertions — resolved 2026-05-02

`opensta-to-ir` carries `--min-arcs N` (default 1) + `--allow-empty-parse`
override and exits with `EXIT_MIN_ARCS_FAILED = 3` and a clear
diagnostic naming the count, threshold, and override flag (see
`crates/opensta-to-ir/src/main.rs:17, :71-77, :112-121`). The Liberty
side is covered by `TimingLibrary::parse` rejecting zero-cell input
(`src/liberty_parser.rs:297-309`), with `parse_unchecked` as the
test-fixture override. Tests in `cli::cli_min_arcs_failure_exit_3`
and `liberty_parser::parse_rejects_library_input_with_zero_cells`.

### 3. WS2.4 — multi-corner CLI flag (small)

The IR carries multiple `Corner` entries; `flatten.rs::ir_corner0_max`
always reads corner 0. When a multi-corner IR appears in practice
(e.g. `opensta-to-ir --corner ss_-40C_1v62 --corner tt_25C_1v80
...`), add `--timing-corner <NAME>` CLI flag in `src/bin/jacquard.rs`
that resolves to the matching `Corner` index. Today's single-corner
producer (`opensta-to-ir` with default `--corner default`) makes
this low-priority until the producer can emit multiple corners.

### 4. Peripheral wiring for I²C / SPI (medium)

Models are ported and unit-tested; not wired to real ports. The Jacquard
mcu_soc fixture lacks i2c_0 / user_spi_* ports anyway, so wiring waits
on either:
- A fuller chipflow mcu_soc fixture that exposes these peripherals, OR
- A different test design

When the fixture lands, the wiring work needed:
1. `named_output_bits: HashMap<String, u32>` in `GpioMapping`
   (`src/sim/cosim_metal.rs:889`), mirror of `named_input_bits`.
2. TestbenchConfig schema for I²C/SPI peripheral port-name maps —
   each model's `I2cPins`/`SpiPins` need to be populated from config.
3. Multi-instance UART config: replace `uart: Option<UartConfig>` with
   `uarts: Vec<UartConfig>` (chipflow has uart_0 + uart_1).
4. The `step_edge` call in `cosim_metal.rs::run_cosim`'s main-loop
   model iter currently passes `&[]` for `output_state` — replace
   with a slice of the GPU output half once I²C/SPI need it.
5. `EmittedEvent` plumbing exists; the dispatcher already accepts
   forwarded events. Just needs models that actually generate events
   on real GPU output state.

### 5. Multi-clock event comparison (parked, design needed)

Current edge-granularity scheme works cleanly for single-domain (parity
of edge index → posedge/negedge). For multi-domain (e.g. 25 MHz +
12.5 MHz), the schedule has more edges per LCM period and the
parity convention doesn't generalize. Tagging events with
`(cycle_count, domain_index, edge_phase)` would generalize. Park
until a multi-domain test fixture exists.

### 6. Optional: upstream `chipflow software` build issue

When trying `pdm run chipflow sim run` from scratch in
`chipflow-examples/mcu_soc`, the build fails with model-UID mismatch
(`Unable to find a simulation model for 'com.chipflow.chipflow_lib.SPISignature'`).
The pre-built binary works. Worth filing upstream — likely a version
skew between `chipflow-examples` and the installed `chipflow-lib`.

## Critical context to know (carried forward)

These facts surfaced during this session and remain relevant:

- **chipflow `events_reference.json` `timestamp` field is in clock
  edges** (= 2 per full cycle for single-domain). chipflow's
  `tick()` lambda increments `++timestamp` twice per call. `num_steps`
  is in full cycles. Jacquard's `--max-clock-edges` is in edges, so
  Jacquard's timestamps and chipflow's are now numerically 1:1 —
  divide by 2 to get clock-cycle counts.
- **Trimmed mcu_soc firmware lives at
  `tests/mcu_soc/software/main.c`**. The original is preserved
  alongside as `main.c.original`. Rebuild via:
  ```sh
  cp tests/mcu_soc/software/main.c \
     ../ChipFlow/chipflow-examples/mcu_soc/design/software/main.c
  (cd ../ChipFlow/chipflow-examples/mcu_soc && pdm run chipflow software)
  cp ../ChipFlow/chipflow-examples/mcu_soc/build/software/software.bin \
     tests/mcu_soc/software.bin
  ```
- **The stripped jacquard mcu_soc fixture only has soc_uart_0,
  soc_gpio_0_gpio, soc_flash, soc_cpu_jtag** ports — the original
  firmware hangs on gpio_1 register read because that port isn't in
  the netlist. The trimmed firmware avoids gpio_1/uart_1/i2c/user_spi.
- **`force_single_edge` activates only when a peripheral model is
  mid-transmission** (`m.is_active()`). Empty input.json or post-action
  idle states fall back to batched mode (~7× faster). See
  `cosim_metal.rs:3083-3092`.
- **Peripheral models implement the `PeripheralModel` trait** in
  `src/sim/models/mod.rs`. Adding a new model: implement the trait,
  push a `Box<dyn PeripheralModel>` to the `models` vec in
  `cosim_metal.rs::run_cosim`, register driven_positions. The main
  loop iterates uniformly.
- **`build_edge_ops` accepts a `model_driven_positions: &[u32]`
  parameter** (`cosim_metal.rs:1430`). For each position, a
  placeholder BitOp is appended to every edge_buffer's ops list. The
  main loop's `update_model_driven_in_ops` patches values in-place
  each batch from the shared `ModelOverrides` map.
- **Initial idle sync**: `cosim_metal.rs:3000-3014` syncs each model's
  initial idle state into the per-edge ops buffers BEFORE the first
  GPU dispatch. Fixes a bug where UART RX line stayed at 0 (placeholder
  BitOp default) instead of idle high (model state), making the
  firmware echo phantom bytes.
- **OpenSTA `[$edge role]` returns `"wire"` for net interconnect
  edges**. Pin → Net via `[$pin net]`; net name via
  `get_full_name $net`. See `crates/opensta-to-ir/tcl/dump_timing.tcl:120-152`.

## Verification

```sh
# Build + lib test (full crate suite).
cargo build --features metal 2>&1 | tail
cargo test --lib 2>&1 | grep "test result"   # expect 133 passed

# opensta-to-ir crate tests (16 total: build_ir x3, cli x2, dump_parser x9, opensta_integration x3).
(cd crates/opensta-to-ir && cargo test 2>&1 | tail -20)

cargo clippy --workspace --no-deps -- -D warnings
cargo fmt --check
```

mcu_soc/sky130 cosim end-to-end (~5 minutes wall time, requires the
sky130 Liberty installed via volare and a pre-built `tests/mcu_soc/data/6_final.jtir`):

```sh
# Autonomous match — 90/90 events, no input.json.
python3 -c "
import json
c = json.load(open('tests/mcu_soc/sim_config_sky130.json'))
c['events_reference'] = 'tests/mcu_soc/events_reference_cutdown.json'
print(json.dumps(c, indent=2))
" > /tmp/cosim_baseline.json

target/release/jacquard cosim tests/mcu_soc/data/6_final.v \
    --config /tmp/cosim_baseline.json \
    --top-module openframe_project_wrapper \
    --timing-ir tests/mcu_soc/data/6_final.jtir \
    --max-clock-edges 6000000
# Expected: PASS: all 90 event payloads match.
```

Stimulus tests (small budgets):

```sh
# UART RX driver round-trip.
python3 -c "
import json
c = json.load(open('tests/mcu_soc/sim_config_sky130.json'))
c['input_commands'] = 'tests/mcu_soc/input_uart_echo.json'
c.pop('events_reference', None)
print(json.dumps(c, indent=2))
" > /tmp/cosim_echo.json

target/release/jacquard cosim tests/mcu_soc/data/6_final.v \
    --config /tmp/cosim_echo.json \
    --top-module openframe_project_wrapper \
    --timing-ir tests/mcu_soc/data/6_final.jtir \
    --max-clock-edges 5000000
# Expected: "UART output: ... Echo: A" (firmware echoed the driven 'A').
```

## References

- [`post-ws3-handoff.md`](post-ws3-handoff.md) — predecessor (resolved)
- [`phase-0-ir-and-oracle.md`](phase-0-ir-and-oracle.md) — Phase 0 work breakdown
- [`ws2-opensta-to-ir.md`](ws2-opensta-to-ir.md) — WS2 design plan
- [`../adr/0002-timing-ir.md`](../adr/0002-timing-ir.md) — IR contract
- [`../adr/0006-sdf-preprocessing-model.md`](../adr/0006-sdf-preprocessing-model.md) — preprocessing-model + cutover philosophy

## Files created / modified

**Created:**
- `src/sim/input_stim.rs`
- `src/sim/models/{mod,gpio,uart,i2c,spi}.rs`
- `tests/mcu_soc/software/{main.c,main.c.original}`
- `tests/mcu_soc/events_reference_cutdown.json`
- `tests/mcu_soc/input_cutdown.json`
- `tests/mcu_soc/input_uart_echo.json`
- `docs/plans/post-cosim-models-handoff.md` (this file)

**Modified (substantive):**
- `src/sim/cosim_metal.rs` — edge-granularity refactor, peripheral integration, model trait
- `src/sim/mod.rs` — added input_stim, models modules
- `src/sim/vcd_io.rs` — `max_cycles` → `max_clock_edges` rename
- `src/bin/jacquard.rs` — `--max-cycles` → `--max-clock-edges`
- `src/testbench.rs` — `input_commands` config field
- `crates/opensta-to-ir/tcl/dump_timing.tcl` — INTERCONNECT emission
- `crates/opensta-to-ir/src/builder.rs` — `build_interconnect`
- `crates/opensta-to-ir/tests/build_ir.rs` — interconnect builder test
- `crates/opensta-to-ir/tests/opensta_integration.rs` — chain+SDF e2e test
- `tests/mcu_soc/software.bin` — rebuilt from trimmed firmware
- `docs/plans/post-ws3-handoff.md` — half-cycle retraction
- `docs/plans/ws3-cosim-sdf-followup.md` — half-cycle retraction
- `docs/plans/ws3-phase-3-4-handoff.md` — annotated 91147dc framing as misdiagnosed

---

**Resume in a new session with:**
```
/resume_handoff docs/plans/post-cosim-models-handoff.md
```
