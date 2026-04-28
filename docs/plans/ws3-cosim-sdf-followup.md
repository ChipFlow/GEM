# Plan — WS3 follow-up: re-add cosim `--sdf` via `opensta-to-ir`

**Status:** Deferred. Tracked here so future work can pick it up.
**Predecessor:** WS3 phase 3.4 (deletes hand-rolled `src/sdf_parser.rs`).

## Background

Phase 3.4 deleted `src/sdf_parser.rs`. The `jacquard sim` subcommand kept
SDF input working (Phase 3.3 wired `--sdf` through
`setup::load_sdf_via_opensta_to_ir`, an internal subprocess wrapper that
calls the `opensta-to-ir` crate to convert SDF→IR). The `jacquard cosim`
subcommand chose **Option B** of the phase 3.4 handoff: drop `--sdf`
entirely rather than thread `--liberty` through. As a result, cosim now
only accepts pre-converted IR via `--timing-ir`.

## What was removed in 3.4

- `CosimArgs::sdf`, `sdf_corner`, `sdf_debug` CLI fields
  (`src/bin/jacquard.rs`).
- The `config.timing.sdf_file` / `sdf_corner` fallback path in
  `src/sim/cosim_metal.rs::run_cosim`.
- `TimingSimConfig::sdf_file` and `sdf_corner` JSON fields
  (`src/testbench.rs`).

## User-facing migration (current state)

The `tests/mcu_soc/` cosim flow that used to load SDF via the testbench
config now needs an explicit pre-conversion step:

```sh
# 1. Convert SDF → IR once.
opensta-to-ir \
    --liberty <path/to/liberty.lib> \
    --verilog tests/mcu_soc/data/6_final.v \
    --sdf tests/mcu_soc/data/6_final.sdf \
    --top <top_module> \
    -o tests/mcu_soc/data/6_final.jtir

# 2. Run cosim with the pre-converted IR.
cargo run -r --features metal --bin jacquard -- cosim \
    tests/mcu_soc/data/6_final.v \
    --config tests/mcu_soc/sim_config_sky130.json \
    --timing-ir tests/mcu_soc/data/6_final.jtir
```

`tests/mcu_soc/sim_config_sky130.json` no longer carries `sdf_file` /
`sdf_corner` (the fields would be silently ignored if added back; cosim
does not consume them).

## Option A — restore cosim `--sdf` ergonomics

When this becomes a priority, mirror the `jacquard sim` surface:

### Changes

1. **Add `--liberty` to `CosimArgs`** (`src/bin/jacquard.rs`). Plumb it
   through `DesignArgs::liberty` (currently hardcoded `None` in
   `cmd_cosim`). Also passthrough `--top-module` if not already.
2. **Add `--sdf`, `--sdf-corner`, `--sdf-debug` back to `CosimArgs`**.
   Make them mutually exclusive with `--timing-ir` (clap
   `conflicts_with = "timing_ir"`).
3. **Re-add `TimingSimConfig::sdf_file` / `sdf_corner` (optional)** —
   plus a new `liberty_file` field for the OpenSTA invocation. Update
   `tests/mcu_soc/sim_config_sky130.json` to use the new shape.
4. **Restore the cosim config-file fallback**: in
   `src/sim/cosim_metal.rs::run_cosim`, when timing is not yet enabled
   and the config provides SDF + Liberty paths, call
   `setup::load_sdf_via_opensta_to_ir`. Match the priority order:
   CLI > config.timing.* > nothing.
5. **Update `--timing-vcd` error message** to mention `--sdf` again.

### Out of scope for Option A

- Rebuilding a hand-rolled SDF parser. (See ADR 0006 — the durable
  replacement is the native Rust SDF→IR converter, tracked separately as
  Phase 3 in the original phasing.)
- Adding cosim-specific corner-selection beyond what `jacquard sim`
  already offers. The IR's `min/typ/max` triple is selected via
  `ir_corner0_max` (currently always `max`); changing that is a separate
  concern that affects both subcommands.

## Verification

After Option A lands:

```sh
cargo build --features metal
cargo test --lib
# Manual smoke test of the previous mcu_soc workflow:
cargo run -r --features metal --bin jacquard -- cosim \
    tests/mcu_soc/data/6_final.v \
    --config tests/mcu_soc/sim_config_sky130.json \
    --liberty <path>/sky130.lib \
    --sdf tests/mcu_soc/data/6_final.sdf
```

Should produce equivalent results to the pre-3.4 hand-rolled-parser
path within the IR's representational bounds (single-value
interconnect delays, `max` corner selection).

## Walk-back

If Option A is never picked up before first release, the existing IR-only
cosim surface is fine — contributors using SDF can pre-convert via
`opensta-to-ir` and pass the resulting `.jtir`. The follow-up exists as a
contributor-ergonomics improvement, not a correctness gap.
