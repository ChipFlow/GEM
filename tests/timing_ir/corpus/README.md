# Primary timing-IR regression corpus

Jacquard-specific designs with golden timing-IR output, used as the primary
regression test for any change that affects timing-IR production or consumption.

Run on every CI execution.

See [`docs/adr/0005-opensta-vendoring-and-corpus.md`](../../../docs/adr/0005-opensta-vendoring-and-corpus.md)
for the rationale behind the corpus split (primary vs stress).

## Layout

Each design lives in its own subdirectory:

```
tests/timing_ir/corpus/
├── README.md                     # this file
└── <design_name>/
    ├── inputs/                   # design files (.v, .sdf, .lib, .spef, .sdc)
    ├── expected.jtir             # golden binary IR
    ├── expected.json             # golden JSON sidecar (flatc --json of expected.jtir)
    └── manifest.toml             # tolerance, notes, ownership
```

The manifest captures per-design testing policy:

```toml
# tests/timing_ir/corpus/<design_name>/manifest.toml

description = "One-line design purpose."
source = "tests/timing_test/inv_chain_pnr/"  # or external reference

[tolerance]
absolute_ps = 5          # absolute delay tolerance in picoseconds
relative = 0.02          # 2% relative tolerance on non-zero delays

[ownership]
# Who to contact when this test breaks. Use GitHub handles or team names.
primary = "@chipflow/timing"
```

## Adding a new entry

1. Create `tests/timing_ir/corpus/<name>/inputs/` and copy the design files.
2. Write `manifest.toml`. Required keys: `description`, `[opensta_to_ir]` (with `liberty`, `verilog`, `top`; optional `sdf`, `sdc`, `spef`), `[tolerance]` (`absolute_ps`, `relative`), `[ownership]` (`primary`). Paths under `[opensta_to_ir]` are resolved relative to the manifest file. Keep tolerances tight; bump only with justification.
3. Generate the golden via `scripts/regenerate-corpus-goldens.sh <name>`. Review the produced `expected.jtir` (binary) and `expected.json` (flatc sidecar) before committing.
4. Verify locally: `cd crates/opensta-to-ir && cargo test --test corpus`.
5. Commit all files together — design + expected IR + JSON sidecar + manifest — as one PR.

## Entries

- `aigpdk_dff_chain` — minimal aigpdk DFF + AND with back-annotated wire delay; covers ARC + SETUP_HOLD + CLOCK_ARRIVAL + INTERCONNECT in a self-contained fixture (no external PDK install needed).

Pending:

- `inv_chain_pnr` — sky130 post-P&R inverter chain, sourced from `tests/timing_test/inv_chain_pnr/`. Lands once a CI strategy for installing sky130 Liberty (likely volare) is decided.
- `mcu_soc` subset — representative MCU SoC blocks. Same blocker as `inv_chain_pnr`.

Do not add entries without golden IR — half-populated entries hide CI regressions.
