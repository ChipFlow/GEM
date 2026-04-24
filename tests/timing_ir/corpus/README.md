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
2. Run `opensta-to-ir` (WS2) to produce `expected.jtir` and `expected.json`.
3. Write `manifest.toml`. Keep tolerances tight; bump only with justification.
4. Verify locally with `cargo test -p timing-ir` (once WS4 CI test is wired up).
5. Commit all files together — design + expected IR + manifest — as one PR.

## Entries

Populated during Phase 0 once `opensta-to-ir` (WS2) can generate golden IR:

- `inv_chain_pnr` — simple inverter chain, post-P&R.  (pending WS2)
- `mcu_soc` subset — representative MCU SoC blocks.   (pending WS2)

Do not add entries without golden IR — half-populated entries hide CI regressions.
