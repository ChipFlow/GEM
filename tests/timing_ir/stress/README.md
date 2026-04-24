# Stress / robustness corpus

References to OpenSTA's own test tree, used as a stress / fuzz input set
for the `opensta-to-ir` converter and the timing-ir reader / diff tooling.

Run **nightly or pre-release**, not per-PR, because the corpus is large and
the run is exploratory rather than regression-oriented.

See [`docs/adr/0005-opensta-vendoring-and-corpus.md`](../../../docs/adr/0005-opensta-vendoring-and-corpus.md)
for the rationale behind the corpus split.

## Exit criterion (stress runs only)

- No crashes, no hangs, no malformed IR.
- Numerical agreement with OpenSTA is **not** required at this level; the
  stress corpus is for robustness, not correctness.
- Primary-corpus regressions are the correctness gate.

## Manifest format

`manifest.toml` lists paths into `vendor/opensta/` that the stress runner
iterates over. Paths are relative to the Jacquard repository root.

```toml
# tests/timing_ir/stress/manifest.toml

[[entries]]
path = "vendor/opensta/examples/example1.v"
description = "Single-corner Verilog example"

[[entries]]
path = "vendor/opensta/test/<specific-test-dir>/"
description = "..."
```

Each entry describes what the runner should feed to `opensta-to-ir`. The
runner enumerates matching files and records results in a run report;
failures do not break CI but are surfaced for investigation.

## Distilling a reproducer into the primary corpus

If a stress entry exposes a real bug:

1. **Verify the licence** of the specific file before copying — OpenSTA
   is GPL-3.0 but individual test inputs may have distinct licences.
2. Prefer writing a synthetic minimal reproducer to copying the original.
3. Add the reproducer under `tests/timing_ir/corpus/<name>/` and commit
   it as a primary corpus entry.

Do not move files from `vendor/opensta/` into the Jacquard tree; rely on
the submodule reference instead.

## Current manifest

`manifest.toml` is intentionally empty until the stress runner lands.
Populating it is part of the Phase 1 stress-testing work, not Phase 0 WS4.
