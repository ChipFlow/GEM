# Changelog

All notable changes to Jacquard are documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project will switch to [Semantic Versioning](https://semver.org/)
once the first numbered release ships. Until then, the `Unreleased`
section accumulates work in progress and the per-PR commit history is
the authoritative record.

## [Unreleased]

### Added

- **Multi-corner support** (WS2.4, commits `5822343` / `530bb36` /
  `59fde04`). `opensta-to-ir` accepts `--liberty NAME=PATH` to attach
  Liberty files to named PVT corners (bare paths still go to a
  "default" corner). The Tcl driver runs `define_corners` + per-scene
  `read_liberty -corner`, emits one record per (key, corner), and the
  Rust builder dedupes them into IR records carrying
  multi-`TimingValue` vectors. Consumer side: new `--timing-corner
  <NAME>` flag on `jacquard sim` selects the corner index from
  `ir.corners()` at load time; defaults to corner 0 when unset. Single-
  corner data continues to work unchanged.
- **`--timing-report <path.json>`** end-of-run structured timing report
  (commit `58a7a04`). Schema-versioned (`schema_version = "1.0.0"`,
  additive-only per ADR 0008's stability contract). Contains per-cycle
  violation list, per-word aggregates, top-10 worst-slack rankings per
  kind, run metadata, and aggregate stats. Sample fixture at
  `tests/timing_ir/sample_reports/two_violations.json`. Schema and
  consumer guide live in `src/timing_report.rs` and
  `docs/timing-violations.md`.
- **`--timing-summary`** human-readable text summary on stdout
  (commit `44e70a0`). Independent of `--timing-report`; either or both
  can be set, the report is built once and routed to the requested
  outputs.
- **Symbolic violation messages**: setup/hold violation lines now name
  the offending DFFs (`top/cpu/regs[7][bit 22] [word=42]`) instead of
  bare state-word indices (commit `0432d9a`). The `[word=N]` suffix is
  preserved for grep/tooling. New `WordSymbolMap` in `src/flatten.rs`
  built once at sim startup.
- **OpenSTA detection + version check** (WS-RH.1, commit `c9c393b`).
  When `--sdf` is requested but OpenSTA is missing, too old, or fails
  to probe, Jacquard now hard-fails with a remediation message instead
  of silently dropping timing data. Newer-than-tested versions warn but
  proceed. Tested against OpenSTA `3.1.0`
  (vendored at `vendor/opensta/`).
- **`--timing-report` violations cap**: per-cycle `violations` array is
  capped at 100k records by default (~8 MB JSON). Override with
  `--timing-report-max-violations <N>`; `0` disables the cap.
  `stats.violations_truncated` records overflow. Setup/hold totals
  and worst-slack rankings always reflect every observed event.
- **Pillar B Stages 1+2** (commits `c403cc8`, `6767c3e`): `ClockArrival`
  IR records emitted by `opensta-to-ir`; per-DFF capture-side clock
  arrival folded into setup/hold checks with `DFFConstraint::clock_arrival_ps`.
  Closes the main per-flop skew accuracy lever ahead of Phase 2.
- **`opensta-to-ir` golden-IR regression corpus** (WS4, commits
  `90558bb` / `6997096` / `9e25bc2`): seed entry `aigpdk_dff_chain` covers
  all four IR record types; runner + regen helper + CI hookup in place.

### Changed

- **ADR 0006 amended 2026-05-02** (commit `a4b1c81`): subprocess
  invocation of user-installed OpenSTA from the shipped runtime is now
  permitted, provided OpenSTA is not bundled and not linked into the
  Jacquard binary. Phase 3 (native Rust SDF→IR) is no longer release-
  gating. Background: GPL-3 § 5 ("aggregate") and FSF subprocess/IPC
  doctrine.
- **`process_events` API**: now takes a `ReportingCtx<'_>` bundle
  (`word_resolver` + `violation_observer`) instead of two separate
  optional callbacks. Default callers pass `ReportingCtx::default()`.
- **Hand-rolled SDF parser deleted** (WS3 phase 3.4, commit `9b2eb00`):
  `--sdf` now subprocesses `opensta-to-ir`, which produces timing IR
  consumed by `load_timing_from_ir`. The IR path (`--timing-ir`) is the
  canonical input.

### Deprecated / Removed

- ADR 0003 (OpenTimer as primary STA) **superseded** 2026-05-01
  (commit `d002bde`). OpenSTA-out-of-process is the sole STA path.
- `src/sdf_parser.rs` removed (WS3 phase 3.4); replaced by `opensta-to-ir`.

### Documentation

- New: `docs/release-process.md` — lightweight release procedure.
- Updated: `docs/why-jacquard.md` — output-interface section now
  describes shipped surface; deferred items moved to "Still on the
  wishlist".
- Updated: `docs/usage.md` — § Timing-Aware Simulation describes
  `--timing-ir` vs `--sdf` paths plus OpenSTA version requirements.
- Updated: `docs/timing-violations.md` — new § Text Summary and
  § Structured JSON Report.
- Plan files: `docs/plans/post-phase-0-roadmap.md` tracks current
  workstreams; Phase 0 / Phase 1 closed.

### Known limitations

- `--timing-report` and `--timing-summary` are wired only into the
  Metal sim path. The CUDA, HIP, and cosim paths detect violations on
  the GPU but do not yet route them through `process_events`. Tracked
  as a follow-up.
- The per-cycle `violations` array in `--timing-report` is capped at
  100k records by default (override via `--timing-report-max-violations`).
  Setup/hold totals + worst-slack always reflect every observed event;
  only the per-cycle list is bounded.
- `worst_slack` ranking is populated only from observed violation
  events. Closest-to-violation tracking on a run that *passed* timing
  needs GPU-side near-miss instrumentation; deferred.
- The `--timing-vcd` flag is wired only into the Metal sim path.
  CUDA / HIP detect violations on the GPU but don't currently route
  them through `process_events`; the JSON / text outputs only fire on
  Metal today.

[Unreleased]: https://github.com/ChipFlow/Jacquard/compare/HEAD
