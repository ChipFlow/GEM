# Release Process

Lightweight by design. Jacquard is a single-binary Rust project with
`vendor/` submodules; releases are git tags + a CHANGELOG entry. No
crates.io publication, no pre-built binaries (until/unless that
demand surfaces).

## When to release

Cut a release when:

- A user-visible feature or fix lands that you want consumers to be
  able to pin against.
- Schema or CLI changes happened (`--timing-report` JSON, CLI flags)
  and consumers need a stable reference point.
- A meaningful chunk of work in `docs/plans/` is closed (e.g. a Phase
  exits all criteria).

There is no fixed cadence.

## Versioning

[SemVer](https://semver.org/), starting once the first numbered
release ships. Pre-1.0 versions (`0.x.0`) carry the standard SemVer
caveat: minor bumps may include breaking changes; the public
contracts (`--timing-report` schema, IR layout) are documented in their
own ADRs and follow stricter rules.

Stable contracts (additive-only, breaking changes require a major
bump and a deprecation window):

- `--timing-report` JSON schema — `src/timing_report.rs::SCHEMA_VERSION`,
  governed by ADR 0008.
- Timing IR FlatBuffers schema — `crates/timing-ir/schemas/timing_ir.fbs`,
  governed by ADR 0002.

CLI flags, log message formats, and `--timing-summary` text output are
**not** stable parseable contracts; consumers that need to script
against them should use `--timing-report` JSON.

## Steps

For maintainers cutting a release:

1. **Verify CI is green** on `main` for all three GPU backends (Metal,
   CUDA, HIP) plus the unit-test, opensta-to-ir, and lint jobs. If any
   GPU runner is offline, hold the release until it's restored — see
   [`.github/workflows/ci.yml`](../.github/workflows/ci.yml). Do not
   ship a binary the CI hasn't built.

2. **Roll the `[Unreleased]` section in [`CHANGELOG.md`](../CHANGELOG.md)
   into a numbered version block.** Format follows
   [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Update
   the link references at the bottom of the file. Leave a fresh empty
   `[Unreleased]` section at the top.

3. **Bump `version` in [`Cargo.toml`](../Cargo.toml)** to match.
   `cargo build` to update `Cargo.lock`.

4. **Commit:** `chore: release v<X.Y.Z>` with the standard
   `Co-developed-by` trailer.

5. **Tag:** `git tag -a v<X.Y.Z> -m "v<X.Y.Z>"` then `git push --tags`.

6. **Create a GitHub release** from the tag. Body = the CHANGELOG
   section for that version. No artefacts attached unless someone has
   asked for them.

## What does NOT need to change at release time

- Submodule pins (unless deliberately bumping a vendored dep).
- The `vendor/opensta/` submodule pin is the version named in
  `crates/opensta-to-ir::MIN_TESTED_OPENSTA_VERSION`. If you bump the
  submodule, also bump the constant and the version-probe test — see
  WS-RH.1 in [`docs/plans/post-phase-0-roadmap.md`](plans/post-phase-0-roadmap.md).
- `LICENSE` (unless re-licensing).

## Pre-release checklist (one-time, before the first numbered release)

These items are tracked in [`docs/plans/post-phase-0-roadmap.md`](plans/post-phase-0-roadmap.md)
§ Release hardening; this section is the visible punch-list:

- [x] Phase 1 (ADR 0008 required outputs) closed.
- [x] WS-RH.1 (OpenSTA detection + version check) shipped.
- [ ] All three GPU CI jobs green on main (currently disabled awaiting
      runner restoration; see `.github/workflows/ci.yml`).
- [ ] Vendored-dep license posture confirmed (currently pending
      [gzz2000/eda-infra-rs#2](https://github.com/gzz2000/eda-infra-rs/issues/2)
      resolution; see "License posture" below).
- [ ] `Cargo.toml::license` field populated.
- [ ] `NOTICE` file enumerating vendored deps + their licenses.
- [ ] CUDA / HIP runtime violation routing through `process_events`
      (or document loudly that `--timing-report` is Metal-only at
      release time).
- [x] Bounded violations array (`--timing-report-max-violations`,
      default 100k).
- [ ] End-to-end test exercising `--timing-report` against a corpus
      design.

## License posture (open)

Top-level project intent is Apache-2.0 (see `LICENSE`). Vendored deps:

- `vendor/eda-infra-rs/` — Apache-2.0 at the workspace root. **Open
  question:** `sverilogparse` sub-crate declares `AGPL-3.0-only` in
  its `Cargo.toml`. If retained, that linkage would propagate AGPL to
  the Jacquard binary, contradicting `docs/project-scope.md`'s
  permissive-only rule. Awaiting upstream clarification at
  [gzz2000/eda-infra-rs#2](https://github.com/gzz2000/eda-infra-rs/issues/2).
  Fallback: replace with `OpenTimer/Parser-Verilog` or bind
  [slang](https://sv-lang.com/namespaceslang_1_1parsing.html) via FFI.
- `vendor/sky130_fd_sc_hd/` — Apache-2.0.
- `vendor/opensta/` — GPL-3 (subprocess only per ADR 0001 + ADR 0006
  § Amendment; never linked, never bundled).

The `NOTICE` file lands once the sverilogparse question resolves.

## Cross-references

- [`CHANGELOG.md`](../CHANGELOG.md) — release log.
- [`docs/adr/0008-structured-timing-output.md`](adr/0008-structured-timing-output.md)
  — `--timing-report` stability contract.
- [`docs/adr/0002-timing-ir.md`](adr/0002-timing-ir.md) — IR schema
  versioning.
- [`docs/adr/0006-sdf-preprocessing-model.md`](adr/0006-sdf-preprocessing-model.md)
  — OpenSTA bundling rules.
- [`docs/project-scope.md`](project-scope.md) — license posture
  contract.
