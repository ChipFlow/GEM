# Handoff — pre-release work blocked on GPU runners

**Created:** 2026-05-02
**Working tree:** clean
**Branch:** main

## Goal & next-up

**Goal of this session:** Close out Phase 1 + WS2.4, then drive
through the lightweight pre-release punch list. Stopped because the
remaining items either need GPU runner validation or are demand-driven.

**Next session should pick up:** check whether the AMD runner
(self-hosted, AMD GPU) is online. If yes → wire HIP runtime violation
routing through `process_events` (see § HIP routing). If still down →
do the eda-infra-rs submodule bump if upstream landed the
sverilogparse fix; otherwise tour-of-the-repo or stop until runners.

**Verification command:**

```sh
git log --oneline -8
gh run list --workflow ci.yml --limit 5
ls -la docs/handoffs/   # should contain only this file
cargo test --lib 2>&1 | tail -3
# Expect: 166 passed; 0 failed
```

## Done this session

| Commit | Subject |
|---|---|
| `f82cd68` | Pre-release housekeeping (CHANGELOG, README, Cargo metadata, release-process.md, CLAUDE.md) |
| `c939704` | timing-report bounded violations + schema 1.1.0 + per-word incremental aggregation |
| `e0ee69c` | License posture closed — Apache-2.0 + NOTICE file |
| `5822343` | WS2.4 (consumer): `--timing-corner` flag + indexed IR consumption |
| `530bb36` | WS2.4 (builder): dedupe by record key, build per-corner `[TimingValue]` |
| `59fde04` | WS2.4 (producer): per-corner Tcl emission + `--liberty NAME=PATH` syntax |
| `d110174` | WS2.4 (test + docs): multi-corner integration test + roadmap closure |

## Open follow-ups (priority-ordered)

### 1. HIP / CUDA runtime violation routing (~1 day per backend, blocked on runners)

`sim_cuda` and `sim_hip` accept `timing_constraints: &Option<Vec<u32>>`
but ignore it — they call `simulate_v1_noninteractive_simple_scan`
which internally hard-codes `timing_constraints = nullptr` and
`event_buffer = nullptr` (`csrc/kernel_v1.hip.cpp:50-51`,
`kernel_v1.cu:31-32`). The "timed" entry points already exist in C++
(`simulate_v1_noninteractive_timed_hip`, `..._cuda`) but the Rust side
never calls them. Implementation:

1. Allocate a host-visible `EventBuffer` (model after `sim_metal`,
   `src/bin/jacquard.rs:739`).
2. Allocate `timing_constraints` as a `UVec<u32>`.
3. Call `simulate_v1_noninteractive_timed_hip` (or `..._cuda`) when
   `timing_constraints.is_some()`; otherwise keep the simple path.
4. After the (single-launch, batched) kernel returns, drain the buffer
   once via `process_events` with the report builder. Note: the buffer
   is shared across all cycles and capped at `MAX_EVENTS=1024`
   (`csrc/event_buffer.h:24`), so violation-storm runs lose events
   beyond 1024 — document loudly.
5. Mirror the `sim_metal` plumbing for `--timing-report` /
   `--timing-summary` / word resolver.

**Do HIP first** (AMD runner closer to online) and review both
backends together when CUDA runner is back up.

### 2. End-to-end `--timing-report` test (needs Metal runner)

All current coverage is unit tests + the hand-crafted sample fixture
at `tests/timing_ir/sample_reports/two_violations.json`. No CI test
runs a real sim with `--timing-report` and validates the JSON shape.
Wire one against `benchmarks/dataset/nvdla*` (or any small corpus
design) once `metal` CI job is live again.

### 3. eda-infra-rs submodule bump (no runner needed)

Upstream maintainer confirmed
[gzz2000/eda-infra-rs#2](https://github.com/gzz2000/eda-infra-rs/issues/2#issuecomment-4363789319)
that `sverilogparse/Cargo.toml`'s `license = "AGPL-3.0-only"` is a
typo. They haven't pushed the fix yet (verify with `git -C
vendor/eda-infra-rs fetch && git log origin/master`). Once they do:

- `git submodule update --remote vendor/eda-infra-rs`
- Verify `vendor/eda-infra-rs/sverilogparse/Cargo.toml` no longer says
  AGPL.
- Remove the corresponding footnote in `NOTICE` (the paragraph under
  `vendor/eda-infra-rs/` that starts "NOTE: at the current submodule
  pin...").
- Update `docs/release-process.md` § License posture.
- Tracked in `docs/release-process.md` "Pre-release checklist" as the
  last open item before tag.

### 4. GPU CI re-enable (user-driven, not Claude work)

User is rebuilding self-hosted runners:
- Apple silicon: `macos-latest-xlarge` runner unavailable since
  2026-05-01; user prioritising today.
- AMD: server "assembled and booted, needs installing" as of
  2026-05-02.
- NVIDIA: blocked on hardware (RAM sticks).

CI jobs `metal`, `cuda`, `hip-on-nvidia`, `mcu-soc-metal` are all
commented out in `.github/workflows/ci.yml`; lint + unit tests +
opensta-to-ir-tests run. Re-enable each when its runner is live.

### 5. Demand-driven Phase 0 carryover (not blocking release)

- **sky130-based corpus entries** (`inv_chain_pnr`, mcu_soc subset).
  Gated on a CI sky130-Liberty install strategy decision. Tracked in
  `docs/plans/post-phase-0-roadmap.md` § Phase 1 carryover.
- **Peripheral wiring for I²C / SPI** when a fuller mcu_soc fixture
  lands. Same plan section.

## Critical context

- **Story A vs Story B (open user decision):** see end of session 5
  pre-release review. Story A = Metal-only `v0.x.0` preview, Story B
  = production `v1.0.0` with all GPU backends + E2E test + fully-
  green CI. ADR 0006 amendment + WS-RH.1 already pulled toward Story
  B; the runner work in progress is the gating factor. No Cargo.toml
  version change has been made — it's still `0.1.0`. The license
  field is set to Apache-2.0.

- **OpenSTA Tcl scene API quirks** (folded into `tcl/dump_timing.tcl`
  comments, but worth re-stating since they're surprising):
  - `[::sta::scenes]` returns a Tcl list of *name strings*, not Scene
    object handles (SWIG typemap stringifies on output).
  - `[::sta::find_scene typ]` likewise returns the name string.
  - `[edge arc_delays $arc]` returns a flat list of length
    `2 * num_scenes`, layout `c0_min, c0_max, c1_min, c1_max, ...`.
    No `-corner` keyword form on `arc_delays`.
  - `[edge arc_delay $arc <name> <min|max>]` is the per-corner
    variant.
  - `set_cmd_scene <name>` + `vertex_worst_arrival_path` is the
    pattern for per-corner clock arrival.

- **Phase 0 closed except for demand-driven carryover.** Don't reopen
  Phase 0 work without a strong reason; the work is in
  `docs/plans/post-phase-0-roadmap.md` and beyond now.

- **`docs/release-process.md` is the canonical pre-release punch list**
  (not this handoff). Two checklists: the pre-release one (with state
  markers) and the release one (the steps to actually cut a tag).
  This handoff is the runner-blocked snapshot only; the doc is
  authoritative.

## Migration note

When this handoff resolves: HIP/CUDA routing notes go into
`docs/plans/post-phase-0-roadmap.md` (probably as a new "Pre-release
hardening WS-RH.2" or similar). The eda-infra-rs bump turns into one
small commit + remove the NOTICE footnote. The OpenSTA Tcl quirks
above already live in the Tcl driver — keep them there, don't
re-migrate. After all open items land or roll into roadmap entries,
delete this file (`git rm docs/handoffs/pre-release-runners-handoff.md`)
in the same commit as the migration.
