# Roadmap — Post-Phase-0 work scheduling

**Status:** Active. ADR 0008 accepted 2026-05-02. ADR 0007 still pending.

This document orders the work captured in those two ADRs alongside the in-flight tail of Phase 0. It is a **scheduling** doc, not a design doc — design lives in the ADRs and in `docs/timing-model-extensions.md` / `docs/why-jacquard.md`.

## Where things stand (2026-05-02)

- **Phase 0 (`phase-0-ir-and-oracle.md`)**: nearing close. WS1–WS3 + WS2.2 + WS4 + WS5 landed (corpus + runner + regen + CI hookup for WS4 in 2026-05-02 commits `90558bb`/`6997096`/`9e25bc2`). Open items: **WS2.4** (multi-corner CLI flag — see § WS2.4 in the phase-0 plan for scope), **sky130-based corpus entries** (gated on a CI sky130-Liberty install strategy), and **peripheral wiring** for I²C/SPI when a fuller mcu_soc fixture lands.
- **OpenTimer spike (`spikes/opentimer-sky130.md`)**: **resolved 2026-05-01 — Superseded.** Q1 (Liberty parse) passed cleanly on SKY130; Q2 (arrival computation) failed on the canonical OpenSTA-bundled GCD example after eight input-pipeline workarounds (bus ports, OpenROAD-emitted SPEF, modern TCL, tap cells). Per the spike's decision matrix, ADR 0003 is now Superseded (commit `d002bde`). **OpenSTA out-of-process is committed as Jacquard's sole STA path** — `opensta-to-ir` is the canonical preprocessor; no in-process reference STA is planned. A future ADR may revisit libreda-sta or an in-house walker if an in-process reference is wanted later, but not on this roadmap.
- **Pillar B Stages 1+2 (per `adr/0007`)**: **landed.** `ClockArrival` IR table + `opensta-to-ir` Tcl emission in commit `c403cc8`; `DFFConstraint.clock_arrival_ps` + skew-aware fold-in in `build_timing_constraint_buffer` in `6767c3e`. Closed Pillar B's main accuracy lever ahead of this roadmap's original Phase 2 schedule.
- **ADR 0006 amended 2026-05-02**: subprocess invocation of user-installed OpenSTA from the shipped runtime is now permitted (no linking, no bundling). Phase 3 (native Rust SDF→IR) is **no longer release-gating** — see § Phase 3 below. New release-hardening workstream **WS-RH.1** (OpenSTA detection + version check) is required before first release; see § Release hardening.
- **ADRs 0007 / 0008**: ADR 0008 accepted 2026-05-02; ADR 0007 still pending review.

## Phase boundaries

The phase numbering established by Phase 0 and ADR 0006 continues:

| Phase | Topic | Trigger |
|---|---|---|
| **0** | Timing IR + OpenSTA preprocessor | In flight, near close |
| **1** | Structured timing output (ADR 0008 required items) + Phase 0 carryover | ADR 0008 accepted ✓ |
| **2** | Timing model fidelity Pillar C Tier 1 + Pillar B Stage 3 if needed (ADR 0007) | Phase 1 lands; ADR 0007 accepted |
| **RH** | Release hardening (OpenSTA detection + version check, see § Release hardening) | WS-RH.1 shipped ✓; no other items currently scoped |
| **3** | Native Rust SDF→IR parser (ADR 0006) | **Deferred indefinitely** — no longer release-gating per amended ADR 0006. Picks up when bandwidth allows or commercial demand appears. |
| **4+** | Pillar A Stage 1 (static IDM); Pillar C Tier 2; ADR 0008 optional outputs | Demand-driven; not committed |

**Parked (require new ADR to revive):** in-process reference STA (ADR 0003 superseded), Pillar A Stage 2 (dynamic δ(T)), Pillar A Stage 3 (sub-cycle ticks), NoC-aware partitioning hints (Pillar C Tier 3).

## Phase 1 — Structured timing output and Phase 0 wrap-up

**Entry criteria:**
- ADR 0008 accepted.
- Phase 0 exit criteria met (per `phase-0-ir-and-oracle.md`).

OpenTimer integration was originally Phase 1's centrepiece (former WS-P1.1) but was retired when the spike Superseded ADR 0003. With OpenSTA-out-of-process as the sole STA path, Phase 1 is now anchored on user-visible output rather than a second STA tool.

**Workstreams (parallel where independent):**

### WS-P1.1 — Structured timing output (ADR 0008 required items)
The four required items from ADR 0008. Single workstream because they share infrastructure.

- **WS-P1.1.a — Symbolic violation messages.** ~1–2 days. `src/event_buffer.rs:305-338` plus a name-resolution helper from netlistdb. No new flags; format change documented in changelog.
- **WS-P1.1.b — `--timing-report <path.json>`.** ~3–5 days. End-of-run JSON document; schema versioned per ADR 0002 conventions; sample reports checked into corpus.
- **WS-P1.1.c — `--timing-summary` text output.** ~1 day, after WS-P1.1.b. Trivial wrapper over the JSON data.
- **WS-P1.1.d — Per-DFF worst-slack ranking.** ~1–2 days, folds into WS-P1.1.b. Top-N by closest-to-violation slack across the run.

Total ~2 weeks.

### WS-P1.2 — Phase 0 follow-ups (carryover)
Tail of Phase 0 work that didn't gate WS3 completion. Listed for completeness.

- WS2.4: multi-corner CLI flag in `opensta-to-ir`.
- WS4: corpus + runner + regen helper + CI hookup shipped 2026-05-02 with the seed entry `aigpdk_dff_chain` (covers all four IR record types). One follow-up: add sky130-based corpus entries (`inv_chain_pnr`, mcu_soc subset) once a CI sky130-Liberty install strategy is decided.
- Peripheral wiring for I²C/SPI when a fuller mcu_soc fixture lands.

(WS5 — parser-success assertions on the Liberty parser path and on `opensta-to-ir` — was already shipped; see `phase-0-ir-and-oracle.md` § WS5.)

These are not gated by any new ADR; pick them up as bandwidth allows.

**Exit criteria for Phase 1:**
- Symbolic violation messages live; old state-word-index format gone.
- `--timing-report` JSON shipping; sample golden reports in corpus.
- `--timing-summary` available.
- Worst-slack ranking included in both report and summary.
- `why-jacquard.md` updated to remove "what's missing" qualifications for items 1–4.

## Phase 2 — Timing model fidelity

**Entry criteria:**
- Phase 1 exit criteria met.
- ADR 0007 accepted.

Pillar B Stages 1 and 2 (per-DFF clock arrival in the IR + setup/hold fold-in) **landed early**, in commits `c403cc8` and `6767c3e` — directly on top of the OpenSTA-out-of-process producer rather than the OpenTimer integration originally planned. Phase 2 is therefore anchored on Pillar C Tier 1 (per-receiver wire delay), with Pillar B Stage 3 only if measurement justifies it.

**Workstreams (parallel where independent):**

### WS-P2.1 — Pillar C Tier 1: Per-receiver wire delay (ADR 0007)
Key wire delay per `(src_aigpin, dst_aigpin)` edge.

- **WS-P2.1.a — Edge-attributed wire delay.** Rewrite of `src/flatten.rs:1850-1872` to key wire delay per fanout; fold into source-side gate_delay per fanout target. ~3–5 days.
- **WS-P2.1.b — Rise/fall preservation.** Carry per-edge rise/fall through the consumer; honour both in `PackedDelay` accumulation. ~1–2 days, after WS-P2.1.a.
- **WS-P2.1.c — Validation.** Long-route corpus addition; tolerance ≤±3% on long-wire paths.

Total ~1 week.

### WS-P2.2 — Pillar B Stage 3: Bucketed per-DFF constraint packing (conditional)
Stages 1+2 collapsed all DFFs in a 32-bit state word to `min(setup), min(hold)` after folding the per-DFF clock arrival in. For most current designs the per-word collapse pessimism is small relative to clock period; for designs running close to the period boundary, splitting each word into clock-arrival buckets eliminates the collapse loss without disturbing the partitioner. See Stage 3 in `docs/timing-model-extensions.md` Part B.

Land **only if** Stage 1+2 measurement on a representative design shows the per-word collapse materially over-reports violations; otherwise defer indefinitely. Effort if pursued: ~3–5 days, touches `src/flatten.rs:1722-1761` and the kernel's constraint indexing.

### WS-P2.3 — Output adjustments for fidelity work
Small touch-ups to ensure Phase 1 outputs continue to work as model fidelity changes. JSON report fields, summary metrics, etc. Folded into WS-P2.1 / WS-P2.2 PRs as needed.

**Exit criteria for Phase 2:**
- Per-receiver wire delay landed; long-route paths reported within ≤±3% of CVC.
- `timing-model-extensions.md` Parts B and C marked **Implemented** with cross-references to landed code (Part B already updated post-Stage-1+2).
- `timing-validation.md` updated with per-pillar tolerances.
- No regression on existing corpus.

## Phase 3 — Native Rust SDF→IR parser

**Deferred indefinitely as of 2026-05-02 per amended ADR 0006.** No longer release-gating: shipped Jacquard binaries may subprocess user-installed OpenSTA via `opensta-to-ir`, provided OpenSTA is not bundled and not linked. The user-facing capability gap is "OpenSTA must be on PATH for `jacquard sim input.sdf`," surfaced by **WS-RH.1** below with a clear error message.

Reasons to revive:
- A downstream commercial integrator's legal team rejects subprocess-of-GPL-tool even with no bundling/linking.
- OpenSTA dialect coverage gaps appear that are easier to fix in our own parser than via `opensta-to-ir` post-processing.
- Bandwidth opens up and the team wants the zero-runtime-dependency story for its own ergonomics.

Effort estimate (unchanged from the original ADR 0006 framing): grammar-based (nom / pest), validated against OpenSTA on the WS4 corpus per ADR 0001. Probably 2–3 weeks of focused work. Not scheduled.

## Release hardening

Pre-first-release work that became necessary when ADR 0006 § Amendment relaxed the no-runtime-subprocess rule. These are blockers for first release, not for any specific Phase.

### WS-RH.1 — OpenSTA detection + version check

**Status:** **Shipped 2026-05-02 in commit `c9c393b`.** All scope items below are landed; this entry is preserved as a brief reference. Test coverage: 9 unit tests for the version parser + 6 integration tests for the locator across the missing / too-old / newer-than-tested / unparseable / failing-probe paths.

**Why:** With the shipped runtime now allowed to subprocess `opensta-to-ir`, a user invoking `jacquard sim input.sdf` on a machine without OpenSTA — or with an untested OpenSTA version — must get an actionable error rather than silent timing-data loss. Pre-WS-RH.1, missing OpenSTA only emitted a `warn!` and the simulation proceeded with no timing information loaded. That was acceptable during development but shipped as a UX bug.

**Scope:**

- **Promote missing-OpenSTA from warning to hard error** when `--sdf` is provided. Today's silent-fallback behaviour is fine for `--liberty`-only runs but wrong when SDF was explicitly requested. Error message must name the env var (`JACQUARD_OPENSTA_BIN`), the PATH lookup, and link to install instructions. ~0.5 day.
- **Pin a tested OpenSTA version range.** Record the version we test against in `vendor/opensta/` (already pinned via submodule per ADR 0005) and surface that as a `MIN_TESTED_OPENSTA_VERSION` / `MAX_TESTED_OPENSTA_VERSION` constant in `crates/opensta-to-ir/src/opensta.rs`. Need to choose a version-detection mechanism — OpenSTA's `-version` flag output format is the obvious target; check whether it's stable across the versions we care about. ~0.5 day.
- **Version probe at first invocation.** On first call to `find_opensta()` per process, run `<binary> -version`, parse the version, and:
  - If older than min-tested → hard error with remediation message ("rebuild via `scripts/build-opensta.sh` or upgrade your system OpenSTA").
  - If newer than max-tested → warn but proceed ("untested OpenSTA version vN.M; please report any timing discrepancies").
  - Cache the result for the rest of the process. ~1 day.
- **Document the dependency in `docs/usage.md`.** Single section: required tooling, install paths, version range, what `jacquard sim` does and doesn't need OpenSTA for. ~0.5 day.
- **Test coverage:** unit tests for the version-string parser (with sample `-version` outputs from the pinned version and a synthetic too-old version); an integration test that points `JACQUARD_OPENSTA_BIN` at a stub script and confirms the error path. ~0.5 day.
- **Stale-framing cleanup** (folded in here per 2026-05-02 decision rather than spun out separately):
  - Reword `INTERIM per ADR 0006` / `Pre-release only` markers in source: `src/sim/setup.rs:176,228,286`, `src/bin/jacquard.rs:187`, `src/sim/cosim_metal.rs:2053`, `src/testbench.rs:255-257`. Replace with "subprocess wrapper per ADR 0006 § Amendment" or similar — these paths are no longer interim.
  - Update `docs/plans/phase-0-ir-and-oracle.md` lines 152, 161, 172 — drop "tagged for pre-release removal" framing; the subprocess wrapper is now the shipping mechanism, not a temporary bridge.
  - Audit `docs/plans/ws3-delete-sdf-parser.md` for the same stale framing and update.
  - ~0.5 day total for the cleanup.

**Total:** ~3.5 days. Single PR, owned by whoever picks up release prep.

**Open question:** does OpenSTA emit a stable `-version` string, or do we need to scrape `git describe` from a build-time-recorded commit? If `-version` is unreliable, fall back to recording the submodule commit at `crates/opensta-to-ir` build time and comparing — this is cheaper than version-string sniffing and avoids the "user has a custom build" problem.

## Phase 4+ — Demand-driven

Items below land when (a) a real use case appears that demands them, and (b) bandwidth is available. Each gets its own ADR amendment / new ADR before scheduling, since the cost is non-trivial.

### Pillar A Stage 1 (static IDM)
Cheapest δ(T) entry point. Lands only after Pillars B and C confirm the wire/skew baseline is correct — characterisation work done before that risks chasing wire-delay error masquerading as δ(T) error.

Effort: 1–2 day spike to validate value, then ~1 week implementation, plus per-cell SPICE characterisation effort (long-pole risk).

### Pillar C Tier 2 (inter-partition wire delay)
Required for many-core/NoC designs at advanced processes. Lands when a representative such design appears in the test corpus and Tier 1 measurement shows it is needed.

Effort: ~2–3 weeks, touches `src/sim/cosim_metal.rs` shuffle pipeline.

### ADR 0008 optional outputs
Items 5–7 from ADR 0008: arrival histograms, STA cross-reference, path-back-trace. Demand-driven prioritisation.

### Pillar C Tier 3 (NoC-aware partitioning hints)
Optional optimisation that makes Tier 2 cheap on tile-decomposed designs. Lands only if Tier 2 lands and partitioning quality on tile designs proves measurably suboptimal.

## Risks and walk-back

- **Pillar measurement shows smaller-than-expected gain.** Each pillar's later stages are deferred or abandoned per ADR 0007's walk-back clause. Pillar B Stage 3 is explicitly conditional on this signal.
- **JSON report schema design wastes time in bikeshedding.** Mitigation: ship v1 quickly, additive-only changes thereafter, breaking changes require explicit ADR-level decision.
- **OpenSTA upstream regressions.** With OpenSTA as the sole STA path, an upstream behaviour change reaches us through `opensta-to-ir`'s output. Mitigation: pin OpenSTA in CI (per ADR 0001) and rely on the regression corpus to surface drift.
- **CRPR pessimism on tight designs.** Stage 1+2 fold-in treats launch=0; a design with very heterogeneous launch arrivals will see pessimism on paths whose launch DFF also has a long clock path. Stage 3 is the lever if this matters; otherwise live with it.

## Cross-references

- `../adr/0007-timing-model-fidelity-roadmap.md` — Pillar definitions for Phase 2.
- `../adr/0008-structured-timing-output.md` — Output items for Phase 1.
- `../adr/0001-opensta-as-oracle.md` — OpenSTA out-of-process commitment (post-ADR-0003 supersedure).
- `../adr/0003-opentimer-primary-sta.md` — **Superseded.** Spike fail outcome documented in `../spikes/opentimer-sky130.md`.
- `../adr/0006-sdf-preprocessing-model.md` — Phase 3.
- `../why-jacquard.md` — User-facing positioning that this roadmap delivers.
- `../timing-model-extensions.md` — Technical analysis underlying ADR 0007.
- `../timing-validation.md` — Validation tolerances each phase updates.
- `phase-0-ir-and-oracle.md` — Predecessor roadmap (current Phase 0 status lives there per workstream).
- `../spikes/opentimer-sky130.md` — Spike outcome (Superseded).
