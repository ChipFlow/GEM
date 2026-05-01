# Roadmap — Post-Phase-0 work scheduling

**Status:** Proposed. Awaits acceptance of ADRs 0007 and 0008.

This document orders the work captured in those two ADRs alongside the in-flight tail of Phase 0 and the pending OpenTimer spike. It is a **scheduling** doc, not a design doc — design lives in the ADRs and in `docs/timing-model-extensions.md` / `docs/why-jacquard.md`.

## Where things stand (2026-05-01)

- **Phase 0 (`phase-0-ir-and-oracle.md`)**: nearing close. WS1–WS3 landed; WS2.2 (interconnect_delays) landed in commit `67210c0`. Open items: **WS2.4** (multi-corner CLI flag), **WS4 reframing decision**, **WS5 parser-success assertions**, **peripheral wiring** for I²C/SPI when a fuller mcu_soc fixture lands. See `post-cosim-models-handoff.md`.
- **OpenTimer spike (`spikes/opentimer-sky130.md`)**: **in flight** as of 2026-05-01, running in parallel with Phase 0 wrap-up. Time-boxed at half a day. Outcome resolves ADR 0003 to either `Accepted` or `Superseded`, which gates Phase 1 entry below.
- **Phase 3 (`adr/0006-sdf-preprocessing-model.md`)**: native Rust SDF→IR parser. Lands before first release. Independent of the work in this doc.
- **ADRs 0007 / 0008**: proposed in this round; pending review.

## Phase boundaries

The phase numbering established by Phase 0 and ADR 0006 continues:

| Phase | Topic | Trigger |
|---|---|---|
| **0** | Timing IR + OpenSTA oracle | In flight, near close |
| **1** | OpenTimer integration + structured timing output (ADR 0008 required items) | OpenTimer spike resolves to `Accepted`; ADR 0008 accepted |
| **2** | Timing model fidelity Pillar B + Pillar C Tier 1 (ADR 0007) | Phase 1 lands; ADR 0007 accepted |
| **3** | Native Rust SDF→IR parser (ADR 0006) | Pre-release; can run parallel to Phase 1/2 |
| **4+** | Pillar A Stage 1 (static IDM); Pillar C Tier 2; ADR 0008 optional outputs | Demand-driven; not committed |

**Parked (require new ADR to revive):** Pillar A Stage 2 (dynamic δ(T)), Pillar A Stage 3 (sub-cycle ticks), NoC-aware partitioning hints (Pillar C Tier 3).

## Phase 1 — OpenTimer integration and usable output

**Entry criteria:**
- OpenTimer spike (`spikes/opentimer-sky130.md`) resolves to `Accepted`.
- ADR 0008 accepted.
- Phase 0 exit criteria met (per `phase-0-ir-and-oracle.md`).

**Workstreams (parallel where independent):**

### WS-P1.1 — OpenTimer integration
Per ADR 0003, integrate OpenTimer as the primary STA tool. Subprocess invocation, IR-output, validation against OpenSTA.

- Detailed plan to be written when the spike resolves. Likely structure mirrors `ws2-opensta-to-ir.md`.
- Deliverables: `opentimer-to-ir` wrapper or unified `sta-to-ir` covering both OpenSTA and OpenTimer; CI parallel-run with `timing-ir-diff` against OpenSTA's output; validation tolerance ≤±2% on the primary corpus.
- Unblocks: Phase 2 Pillar B (clock-tree skew via CRPR-aware arrivals).

### WS-P1.2 — Structured timing output (ADR 0008 required items)
The four required items from ADR 0008. Single workstream because they share infrastructure.

- **WS-P1.2.a — Symbolic violation messages.** ~1–2 days. `src/event_buffer.rs:305-338` plus a name-resolution helper from netlistdb. No new flags; format change documented in changelog.
- **WS-P1.2.b — `--timing-report <path.json>`.** ~3–5 days. End-of-run JSON document; schema versioned per ADR 0002 conventions; sample reports checked into corpus.
- **WS-P1.2.c — `--timing-summary` text output.** ~1 day, after WS-P1.2.b. Trivial wrapper over the JSON data.
- **WS-P1.2.d — Per-DFF worst-slack ranking.** ~1–2 days, folds into WS-P1.2.b. Top-N by closest-to-violation slack across the run.

Total ~2 weeks. Independent of WS-P1.1 — can land in either order.

### WS-P1.3 — Phase 0 follow-ups (carryover)
Tail of Phase 0 work that didn't gate WS3 completion. Listed for completeness.

- WS2.4: multi-corner CLI flag in `opensta-to-ir`.
- WS4 reframing: decide whether `timing-ir-diff` continues as authored or is folded into a wider regression harness.
- WS5: parser-success assertions on the Liberty parser path.
- Peripheral wiring for I²C/SPI when a fuller mcu_soc fixture lands.

These are not gated by any new ADR; pick them up as bandwidth allows.

**Exit criteria for Phase 1:**
- OpenTimer integrated and CI-validated against OpenSTA.
- Symbolic violation messages live; old state-word-index format gone.
- `--timing-report` JSON shipping; sample golden reports in corpus.
- `--timing-summary` available.
- Worst-slack ranking included in both report and summary.
- `why-jacquard.md` updated to remove "what's missing" qualifications for items 1–4.

## Phase 2 — Timing model fidelity

**Entry criteria:**
- Phase 1 exit criteria met (specifically: OpenTimer integration provides per-DFF clock arrival via CRPR).
- ADR 0007 accepted.

**Workstreams (parallel where independent):**

### WS-P2.1 — Pillar B: Clock-tree skew (ADR 0007)
Per-DFF clock arrival accounting via TimingIR extension.

- **WS-P2.1.a — `ClockArrival` IR table.** Schema addition in `crates/timing-ir/schemas/timing_ir.fbs`; populator in OpenTimer wrapper. ~2–3 days.
- **WS-P2.1.b — Consumer plumbing.** Extend `DFFConstraint` with `clock_arrival_ps: i16`; fold into per-word setup/hold buffer in `src/flatten.rs:1732`. ~1–2 days.
- **WS-P2.1.c — Validation extension.** Add skew-aware tolerance to `timing-validation.md`; add corpus regression on a design with non-trivial clock tree.

Gated on WS-P1.1 (OpenTimer integration). Total ~1 week.

### WS-P2.2 — Pillar C Tier 1: Per-receiver wire delay (ADR 0007)
Key wire delay per `(src_aigpin, dst_aigpin)` edge. Independent of WS-P2.1.

- **WS-P2.2.a — Edge-attributed wire delay.** Rewrite of `src/flatten.rs:1850-1872` to key wire delay per fanout; fold into source-side gate_delay per fanout target. ~3–5 days.
- **WS-P2.2.b — Rise/fall preservation.** Carry per-edge rise/fall through the consumer; honour both in `PackedDelay` accumulation. ~1–2 days, after WS-P2.2.a.
- **WS-P2.2.c — Validation.** Long-route corpus addition; tolerance ≤±3% on long-wire paths.

Total ~1 week.

### WS-P2.3 — Output adjustments for fidelity work
Small touch-ups to ensure Phase 1 outputs continue to work as model fidelity changes. JSON report fields, summary metrics, etc. Folded into WS-P2.1 / WS-P2.2 PRs as needed.

**Exit criteria for Phase 2:**
- `ClockArrival` table populated in IR; consumer plumbed through; setup/hold reports skew-aware.
- Per-receiver wire delay landed; long-route paths reported within ≤±3% of CVC.
- `timing-model-extensions.md` Parts B and C marked **Implemented (Phase 2)** with cross-references to landed code.
- `timing-validation.md` updated with per-pillar tolerances.
- No regression on existing corpus.

## Phase 3 — Native Rust SDF→IR parser

Existing per ADR 0006. Runs parallel to Phases 1/2 as bandwidth allows. Lands before first release. No additions from this roadmap.

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

- **OpenTimer spike fails.** Per ADR 0003, that resolves the ADR to `Superseded` and OpenSTA remains primary. Pillar B falls back to manual clock-tree accumulation in `src/aig.rs` — lower fidelity (no CRPR), but unblocks the rest.
- **Pillar measurement shows smaller-than-expected gain.** Each pillar's later stages are deferred or abandoned per ADR 0007's walk-back clause.
- **JSON report schema design wastes time in bikeshedding.** Mitigation: ship v1 quickly, additive-only changes thereafter, breaking changes require explicit ADR-level decision.
- **Phase 1 and Phase 2 overlap creates merge churn.** Pillar B depends on Phase 1 OpenTimer; sequence the merges to avoid trivial conflicts. Pillar C Tier 1 is independent; can land in parallel with Phase 1.

## Cross-references

- `../adr/0007-timing-model-fidelity-roadmap.md` — Pillar definitions for Phase 2.
- `../adr/0008-structured-timing-output.md` — Output items for Phase 1.
- `../adr/0003-opentimer-primary-sta.md` — Gates Phase 1.
- `../adr/0006-sdf-preprocessing-model.md` — Phase 3.
- `../why-jacquard.md` — User-facing positioning that this roadmap delivers.
- `../timing-model-extensions.md` — Technical analysis underlying ADR 0007.
- `../timing-validation.md` — Validation tolerances each phase updates.
- `phase-0-ir-and-oracle.md` — Predecessor roadmap.
- `post-cosim-models-handoff.md` — Current Phase 0 state.
- `../spikes/opentimer-sky130.md` — Gates Phase 1.
