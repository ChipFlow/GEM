# ADR 0007 — Timing model fidelity roadmap

**Status:** Proposed.

## Context

Jacquard's timing model today consumes SDF-equivalent annotations via the timing IR (ADR 0002), validated against OpenSTA (ADR 0001) and OpenTimer (ADR 0003). The accuracy contract at present is "±5% on arrival times against CVC reference" per `timing-validation.md`. This is acceptable for sky130-class designs at ≥10 ns clock periods.

Three structural simplifications in the current implementation become accuracy bottlenecks at scale:

1. **Static δ∞ per gate.** No pulse-degradation modelling. Glitch behaviour and short-pulse propagation cannot be represented. The Involution Delay Model (Maier 2021, [arXiv:2107.06814](https://arxiv.org/abs/2107.06814)) demonstrates this is the root cause of inertial-delay's known failure modes, and provides a model that's both faithful and implementable.
2. **Zero clock-tree skew.** During AIG construction (`src/aig.rs:495-560`), clock buffers/inverters/gating cells collapse to a single polarity flag on the DFF. SDF arcs and interconnect on the clock tree are silently dropped. Every DFF on a clock domain is treated as capturing simultaneously.
3. **Per-cell-max wire delay.** `src/flatten.rs:1850-1872` lumps all interconnect arrivals at a destination cell into a single max value, with no rise/fall distinction. Adequate for short local routes; incorrect for long routes where wire delay rivals or exceeds gate delay (typical of NoCs at 22nm and faster).

The full design analysis is in `docs/timing-model-extensions.md`. This ADR captures the **decision to commit** to closing these three gaps as a roadmap, sets the staged ordering, and constrains how the implementation may evolve.

## Decision

Adopt a three-pillar roadmap for closing the fidelity gap with CVC, while preserving Jacquard's GPU-throughput advantage. All three pillars are consumer-side work (`src/flatten.rs`, `src/aig.rs`, `src/sim/cosim_metal.rs`, the kernel arrival math); none require schema changes inconsistent with ADR 0002 nor abandoning the cycle-accurate boomerang kernel architecture.

### Pillar A — Dynamic delay (δ(T))

Per-gate dynamic delay parameterised on T (time since last output transition). Three accuracy tiers:

- **Static IDM.** Bake worst-case δ(T) into existing per-thread script slot using STA pulse-width estimates. No kernel change.
- **Dynamic δ(T).** Add `last_transition_ps` and `last_value` persistent buffers per AIG pin; kernel evaluates δ(T) from a small per-cell LUT during arrival propagation.
- **Sub-cycle ticks.** Multiple arrival propagations per logical cycle, enabling true glitch suppression. **Out of scope by this ADR.** Would require a different kernel architecture; if pursued, requires its own ADR superseding this one.

### Pillar B — Clock-tree skew

Per-DFF clock arrival accounting via TimingIR extension (`ClockArrival` table) populated by the chosen STA tool (OpenTimer per ADR 0003), with CRPR. Consumed by extending `DFFConstraint` with a `clock_arrival_ps: i16` field, folded into the existing per-word setup/hold check in `src/flatten.rs:1732`. No kernel change for the baseline case; bucketed packing is an option if pessimism becomes material.

### Pillar C — Wire delay at scale

Three fidelity tiers:

- **Tier 1: Per-receiver consumption.** Key wire delay by `(src_aigpin, dst_aigpin)` edge in the AIG, with rise/fall distinction preserved. Mostly a `src/flatten.rs:1850-1872` rewrite. No kernel change.
- **Tier 2: Inter-partition arc delay.** Explicit modelling of wire delay on partition-crossing signals. Touches `src/sim/cosim_metal.rs` shuffle pipeline. Required for many-core/NoC designs at advanced processes.
- **Tier 3: NoC-aware partitioning hints.** Soft bias in `src/repcut.rs` favouring cuts on flagged net patterns. Optional optimisation that makes Tier 2 cheap on tile-decomposed designs.

### Sequencing constraint

- **Pillar B Stage 1+2** is the cheapest accuracy improvement and is gated on Phase 1 OpenTimer integration (ADR 0003). Lands first.
- **Pillar C Tier 1** is independent of OpenTimer and can proceed in parallel.
- **Pillar A Stage 1 (Static IDM)** is the cheapest δ(T) entry point, gated on per-cell SPICE characterisation effort. Schedule this only after Pillars B and C land — δ(T) compounds on top of correct wire/skew baseline; doing it earlier risks chasing characterisation noise that's actually wire-delay error.
- **Pillar C Tier 2** lands when a real many-core/NoC use case appears in the test corpus and Tier 1 measurement shows it's needed.
- **Pillar A Stage 2 (Dynamic δ(T))** is a substantial implementation; schedule only when Stage 1 reports indicate the value is real, *and* a contributor with the analog-characterisation domain expertise is willing to lead it.
- **Pillar A Stage 3 (Sub-cycle ticks)** is explicitly out of scope of this ADR.

### Validation contract

- Each pillar lands with regression coverage extending `timing-validation.md`'s ±5% tolerance. Tighter tolerances may apply per pillar (Pillar B should achieve ≤±2% on skew-aware paths once OpenTimer is integrated; Pillar C Tier 1 should achieve ≤±3% on long-wire paths).
- Each pillar must demonstrate no regression on the existing primary corpus before merge.
- The IR schema may be extended (additive only) per ADR 0002 to carry pillar-specific data. Extensions require a minor schema bump and a documented consumer-version compatibility note.

## Consequences

- The "±5%" line in `timing-validation.md` becomes a per-pillar specification rather than a single number. The doc is updated as each pillar lands.
- `crates/timing-ir/schemas/timing_ir.fbs` accumulates additive extensions for clock arrival and per-cell δ(T) parameters. Schema versioning per ADR 0002 governs.
- No changes to the cycle-accurate boomerang kernel architecture. The cost of preserving that architecture is permanent: no glitch propagation, no metastability oscillation, no asynchronous handling. These remain non-goals (per `project-scope.md`) unless a future ADR explicitly supersedes this position.
- Per-cell SPICE characterisation effort is acknowledged as the long-pole risk for Pillar A. If characterisation cost proves prohibitive, Pillar A reduces to "Stage 1 only, using Liberty-derived ECSM/CCSM data as approximation," and the gap with CVC's full IDM fidelity remains open. This is acceptable; Pillar A Stage 2 is not a release-gating commitment.
- Jacquard's positioning (`why-jacquard.md`) becomes coherent: STA-complement-not-replacement, vector-driven timing at GPU scale, fidelity comparable to CVC where the cycle-accurate kernel architecture allows.

## Walk-back options

- **If a pillar's measurement shows the accuracy gain is smaller than expected**, descope it. Each pillar's first stage is sized to deliver measurable improvement; if it doesn't, later stages of that pillar are deferred or abandoned.
- **If the IR schema extensions cause downstream tooling friction**, fall back to vendor-extension passthrough (`VendorExtension` in `timing_ir.fbs`) until the typed schema stabilises. Already supported.
- **If OpenTimer integration stalls** (per ADR 0003 spike outcome), Pillar B falls back to manual clock-tree accumulation in `src/aig.rs`. Lower fidelity (no CRPR), but unblocks the rest of the roadmap.

## Links

- `../timing-model-extensions.md` — full technical analysis underlying this ADR.
- `../why-jacquard.md` — positioning context: where this fidelity work fits in the user value story.
- `../adr/0001-opensta-as-oracle.md`, `../adr/0002-timing-ir.md`, `../adr/0003-opentimer-primary-sta.md` — preceding decisions this ADR builds on.
- `../timing-validation.md` — validation tolerance contract that each pillar updates.
- `../project-scope.md` — synchronous-only / cycle-accurate constraints that bound what this ADR can pursue.
