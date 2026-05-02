# Why Jacquard — positioning and output interface

**Status:** Honest assessment of where Jacquard fits in an EDA flow alongside dedicated STA tools (OpenTimer/OpenSTA) and event-driven simulators (Verilator, iverilog, CVC). Includes a survey of what timing information Jacquard exposes today and what would let users actually consume it.

This is not a marketing document. The goal is for a contributor or user to read it and decide accurately whether Jacquard helps them — and, if it does, how to extract the answer they need.

---

## TL;DR

Jacquard's unique value is **vector-driven timing analysis at GPU scale**: answering "did this stimulus violate setup/hold at any DFF, on which cycle, on which signal?" for designs large enough that SDF-annotated event-driven sim is too slow to finish in useful time.

Everything else Jacquard offers is offered, often better, by the standard flow:

- For functional sim: Verilator is faster on small designs.
- For timing: OpenSTA gives more accurate answers than Jacquard, vector-independent.
- For glitch / metastability: event-driven sim with SDF (CVC, iverilog) sees behaviours Jacquard's lockstep kernel structurally cannot.

Jacquard becomes the right tool when **(design size × vector length)** exceeds what event-driven SDF-annotated sim can handle, and you specifically want vector-driven timing answers.

**STA is not optional even with Jacquard.** Jacquard does not replace OpenSTA; it complements it. The right framing is "STA proves no bad vectors exist; Jacquard proves your real workload runs cleanly within those bounds." OpenSTA is also a hard runtime dependency for any timing-aware Jacquard flow — the timing IR is produced by `opensta-to-ir`, which subprocesses OpenSTA. See ADR 0001.

---

## What's actually unique

The intersection where Jacquard wins is narrow but real:

1. **Activity-driven setup/hold sweep at scale.** Run a long workload (boot trace, architectural validation, NoC congestion stimulus) on a large design at GPU speed; get a per-cycle violation report. STA can't tell you "this real workload trips violation X at cycle 12,847"; CVC can but won't finish in time on big designs.

2. **Arrival-time distributions for power/activity analysis.** Per-signal arrival histograms across millions of cycles → useful for worst-case-power analysis informed by actual switching activity. STA gives you nothing here; CVC could but slowly.

3. **Failure forensics.** When a functional test fails, answering "was this a timing issue?" without rerunning under a different simulator. Jacquard's timing-VCD output ties violations to cycle/signal/path — useful when you already have it from the same run.

4. **Fast iteration during timing closure.** Change a constraint, resynthesise, re-run a long test — Jacquard's loop time is short enough to make this practical on big designs in a way iverilog+SDF isn't.

## What dedicated STA (OpenSTA) gives you that Jacquard doesn't

This list is long and you should know it:

- **Worst-case path enumeration.** STA tells you the top-N critical paths *over all possible inputs*. Jacquard sees only what your stimulus exercises. If your testbench misses a critical path, Jacquard's "no violations" report is silent on it; OpenSTA would flag it.
- **True min-delay analysis.** OpenSTA does proper min-delay path search. Jacquard's hold check is per-DFF against actual stimulus only.
- **Per-pair CRPR.** OpenSTA applies common-path-pessimism removal as a launch/capture credit on each path. Jacquard consumes per-DFF clock arrival from `opensta-to-ir` and folds it into setup/hold (see [`timing-model-extensions.md`](timing-model-extensions.md), Part B Stages 1+2 — landed), but treats the launch reference as 0 — i.e. the per-pair CRPR credit is intentionally not modelled at this stage. Stage 3 in the same doc is the lever if Stage 1+2 pessimism turns out to matter on a real design.
- **SDC-aware constraint handling.** False paths, multi-cycle paths, generated clocks, async groups — OpenSTA reads SDC and respects it. Jacquard doesn't read SDC at the timing layer.
- **Coverage by construction.** STA covers every path by definition. Dynamic sim covers only what's exercised.
- **Vector-independent confidence.** "This design meets timing" is something STA can claim; Jacquard can only claim "this design met timing on these vectors."

## What event-driven SDF sim (CVC/iverilog) gives you that Jacquard doesn't

The honest comparison isn't "Jacquard vs. Verilator + OpenTimer." It's "Jacquard vs. iverilog/CVC-with-SDF + OpenTimer." On the timing-sim side specifically:

- **Glitch propagation.** CVC/iverilog with inertial or transport delay see intra-cycle pulses. Jacquard's lockstep cycle-accurate kernel does not.
- **Per-pin wire delay fidelity.** CVC consumes SDF interconnect records per-receiver, per-edge, with rise/fall distinction. Jacquard collapses to per-cell-max (see [`timing-model-extensions.md`](timing-model-extensions.md), Part C).
- **Per-DFF setup/hold without per-word collapse pessimism.** Jacquard collapses all DFFs in a 32-bit state word to `min(setup), min(hold)`; CVC checks each flop individually.
- **Async event handling.** Real `$setup`/`$hold` checks across asynchronous control. Jacquard explicitly assumes synchronous designs.

So today, accuracy-per-vector goes to CVC; throughput goes to Jacquard.

## When to choose what

| Your situation | Best tool |
|---|---|
| Small design, just want functional results | Verilator (free, fast, mature) |
| Small design, need timing certainty | OpenSTA + Verilator (or +CVC for vector-driven) |
| Large design, functional only | Verilator if it scales, else Jacquard |
| Large design, vector-driven timing needed | **Jacquard** + OpenSTA for STA backstop |
| Glitch / metastability investigation | CVC or iverilog with SDF — Jacquard cannot model these structurally |
| Asynchronous design / latches | Not Jacquard (synchronous-only) — use CVC/iverilog |
| Sign-off STA | OpenSTA / commercial — Jacquard is not a sign-off tool |

## The trajectory

Jacquard's timing fidelity gap with CVC is closeable. The work in [`timing-model-extensions.md`](timing-model-extensions.md) — δ(T), clock-tree skew, per-receiver wire delay — closes much of it while preserving GPU throughput. The further along that path the project goes, the more "Jacquard" looks like "GPU-accelerated SDF-annotated event-driven sim, with the inherent limits the cycle-accurate kernel imposes (no glitches, lockstep cycles)" — i.e. CVC's report quality at Verilator's speed, on designs where neither alone suffices.

---

## Output interface — what Jacquard exposes today

Jacquard's unique value depends on getting the timing information *out* of a run in a form users can act on. Phase 1 of the post-Phase-0 roadmap (ADR 0008) closed the gap between "data Jacquard has" and "answers users want" for setup/hold violations.

### Symbolic stderr violation messages
The kernel writes setup/hold violation events to a per-block event buffer (`csrc/kernel_v1.metal:554-576`). The host drains the buffer each cycle (`src/event_buffer.rs`), resolves the state-word index to a hierarchical DFF site name via `WordSymbolMap`, and emits:

```
[cycle 12847] SETUP VIOLATION at top/cpu/regs[7][bit 22] [word=412]: arrival=2150ps setup=80ps slack=-30ps
[cycle 12847] HOLD VIOLATION at top/cpu/state[bit 3] [word=412]: arrival=12ps hold=20ps slack=-8ps
```

The bare `[word=N]` suffix is preserved for grep/tooling compatibility; up to four DFFs per word are named, with `+N more` truncation beyond that.

### Structured timing report (`--timing-report <path.json>`)
Schema-versioned JSON document written at end of run. Contents:
- Per-cycle violation list (cycle, kind, word, site, arrival, constraint, slack).
- Per-word aggregate: violation counts and worst slack (sorted by total violations).
- Top-N worst-slack ranking per kind (setup, hold).
- Run metadata: design, vector source, timing source, clock period, cycles run, Jacquard version.
- Aggregate stats: setup/hold totals, dropped events.

Machine-readable, CI-friendly. Sample at `tests/timing_ir/sample_reports/two_violations.json`; full schema in `src/timing_report.rs` (`SCHEMA_VERSION = "1.0.0"`). Stability contract per ADR 0008: additive-only extensions, breaking changes bump the major.

### Text summary (`--timing-summary`)
One-screen human summary on stdout. Same data as the JSON report, different channel; either or both flags can be set:

```
=== Jacquard Timing Summary ===
Design:        my_cpu.gv
Vectors:       boot.vcd (1000 cycles)
Clock period:  1000 ps
Timing source: my_cpu.jtir

Violations:
  Setup: 5
  Hold:  2
  Total: 7

Worst slack:
  Setup: -150ps  at top/cpu/regs[7][bit 22] [word=5]  (cycle 87)
  Hold:   -40ps  at top/cpu/state[bit 3] [word=12]  (cycle 91)

Top 2 by violation count (of 2 total words with violations):
  top/cpu/regs[7][bit 22] [word=5] (5 violations): worst setup=-150ps hold=- arrival=950ps
  top/cpu/state[bit 3] [word=12] (2 violations): worst setup=- hold=-40ps arrival=10ps
```

Format is for human inspection — explicitly **not** a stable parseable contract. Tools should use `--timing-report` JSON.

### Timed VCD (`--timing-vcd`)
Annotates the output VCD with per-signal arrival times. Largest, most detailed output; suitable for waveform-level inspection.

- **What you get:** per-signal arrival ps at each writeout cycle.
- **Caveat:** the VCD doesn't carry slack relative to the clock edge — you compute it yourself.
- **Cost:** doubles VCD size. Not appropriate for long workloads on large designs.

### `SimStats` aggregate counts (in-process)
`SimStats { setup_violations, hold_violations, ... }` is available to in-process consumers (`src/event_buffer.rs`). Only counts; full detail flows through the structured report path.

## Still on the wishlist

Items captured in ADR 0008's "Optional / later outputs" plus a few caveats on what shipped. Demand-driven; not scheduled.

### Closest-to-violation tracking when no violation occurred
The shipped `worst_slack` ranking is populated only from observed violation events. Surfacing "where am I close to the edge" on a run that *passed* timing requires GPU-side near-miss instrumentation (emit slack events whenever |slack| falls below a configurable threshold). Useful for proactive signoff regression. Separate workstream — needs a kernel change.

### Arrival histogram (`--arrival-histogram <pattern>`)
Per-signal arrival histogram dump for matched signal patterns, as JSON or CSV. Foundation for activity-based power analysis and "is my actual timing margin healthy" reporting.

### STA cross-reference (`--sta-cross-reference <opensta-paths.txt>`)
Read OpenSTA's worst-N critical-path report and produce coverage output: of those paths, which were exercised by the stimulus, at what observed arrival. Closes the loop between vector-driven and static analysis.

### Path back-trace from worst-arrival DFF
Given a flagged DFF, walk the max-of-fanin chain backward to the source AIG pin / primary input, emitting per-edge contributions. Most expensive item on the wishlist; only useful once symbolic names are in place (which they now are).

### CUDA / HIP / cosim runtime violation routing
The current Metal sim path routes runtime violations through `process_events` (which is what feeds the resolver, structured report, and text summary). The CUDA, HIP, and cosim paths don't yet share that plumbing — they detect violations on the GPU but don't drain through `process_events`. Independent plumbing follow-up; doesn't affect the Metal user experience.

### Per-signal activity / transition counts
Listed in ADR 0008 as part of the JSON report's wishlist. Not in v1.0.0 of the schema; will be added (additively) when the GPU kernel emits transition events.

### "Corner" and "margin percentage" in the text summary
ADR 0008's summary template includes both. Corner is missing because the metadata struct doesn't carry it through from the IR yet; margin percentage is trivially derivable from `slack_ps / clock_period_ps` and was omitted to keep the v1 summary terse.

---

## Related artefacts

- [`project-scope.md`](project-scope.md) — what Jacquard is for and not for; the formal contract this doc operates inside.
- [`timing-correctness.md`](timing-correctness.md) — forward-looking validation requirements.
- [`timing-violations.md`](timing-violations.md) — current GPU-side violation detection mechanics.
- [`timing-validation.md`](timing-validation.md) — how Jacquard's timing output is validated against CVC/iverilog.
- [`timing-model-extensions.md`](timing-model-extensions.md) — proposed accuracy improvements (δ(T), clock-tree skew, wire delay).
