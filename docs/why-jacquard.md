# Why Jacquard — positioning and output interface

**Status:** Honest assessment of where Jacquard fits in an EDA flow alongside dedicated STA tools (OpenTimer/OpenSTA) and event-driven simulators (Verilator, iverilog, CVC). Includes a survey of what timing information Jacquard exposes today and what would let users actually consume it.

This is not a marketing document. The goal is for a contributor or user to read it and decide accurately whether Jacquard helps them — and, if it does, how to extract the answer they need.

---

## TL;DR

Jacquard's unique value is **vector-driven timing analysis at GPU scale**: answering "did this stimulus violate setup/hold at any DFF, on which cycle, on which signal?" for designs large enough that SDF-annotated event-driven sim is too slow to finish in useful time.

Everything else Jacquard offers is offered, often better, by the standard flow:

- For functional sim: Verilator is faster on small designs.
- For timing: OpenTimer/OpenSTA gives more accurate answers than Jacquard, vector-independent.
- For glitch / metastability: event-driven sim with SDF (CVC, iverilog) sees behaviours Jacquard's lockstep kernel structurally cannot.

Jacquard becomes the right tool when **(design size × vector length)** exceeds what event-driven SDF-annotated sim can handle, and you specifically want vector-driven timing answers.

**STA is not optional even with Jacquard.** Jacquard does not replace OpenTimer; it complements it. The right framing is "STA proves no bad vectors exist; Jacquard proves your real workload runs cleanly within those bounds."

---

## What's actually unique

The intersection where Jacquard wins is narrow but real:

1. **Activity-driven setup/hold sweep at scale.** Run a long workload (boot trace, architectural validation, NoC congestion stimulus) on a large design at GPU speed; get a per-cycle violation report. STA can't tell you "this real workload trips violation X at cycle 12,847"; CVC can but won't finish in time on big designs.

2. **Arrival-time distributions for power/activity analysis.** Per-signal arrival histograms across millions of cycles → useful for worst-case-power analysis informed by actual switching activity. STA gives you nothing here; CVC could but slowly.

3. **Failure forensics.** When a functional test fails, answering "was this a timing issue?" without rerunning under a different simulator. Jacquard's timing-VCD output ties violations to cycle/signal/path — useful when you already have it from the same run.

4. **Fast iteration during timing closure.** Change a constraint, resynthesise, re-run a long test — Jacquard's loop time is short enough to make this practical on big designs in a way iverilog+SDF isn't.

## What OpenTimer gives you that Jacquard doesn't

This list is long and you should know it:

- **Worst-case path enumeration.** STA tells you the top-N critical paths *over all possible inputs*. Jacquard sees only what your stimulus exercises. If your testbench misses a critical path, Jacquard's "no violations" report is silent on it; OpenTimer would flag it.
- **True min-delay analysis.** OpenTimer does proper min-delay path search. Jacquard's hold check is per-DFF against actual stimulus only.
- **Clock-tree skew / CRPR.** OpenTimer computes per-flop clock arrival with common-path-pessimism removal. Jacquard today treats all DFFs on a clock as zero-skew (see [`timing-model-extensions.md`](timing-model-extensions.md), Part B).
- **SDC-aware constraint handling.** False paths, multi-cycle paths, generated clocks, async groups — OpenTimer reads SDC and respects it. Jacquard doesn't read SDC at the timing layer.
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
| Small design, need timing certainty | OpenTimer + Verilator (or +CVC for vector-driven) |
| Large design, functional only | Verilator if it scales, else Jacquard |
| Large design, vector-driven timing needed | **Jacquard** + OpenTimer for STA backstop |
| Glitch / metastability investigation | CVC or iverilog with SDF — Jacquard cannot model these structurally |
| Asynchronous design / latches | Not Jacquard (synchronous-only) — use CVC/iverilog |
| Sign-off STA | OpenTimer / OpenSTA / commercial — Jacquard is not a sign-off tool |

## The trajectory

Jacquard's timing fidelity gap with CVC is closeable. The work in [`timing-model-extensions.md`](timing-model-extensions.md) — δ(T), clock-tree skew, per-receiver wire delay — closes much of it while preserving GPU throughput. The further along that path the project goes, the more "Jacquard" looks like "GPU-accelerated SDF-annotated event-driven sim, with the inherent limits the cycle-accurate kernel imposes (no glitches, lockstep cycles)" — i.e. CVC's report quality at Verilator's speed, on designs where neither alone suffices.

---

## Output interface — what Jacquard exposes today

Jacquard's unique value depends on getting the timing information *out* of a run in a form users can act on. Today the surface area is functional but limited.

### Timed VCD (`--timing-vcd`)
Annotates the output VCD with per-signal arrival times. Largest, most detailed output; suitable for waveform-level inspection.

- **What you get:** per-signal arrival ps at each writeout cycle.
- **What's missing:** the VCD doesn't carry slack relative to the clock edge — you compute it yourself.
- **Cost:** doubles VCD size. Not appropriate for long workloads on large designs.

### Stderr violation messages
The kernel writes setup/hold violation events to a per-block event buffer (`csrc/kernel_v1.metal:554-576`). The host drains the buffer each cycle (`src/event_buffer.rs:305-338`) and emits human-readable warnings:

```
[cycle 12847] SETUP VIOLATION: word 412 arrival=2150ps setup=80ps slack=-30ps
[cycle 12847] HOLD VIOLATION: word 412 arrival=12ps hold=20ps slack=-8ps
```

- **What you get:** every violation, in real time, tagged with cycle and state-word.
- **What's missing:** the message identifies a *state-word index*, not a signal name. Mapping back to "which DFF, which path" is currently a manual exercise. Volume on a violating design can be enormous (one warning per word per cycle per type).

### `SimStats` aggregate counts (in-process)
`SimStats { setup_violations, hold_violations, ... }` is available to in-process consumers (`src/event_buffer.rs:218-231`). Only counts; no detail.

### The event buffer (raw)
Each violation event in the buffer carries `cycle`, `data[0]` = state-word, `data[1]` = signed slack ps, `data[2]` = arrival ps, `data[3]` = constraint ps (`src/event_buffer.rs:305-338`). This is the most structured violation data Jacquard produces, but it's only consumed by the logging path — there's no public dump-to-file mechanism today.

### What's missing — the gap between "data Jacquard has" and "answers users want"

| User question | Today | What's needed |
|---|---|---|
| "Did my workload trip any violations?" | `SimStats` counts | Already available in-process; needs CLI exposure |
| "Which DFFs nearly missed timing?" | Not extractable without parsing stderr | Per-DFF worst-slack ranking written at end-of-run |
| "Show me arrival distribution per signal" | Reconstructable from --timing-vcd via post-processing | Native histogram output, opt-in per signal pattern |
| "Which DFF was that violation on?" | State-word index + manual lookup | Symbolic mapping in violation reports (signal name, hierarchical path) |
| "What path caused the worst arrival?" | Not available | Path-back-trace from worst-arrival DFF through max-of-fanin chain to source |
| "What activity factor did each signal see?" | Not available | Per-signal transition-count alongside arrival data |
| "Did my stimulus exercise these critical paths from STA?" | Not available | Coverage report: cross-reference STA's critical-path list vs. observed arrival peaks |
| "Run this in CI and fail if any violation" | Possible via stderr grep | Structured exit-code semantics + machine-readable summary |

## Proposed output interface

What follows is design intent, not implementation. It belongs in [`timing-model-extensions.md`](timing-model-extensions.md) once scoped, but is captured here as the "what's the user-visible win" framing.

### Structured timing report (`--timing-report path.json`)
Write a JSON document at end-of-run with:

- Per-DFF worst arrival, worst slack, violation count over the run (top-N or all).
- Per-cycle violation list (cycle, signal name, hierarchical path, arrival, constraint, slack).
- Aggregate stats: total violations, distribution buckets, peak arrival per clock domain.
- Activity per signal: transition count, average/max arrival, idle cycles.
- Run metadata: clock period, SDF/IR file, design hash, vector source.

Machine-readable, CI-friendly, post-processable. The single most useful addition for actually using Jacquard's timing output.

### Timing summary (`--timing-summary`)
Fast text summary, no VCD. Designed for scripts and dashboards:

```
Jacquard timing summary
  design: nvdla_post_pnr     vectors: 100000 cycles
  clock:  1.2 ns             SDF corner: typ
  violations:  setup=0  hold=0
  worst slack:  setup=+213ps (DFF nvdla/conv/regs[42])
                hold=+18ps   (DFF nvdla/dma/state[3])
  peak arrival per writeout: 987ps  (clock budget 1200ps, 17.7% margin)
```

### Symbolic violation messages
Replace state-word indices with hierarchical signal names (`nvdla/conv/regs[42]`) in stderr violation messages. Pure UX improvement; the mapping data already exists in the netlistdb.

### Arrival histogram output (`--arrival-histogram`)
Optional per-signal arrival histogram dump. For a flagged signal pattern, write a CSV/JSON of arrival distribution across the run. Foundation for activity-based power analysis and "is my actual timing margin healthy" reporting.

### Optional: STA cross-reference (`--sta-cross-reference path.txt`)
Read OpenTimer's critical-path report and produce a coverage-style output: which of STA's worst-N paths were actually exercised by the stimulus, and what was their observed worst arrival. Closes the loop between vector-driven and static analysis.

## Priority

For a user trying to actually extract Jacquard's unique value today, the gap is biggest at:

1. **Symbolic violation messages.** Cheapest fix, biggest UX win. Without this, every violation requires manual investigation.
2. **`--timing-report` JSON.** Required for CI integration and for any downstream tooling.
3. **`--timing-summary` text.** Required for human inspection of long runs.
4. **Per-DFF worst-slack ranking.** Required for "where am I close to the edge" analysis.

The other items (histograms, path-back-trace, STA cross-reference) are higher-value but lower-priority — they make Jacquard a *better* timing-analysis tool, while the first four make Jacquard *useful at all* for non-expert users running it in a real flow.

---

## Related artefacts

- [`project-scope.md`](project-scope.md) — what Jacquard is for and not for; the formal contract this doc operates inside.
- [`timing-correctness.md`](timing-correctness.md) — forward-looking validation requirements.
- [`timing-violations.md`](timing-violations.md) — current GPU-side violation detection mechanics.
- [`timing-validation.md`](timing-validation.md) — how Jacquard's timing output is validated against CVC/iverilog.
- [`timing-model-extensions.md`](timing-model-extensions.md) — proposed accuracy improvements (δ(T), clock-tree skew, wire delay).
