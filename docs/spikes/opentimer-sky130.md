# Spike — OpenTimer on SKY130 and MCU SoC

**Status:** Proposed. Not yet executed.

**Time box:** Half a day. Extend by up to one day if initial signs are positive but hitting specific SKY130 quirks. Abort and fall back if first-four-hours progress is blocked.

## Goal

Determine whether OpenTimer (MIT, C++17) can reliably parse and analyse Jacquard's real-flow inputs — SKY130 Liberty and OpenLane2 MCU SoC post-P&R output — well enough to serve as Jacquard's in-process reference STA (per ADR 0003).

The outcome resolves ADR 0003's `Pending Spike` status to either `Accepted` or `Superseded`.

## Out of scope for this spike

- C++ FFI / bindgen integration work. Pure spike on OpenTimer's standalone behaviour.
- Timing-IR integration. Establishing that OpenTimer produces usable arrival/slack output is sufficient; converting it to IR belongs in phase 1.
- Performance measurement beyond rough "does it complete in reasonable time."
- GF130 coverage. SKY130 is the spike target; GF130 private-track confirmation is later.

## Setup

Required artefacts (checked before starting):

1. OpenTimer clone and local build (MIT licence, standard CMake).
2. SKY130 Liberty file(s) matching the corner the MCU SoC flow uses. At minimum `sky130_fd_sc_hd__tt_025C_1v80.lib`.
3. MCU SoC post-P&R output: synthesised `.v`, SDC, and — critically — `.spef`. Check that the current OpenLane2 invocation is configured to produce SPEF; if not, enable it. OpenTimer requires SPEF, it does not consume SDF.
4. Jacquard's current `timing-analysis` binary output on the same design for comparison.
5. OpenSTA installed locally, for three-way comparison.

## Success criteria

The spike answers four questions. Each is a pass/fail observation, not a measurement.

### Q1 — Does OpenTimer parse SKY130 Liberty without errors?

- **Pass:** clean parse, no warnings that indicate misinterpreted cells.
- **Partial:** parses but warns on specific cells — in particular `sky130_fd_sc_hd__dlygate4sd3_*` or anything with non-trivial conditional timing. Document which cells and whether their timing is discarded or mishandled.
- **Fail:** parse errors, segfaults, or silently-wrong output on recognised cells.

### Q2 — Does OpenTimer compute arrivals on the MCU SoC design?

Feed `.lib` + `.v` + `.spef` + `.sdc`. Run `report_timing -worst 20` or equivalent. Observe:

- **Pass:** produces a full timing report with reasonable-looking arrivals (non-zero, monotonic along paths).
- **Partial:** produces a report but with suspect values (many zeros, missing cells, incomplete paths).
- **Fail:** hangs, crashes, or refuses to analyse.

### Q3 — Does OpenTimer's result agree with OpenSTA?

Run OpenSTA on the same inputs, compare top-20 critical endpoints' arrivals. Declare tolerance: ±5% on arrival time, ±10 ps absolute floor for very short paths.

- **Pass:** all top-20 endpoints within tolerance.
- **Partial:** most within tolerance, a small number of outliers traceable to specific delay-model differences (e.g., CCS vs NLDM).
- **Fail:** systematic disagreement suggesting OpenTimer is computing something meaningfully different. Investigate; if the disagreement is on SKY130 cell interpretation (a PDK handling issue) this is essentially a fail for our purposes.

### Q4 — Does OpenTimer's result correlate with Jacquard's current timing analysis?

Compare worst-slack and top-K endpoint lists (not exact values — pessimism differences are expected and documented). Observe:

- **Pass:** top-K lists overlap substantially; worst-slack is on a comparable path.
- **Informational:** any systematic discrepancy tells us what the pessimism delta actually looks like in practice. This data informs R4 (critical-path refinement reporting) whether OpenTimer is adopted or not.

## Decision matrix

| Q1 | Q2 | Q3 | Outcome |
|---|---|---|---|
| Pass | Pass | Pass | ADR 0003 → Accepted. Proceed to phase 1 integration. |
| Pass | Pass | Partial | ADR 0003 → Accepted with documented scope limits. Define where OpenTimer is authoritative vs deferred to OpenSTA. |
| Pass | Partial | — | ADR 0003 → Accepted provisionally; spike extends to investigate Q2 anomalies. |
| Partial | — | — | ADR 0003 → Accepted with SKY130 cell workarounds documented, or → Superseded if the workarounds are too invasive. |
| Fail on any | — | — | ADR 0003 → Superseded. Fall back to OpenSTA-subprocess-only validation. Revisit libreda-sta or in-house walker as alternatives in a follow-up ADR. |

## Fallback

If the spike fails, Jacquard operates with:

- OpenSTA subprocess validation in CI (ADR 0001) as the sole timing-reference mechanism.
- No per-PR in-process timing cross-check; feedback timing degrades.
- Phase 1 drops OpenTimer integration work and refocuses on tightening OpenSTA-driven CI.

Superseding ADR 0003 is clean — it is currently `Pending Spike` so no downstream work has accrued to it. Phases 0 and 2 are unaffected.

## Deliverable

A short report added to this document as a "Spike outcome" section, summarising:

- Which Q1–Q4 answers were observed.
- Specific SKY130 cells where OpenTimer misbehaves (if any).
- Whether SPEF generation had to be added to the OpenLane2 flow, and what that change was.
- Decision: confirm, scope-limit, or supersede ADR 0003.

## Links

- ADR 0003 — OpenTimer as in-process reference STA.
- `../timing-correctness.md` — requirement R2.
- `../plans/phase-0-ir-and-oracle.md` — phase 0 (independent of this spike; runs in parallel).
