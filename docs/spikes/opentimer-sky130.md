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

## Progress log

### Setup (2026-04-23 → 2026-04-30)

- OpenTimer 2.1.0 and OpenSTA 3.1.0 cloned to `Jacquard-depends/` and built
  locally. Build notes in that repo's `README.md`.
- SKY130 Liberty already on disk via volare:
  `~/.volare/volare/sky130/versions/c6d73a35f524070e85faff4a6a9eef49553ebc2b/sky130A/libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib`.
- Spike artefacts kept in this worktree under `spike-out/` (gitignored —
  reproducible from `Jacquard-depends/`).

### Q1 — Liberty parse (2026-04-30) — **Pass**

| Tool          | Cells loaded | Wall time | Warnings |
| ------------- | ------------ | --------- | -------- |
| OpenTimer 2.1.0 | 428        | 0.12 s    | 1        |
| OpenSTA 3.1.0   | 428        | 0.18 s    | 0        |

Cell counts agree exactly. OpenSTA parses cleanly. OpenTimer emits one
warning:

```
W celllib.cpp:274] unexpected lut template variable normalized_voltage
```

The `normalized_voltage` axis appears in exactly one place in the Liberty:
the library-level `normalized_driver_waveform("driver_waveform_template")`
block, which is CCS-driver-waveform data. No per-cell timing arc references
it — `cell_rise`/`cell_fall`/`rise_constraint`/`fall_constraint` all use
the NLDM templates `del_1_7_7`, `vio_3_3_1`, `constraint_3_0_1`. So the
warning has no impact on arrival/slack computation under NLDM, which is
what OpenTimer does anyway.

**Operational note:** OpenTimer's `read_celllib` is lazy — the parse only
runs when an action like `update_timing` (or `report_*`) forces taskflow
execution. Issuing `dump_celllib` immediately after `read_celllib` reports
"celllib not found" because the read hasn't fired yet. Always insert
`update_timing` before any inspection command.

The documented `read_celllib -min|-max <file>` syntax silently no-ops; bare
`read_celllib <file>` loads the lib as both min and max corners. Filed as a
docs/build mismatch in our `Jacquard-depends/README.md`.

### Q2 — Arrival computation on SKY130 (2026-05-01) — **Fail**

Used OpenSTA's bundled `gcd_sky130hd` example (a canonical SKY130-HD GCD
with `.v`, `.sdc`, `.spef`, `.lib`) as a fast smoke test before tackling
MCU-SoC SPEF generation. If OpenTimer can't handle this, the MCU-SoC
effort is wasted.

**OpenSTA baseline:** clean run, period 5 ns, top arrival 4.82 ns,
WNS 0.00, slack 0.09 met. 0.28 s wall, zero warnings.

**OpenTimer:** could not produce a single timing path. The result was
`no critical path found`, `wns = nan`, `tns = nan` — even after working
around the following issues, each of which had to be discovered and
patched manually:

| # | Issue | Workaround tried | Status |
|---|-------|------------------|--------|
| 1 | `read_celllib -min|-max <file>` (the documented syntax) silently no-ops | bare `read_celllib <file>` loads as both corners | works |
| 2 | `dump_*` after `read_*` reports state-not-loaded because the read is lazy | insert `update_timing` before any inspection | works |
| 3 | Tap cells in post-P&R Verilog (`sky130_fd_sc_hd__tapvpwrvgnd_*`) trigger 1040 `cell not found in celllib` errors and abort the netlist load | strip tap cell instances from Verilog | works |
| 4 | OpenTimer's bundled SDC parser uses pre-TCL-8.5 syntax (`trace variable VAR w CMD`); fails on the system's TCL 8.6 with `bad option "variable"` and produces zero parsed commands — even on OpenTimer's own bundled examples | patch `ot/sdc/sdcparsercore.tcl:144` to `trace add variable sdc_version write __set_v` | works (one-line fix; should be upstreamed) |
| 5 | OpenSTA-style SDC with `set period 5 / expr $period * 0.2 / [all_inputs]` parses as zero commands | hand-write a literal SDC with `create_clock -name clk -period 5 [get_ports clk]` | works for trivial constraints; non-trivial SDC remains uncovered |
| 6 | SPEF `*PORTS` section (standard SPEF, IEEE 1481, emitted by OpenROAD/OpenLane) is rejected with a parse error pointing at the first port line | strip `*PORTS` block from SPEF before reading | works |
| 7 | Verilog bus ports (`input [31:0] req_msg;`) are not bit-blasted by OpenTimer's Verilog parser, but post-P&R SPEF references the bus as bit-indexed nets (`req_msg[0]`, `req_msg[1]`, …). 48 bus-element nets fail to match between netlist and SPEF | none found | **blocking** |
| 8 | After all of the above, two interior pins (`_251_:B`, `_218_:B`) report "not found in rctree" and the timing graph remains disconnected enough that no path can be reported | not investigated further | **blocking** |

Issues 7 and 8 mean that on a SKY130 design with bus ports — i.e. any
design that talks to the rest of the world — OpenTimer cannot compute
arrivals from a standard OpenROAD `.v`/`.spef` pair without inputs being
pre-processed by code that doesn't exist.

The cumulative finding is not "OpenTimer mishandles a few SKY130 cells".
It is that **OpenTimer's input pipeline (Verilog parser, SPEF parser,
bundled SDC parser) is incomplete relative to what real OpenROAD-flow
outputs contain**, and the gaps fall on hot paths (bus ports, tap cells,
modern TCL, OpenROAD-emitted SPEF). The cells themselves parse fine (Q1);
it's the surrounding ecosystem that doesn't.

### Q3, Q4 — not run

Q3 (cross-check vs OpenSTA) and Q4 (correlation with Jacquard's
`timing-analysis`) both depend on OpenTimer producing arrivals. With Q2
unable to produce a single path, they're moot for this spike.

### Decision

**ADR 0003 → Superseded.** Per the spike's decision matrix
("Fail on any → ADR 0003 → Superseded. Fall back to OpenSTA-subprocess-only
validation"), the right move is to retire the in-process-OpenTimer plan
and lean on OpenSTA-subprocess validation (ADR 0001) as the sole timing
reference. A follow-up ADR should consider libreda-sta or an in-house
walker if an in-process reference is still wanted later.

OpenTimer's strengths (in-process C++17, taskflow-based, MIT, fast for
the academic benchmarks it ships with) are real, but the input-pipeline
gaps are large enough that adopting it would mean owning a non-trivial
fork — the opposite of what a "lightweight in-process reference" is
supposed to be.

The Liberty parser is genuinely capable (Q1 passed cleanly on the 12 MB
SKY130 NLDM lib in 120 ms), so OpenTimer remains an option for future
narrow tasks like Liberty introspection, but not as the STA engine.

### Setup notes worth keeping

- OpenSTA bundles `gcd_sky130hd.{v,sdc,spef}` and `sky130hd_tt.lib.gz` —
  a cleaner SKY130 smoke-test fixture than anything we'd have produced
  from chipflow in the time we had.
- `~/.volare/volare/sky130/versions/c6d73a35f524070e85faff4a6a9eef49553ebc2b/sky130A/...`
  is the live SKY130 PDK already on this machine (chipflow installs it).
  No need to fetch it separately.



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
