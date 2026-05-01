# ADR 0008 — Structured timing output as first-class deliverable

**Status:** Proposed.

## Context

Jacquard produces timing information today through three channels: timed VCD (`--timing-vcd`), per-violation `clilog::warn!` messages on stderr, and an in-process `SimStats` counter. The `why-jacquard.md` analysis identifies a gap between the timing data Jacquard *has* internally and the answers users actually need from a flow:

| User question | Today |
|---|---|
| Did my workload trip any violations? | `SimStats` counts (in-process API only) |
| Which DFFs nearly missed timing? | Not extractable without parsing stderr |
| Show me arrival distribution per signal | Reconstructable from --timing-vcd via post-processing only |
| Which DFF was that violation on? | State-word index + manual lookup |
| What path caused the worst arrival? | Not available |
| Run this in CI and fail if any violation | Possible only via stderr grep |

The most acute problem: stderr violation messages identify a *state-word index*, not a signal name. Mapping back to "which DFF, which path" requires manual investigation. On a violating design the message volume can be enormous (one warning per word per cycle per type). The data needed to do better — hierarchical signal names, DFF instance paths, per-DFF arrival distributions — already exists in the netlistdb and event buffer; it is simply not surfaced in usable form.

This ADR is about making Jacquard's timing output **useful in a real flow** rather than merely *produced*. The substantive work in ADR 0007 (model fidelity) is wasted if no one can extract the answers.

The full design analysis is in `docs/why-jacquard.md`, "Output interface" section.

## Decision

Treat structured, machine-readable timing output as a first-class shipping deliverable, not an optional improvement. Land the work in priority order, where priority is set by *user impact per implementation cost* not by technical interest.

### Required outputs

The following are required for Jacquard to be considered usable for vector-driven timing analysis in a real flow. They land **before** any further fidelity work past ADR 0007 Pillar B Stage 1+2.

1. **Symbolic violation messages.** Replace state-word indices with hierarchical signal names in stderr violation output. Mapping data already exists in netlistdb. Cost: contained edit in `src/event_buffer.rs:305-338` plus name-resolution helper. Highest UX impact per LoC of any improvement on this list.

2. **`--timing-report <path.json>`.** Structured JSON document at end-of-run containing:
   - Per-DFF worst arrival, worst slack, violation count over the run.
   - Per-cycle violation list (cycle, signal name, hierarchical path, arrival, constraint, slack).
   - Aggregate stats: total violations, distribution buckets, peak arrival per clock domain.
   - Per-signal activity summary: transition count, average/max arrival, idle cycles.
   - Run metadata: clock period, SDF/IR file, design hash, vector source.

   Required for CI integration and any downstream tooling. Schema versioned; additive extension policy mirrors `crates/timing-ir`.

3. **`--timing-summary`.** Fast text summary, no VCD. Designed for scripts and human inspection of long runs. Contents:
   - Vectors run, clock period, corner.
   - Setup/hold violation totals.
   - Worst-slack DFF (setup and hold) with hierarchical path.
   - Peak arrival per writeout vs clock budget, with margin percentage.

   Cost: trivial wrapper over (2)'s data.

4. **Per-DFF worst-slack ranking.** Top-N DFFs by closest-to-violation slack across the entire run, even when no violation occurred. Surfaces "where am I close to the edge" without requiring a violation to actually trip. Output as part of (2) and (3); also accessible via a dedicated `--worst-slack-n N` flag for quick inspection.

### Optional / later outputs

The following are higher-value-but-lower-priority. They land after the four required items above, in any order driven by user demand.

5. **`--arrival-histogram <pattern>`.** Per-signal arrival histogram dump for matched signal patterns, as JSON or CSV. Foundation for activity-based power analysis.

6. **`--sta-cross-reference <opentimer-paths.txt>`.** Cross-reference OpenTimer's critical-path report against observed worst arrivals. Closes the loop between vector-driven and static analysis. Coverage-style "of the top-N STA paths, which were exercised, and at what observed arrival."

7. **Path-back-trace from worst-arrival DFF.** Given a flagged DFF, walk the max-of-fanin chain backward to the source AIG pin / primary input, emitting the path with per-edge contribution. Most expensive item on this list; only useful once the cheaper items are in place.

### Backward compatibility

- All new outputs are opt-in via flags. Existing stderr behaviour and `--timing-vcd` semantics are unchanged.
- Symbolic violation messages (item 1) **do** change existing stderr format. This is intentional: the current state-word-index format is not a stable contract and is not consumed by any known automation. Format change documented in changelog at land time.

### Output stability contract

- The `--timing-report` JSON is a stable consumer-facing format. Schema versioned. Additive-only extensions per the IR convention; breaking changes require a major version bump and a transition period.
- `--timing-summary` is human-readable and explicitly **not** stable for parsing. Tools should consume the JSON.
- Stderr violation messages remain human-oriented; tools should not parse them.

## Consequences

- Jacquard becomes usable in CI without bespoke stderr parsing. Existing users who scrape stderr will need to migrate to the JSON report; the migration window is the release in which symbolic messages land.
- The `SimStats` in-process API gains a public counterpart: end-of-run JSON. This raises the bar for changes to either — they must agree.
- Documentation gains a "Jacquard timing report format" reference page. Sample reports from the corpus designs are checked in to `tests/timing_ir/corpus/` alongside golden IR.
- The `why-jacquard.md` positioning becomes truthful: the user-facing claim "vector-driven setup/hold answers at GPU scale" is backed by an interface that delivers them.

## Walk-back options

- **If the JSON schema causes consumer-tooling friction**, the format may be extended additively but not narrowed. Existing consumers must continue to work. If a fundamental rethink is required, ship a v2 alongside v1 with a deprecation window.
- **If symbolic name resolution is too slow at scale** (millions of DFFs, very long runs), the resolution step becomes opt-in via flag, with the existing state-word-index format retained as a fast-path default. No evidence yet that this is a problem; treated as a deferred consequence.
- **If users specifically want the path-back-trace (item 7)** before the cheaper items are scheduled, it can be promoted, but only once items 1–4 are in place. Path-back-trace without symbolic names is unusable.

## Priority and effort estimate

| Item | Effort | Blocks | User impact |
|---|---|---|---|
| 1. Symbolic violations | 1–2 days | Nothing | High (turns stderr from noise to signal) |
| 2. JSON report | 3–5 days | CI integration | High |
| 3. Text summary | 1 day (after #2) | Human dashboards | Medium |
| 4. Worst-slack ranking | 1–2 days (folds into #2) | "Am I close?" | High |
| 5. Arrival histogram | 3–5 days | Power analysis | Medium |
| 6. STA cross-ref | 1 week | Vector coverage report | Medium |
| 7. Path-back-trace | 2–3 weeks | Forensics | Lower-frequency-but-high-value |

Items 1–4 are a single workstream, ~2 weeks total. They constitute the "Jacquard is now usable" bar. Items 5–7 are scheduled per user demand after that.

## Links

- `../why-jacquard.md` — positioning analysis and full output-interface design.
- `../timing-violations.md` — current violation detection mechanics; updated to describe new outputs once they land.
- `../timing-validation.md` — validation tolerances; will reference the JSON report format for golden comparisons.
- `../adr/0002-timing-ir.md` — IR schema versioning policy that the JSON report mirrors.
- `../project-scope.md` — output stability constraints that apply to any user-facing format.
