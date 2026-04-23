# ADR 0003 — OpenTimer as in-process reference STA

**Status:** Pending Spike. Confirmation depends on the outcome of `../spikes/opentimer-sky130.md`.

## Context

Jacquard needs an in-process reference STA path to:

- Validate SDF-derived timing against an independent computation at load time and on demand (requirement R2 in `timing-correctness.md`).
- Provide exact per-edge arrival for top-K critical paths (requirement R4, pessimism-delta reporting).

OpenSTA (ADR 0001) is the ground-truth oracle but runs only as a subprocess — unsuitable for per-run, in-process checking. A linked alternative is needed.

Options surveyed:

- **OpenTimer** (MIT, C++17). Parses `.lib` / `.v` / `.spef` / `.sdc` directly. Won TAU Timing Analysis Contests (2014 1st, 2015 2nd, 2016 1st); industry "Golden Timer" for benchmark comparisons. Actively maintained (latest push 2025-12-26 as of this writing). Does not parse SDF — timing is computed from Liberty + parasitics.
- **libreda-sta** (Rust, permissive). Young framework, self-described as "basic components." Unknown whether it handles SKY130 Liberty robustly. Lower maturity risk than OpenTimer.
- **Tatum** (MIT, C++). Analysis engine only; does not parse Liberty/SDF/Verilog. Using Tatum would require supplying our own parsers, so it does not solve the problem directly.
- **In-house Rust walker.** Author-shared blind spots with Jacquard's main pipeline reduce the independence benefit.

## Decision

**Subject to the spike's success**, OpenTimer becomes Jacquard's in-process reference STA, integrated via C++ FFI (`bindgen` or equivalent).

- Linked directly; MIT licence satisfies `project-scope.md`.
- Computes timing from `.lib` + `.spef` independently of any SDF-derived path. This is an accepted (and arguably preferable) property: the reference path shares no parsing with Jacquard's SDF consumer, so a parse bug on either side is detectable rather than mutually masked.
- Emits timing IR (per ADR 0002) so its output is directly diffable against Jacquard's SDF-derived IR.

Spike criteria in `../spikes/opentimer-sky130.md`. On spike failure, fallback is to drop the in-process reference entirely and rely on OpenSTA subprocess validation (ADR 0001). This weakens per-PR feedback on timing correctness but is not fatal.

## Consequences

- C++ FFI dependency; `bindgen`-generated bindings; build complexity rises modestly.
- Direct linking preserves permissive licensing (MIT).
- Three-way cross-check becomes the default in CI: Jacquard (SDF path) vs OpenTimer (Liberty+SPEF path) vs OpenSTA (subprocess, full ground truth). Three-way disagreement localises bugs to SDF parse / delay model / tool issue cleanly.
- OpenTimer does not parse SDF. To use it in Jacquard's current flow, OpenLane2 (or equivalent) must produce SPEF alongside SDF. This plumbing change is tracked in the phase-0 plan.
- OpenTimer's maturity is measured in contest benchmarks, not SKY130/GF130 real-flow output. Spike must verify it handles our actual Liberty and SPEF. The spike is structured to fail fast if it does not.
- If OpenTimer is dropped post-spike, alternative in-process references (libreda-sta, in-house) can be revisited; this ADR would be superseded rather than amended.

## Links

- `../project-scope.md` — licensing constraint, validation constraint.
- `../timing-correctness.md` — requirement R2, requirement R4.
- `../spikes/opentimer-sky130.md` — spike and success criteria.
- ADR 0001 — OpenSTA oracle.
- ADR 0002 — timing IR (OpenTimer emits it).
