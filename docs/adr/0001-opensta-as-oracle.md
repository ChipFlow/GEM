# ADR 0001 — OpenSTA as the timing correctness oracle

**Status:** Accepted.

## Context

Jacquard's current correctness validation for timing relies on its own CPU reference simulator (`--check-with-cpu`), which shares the Rust source tree, data structures, and parsers with the GPU simulation path. Representation bugs (e.g., hierarchical SDF prefix mismatch, inverter-collapse issues) have passed both paths silently because they affect both.

Historical regressions have been caught only by comparing against genuinely external tools — specifically CVC for functional simulation and, by implication, OpenSTA for timing. No format or tool inside Jacquard is currently treated as authoritative.

OpenSTA is widely deployed in open-source EDA (SKY130, OpenLane2, OpenROAD) and has the largest effective test surface of any open-source STA tool for the Liberty + SDF + Verilog + SPEF stack. It is licensed under GPL-3.0 and also sold commercially.

Jacquard requires permissive licensing for code linked into its binary (see [`../project-scope.md`](../project-scope.md)).

## Decision

OpenSTA is the ground-truth oracle for timing correctness.

- In the shipped release, OpenSTA is never invoked from the `jacquard` runtime binary, and never linked. Subprocess invocation from CI pipelines, test harnesses, and the standalone `opensta-to-ir` preprocessing tool (see ADR 0006) is acceptable — GPL's reciprocal requirements do not cross a subprocess boundary ("mere aggregation") and so Jacquard's permissive licensing is preserved. Pre-release, a runtime subprocess invocation may exist as a contributor-ergonomics convenience (per ADR 0006); it is removed before release.
- All timing, STA, and parser-related code paths are validated against OpenSTA on (a) a vendored subset of OpenSTA's own test corpus, and (b) representative Jacquard test designs.
- Where Jacquard's output disagrees with OpenSTA's output past a declared tolerance, Jacquard is wrong until proven otherwise. Divergence is either fixed, explicitly justified in writing, or filed as a bug.

## Consequences

- OpenSTA becomes a build/CI-time dependency for validated timing work. Contributors running timing-related tests must have it installed or use CI.
- Subprocess integration preserves Jacquard's permissive licensing (satisfies `project-scope.md`).
- "Oracle-diff clean" becomes a required CI gate for timing-related PRs, run nightly or pre-release (not per-PR — OpenSTA runs on large designs can be slow).
- OpenSTA bugs may produce false-positive divergences. The expectation is to file upstream rather than work around silently. A pinned OpenSTA version in CI avoids drift.
- A vendored OpenSTA test corpus (or git submodule) must be added to the repo as a fixture. Licensing of specific test inputs is verified per file before inclusion.

## Links

- `../project-scope.md` — permissive-license constraint.
- `../timing-correctness.md` — principle P1, requirement R3.
- ADR 0002 — timing IR (the concrete diff format used for oracle comparison).
- ADR 0003 — OpenTimer (in-process permissive reference, complements the oracle).
