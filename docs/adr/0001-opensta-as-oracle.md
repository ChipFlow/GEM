# ADR 0001 — OpenSTA as the timing correctness oracle and sole STA path

**Status:** Accepted. Scope expanded 2026-05-01 — see Decision §3 below.

## Context

Jacquard's current correctness validation for timing relies on its own CPU reference simulator (`--check-with-cpu`), which shares the Rust source tree, data structures, and parsers with the GPU simulation path. Representation bugs (e.g., hierarchical SDF prefix mismatch, inverter-collapse issues) have passed both paths silently because they affect both.

Historical regressions have been caught only by comparing against genuinely external tools — specifically CVC for functional simulation and, by implication, OpenSTA for timing. No format or tool inside Jacquard is currently treated as authoritative.

OpenSTA is widely deployed in open-source EDA (SKY130, OpenLane2, OpenROAD) and has the largest effective test surface of any open-source STA tool for the Liberty + SDF + Verilog + SPEF stack. It is licensed under GPL-3.0 and also sold commercially.

Jacquard requires permissive licensing for code linked into its binary (see [`../project-scope.md`](../project-scope.md)).

## Decision

OpenSTA is the ground-truth oracle for timing correctness **and the sole STA path used by Jacquard**.

1. In the shipped release, OpenSTA is never invoked from the `jacquard` runtime binary, and never linked. Subprocess invocation from CI pipelines, test harnesses, and the standalone `opensta-to-ir` preprocessing tool (see ADR 0006) is acceptable — GPL's reciprocal requirements do not cross a subprocess boundary ("mere aggregation") and so Jacquard's permissive licensing is preserved. Pre-release, a runtime subprocess invocation may exist as a contributor-ergonomics convenience (per ADR 0006); it is removed before release.
2. All timing, STA, and parser-related code paths are validated against OpenSTA on (a) a vendored subset of OpenSTA's own test corpus, and (b) representative Jacquard test designs.
3. **OpenSTA is also Jacquard's only STA path, not just its oracle.** ADR 0003 originally proposed an in-process reference STA via OpenTimer to complement this oracle role; the spike (`../spikes/opentimer-sky130.md`) found OpenTimer's input pipeline unfit for OpenROAD-flow outputs (commit `d002bde` superseded ADR 0003). The role OpenTimer would have played — providing per-DFF clock arrival, structured timing data for the IR, etc. — now sits with OpenSTA, called out of process via `opensta-to-ir`. OpenSTA is therefore a **required runtime dependency for any timing-aware Jacquard flow**, not just for CI validation.
4. Where Jacquard's output disagrees with OpenSTA's output past a declared tolerance, Jacquard is wrong until proven otherwise. Divergence is either fixed, explicitly justified in writing, or filed as a bug.

## Consequences

- OpenSTA is a **required runtime dependency** for timing-aware Jacquard flows (post §3 expansion), not merely a CI/validation dependency. Users running `jacquard sim --timing-ir ...` need a `.jtir` produced by `opensta-to-ir`, which subprocesses OpenSTA. Documented in `../why-jacquard.md`.
- Subprocess integration preserves Jacquard's permissive licensing (satisfies `project-scope.md`).
- "Oracle-diff clean" becomes a required CI gate for timing-related PRs, run nightly or pre-release (not per-PR — OpenSTA runs on large designs can be slow).
- OpenSTA bugs may produce false-positive divergences. The expectation is to file upstream rather than work around silently. A pinned OpenSTA version in CI avoids drift. With OpenSTA now also the only STA path (not just the oracle), upstream regressions land in users' hands too — pinning matters more than before.
- A vendored OpenSTA test corpus (or git submodule) is added to the repo as a fixture. Licensing of specific test inputs is verified per file before inclusion.
- No second STA tool to maintain. The original ADR 0003 proposal would have given Jacquard a permissive-licensed in-process reference; the spike showed that's not achievable today with OpenTimer. A future ADR may revisit libreda-sta or an in-house walker if an in-process reference is wanted.

## Links

- `../project-scope.md` — permissive-license constraint.
- `../timing-correctness.md` — principle P1, requirement R3.
- ADR 0002 — timing IR (the concrete diff format used for oracle comparison).
- ADR 0003 — **Superseded.** OpenTimer in-process reference; spike Q2 fail moved Jacquard to OpenSTA-only. See `../spikes/opentimer-sky130.md` for the spike outcome.
- `../why-jacquard.md` — user-facing consequence: OpenSTA as a runtime dependency.
