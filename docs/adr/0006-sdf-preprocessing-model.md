# ADR 0006 — SDF preprocessing model and interim-to-release cutover

**Status:** Accepted.

## Context

Jacquard's hand-rolled SDF parser (`src/sdf_parser.rs`) has accumulated reactive maintenance over time — empty `()` delays, `(COND …)` pin specs, escape handling, edge-qualified timing checks, TIMINGCHECK-stripping workarounds for OpenLane2 output. Each production failure has been a one-off patch. The timing-correctness review flagged this as issue #3, and a native Rust grammar-based replacement is the Phase 3 deliverable.

Concurrently, ADR 0001 establishes OpenSTA as the timing correctness oracle (subprocess, never linked, GPL), and ADR 0002 introduces a timing IR that decouples parsing from consumption.

Two facts together shape the decision:

1. **No release pressure.** Release can happen after Phase 3 lands. We are not forced to keep the hand-rolled parser alive while waiting on Phase 3.
2. **Permissive-license constraint applies to the shipped binary.** Subprocess invocation of GPL tooling is acceptable — does not trigger reciprocal obligations — and during pre-release development, even in-runtime subprocess invocation does not violate the constraint because no runtime binary is being distributed.

Given these, maintaining the hand-rolled parser through Phase 0–2 is unnecessary. OpenSTA's mature dialect coverage can substitute, via subprocess, while we build toward a native Rust replacement at our own pace.

## Decision

### Phase 0

- **Delete `src/sdf_parser.rs`** and the SDF→Jacquard-internal-types code path. All paths that previously consumed SDF now consume timing IR.
- **Ship `opensta-to-ir`** as a standalone preprocessing tool that consumes Liberty + Verilog + SDF + SPEF + SDC and emits timing IR. Subprocess-based on OpenSTA. Production-quality: stable CLI, documented exit codes, clear diagnostics.
- **Canonical runtime path** is `jacquard sim --timing-ir <path>`, consuming pre-converted IR. This path works without OpenSTA on the user's machine — pre-converted IR is sufficient.
- **Interim ergonomic path**: during development (pre-release only), `jacquard sim input.sdf` subprocesses `opensta-to-ir` internally to produce IR on the fly. This is a contributor convenience, not a shipping feature. Flag exists in code as `pre-release only` with a clear comment tying back to this ADR.

### Phase 3

- **Native Rust SDF→IR converter** replaces the OpenSTA subprocess call inside `jacquard sim input.sdf`. Grammar-based (nom / pest), validated against OpenSTA on the corpus per ADR 0001.
- Lands **before first release**.

### Shipped release

- **No OpenSTA invocation from the `jacquard` runtime binary.** The native Rust converter handles SDF inputs directly.
- **`opensta-to-ir` remains** as an alternative preprocessing tool. Users who want OpenSTA-computed timing may use it; subprocess model preserves permissive licensing.

### Walk-back options (if assumptions change)

- **If OpenSTA dialect coverage proves insufficient** during Phase 0 — e.g., a current Jacquard-supported SDF fails to parse — add dialect shims to `opensta-to-ir`'s post-processing. Reinstating the hand-rolled parser is the last resort, not the first.
- **If the Phase 3 Rust rewrite stalls** — ship the first release with preprocessing-only (no `jacquard sim input.sdf` path), remove the interim subprocess, and land the native converter in a later release. No information lost; users preprocess manually. This is already the post-release shape for `opensta-to-ir`; it's only the `jacquard sim input.sdf` convenience that would be deferred.
- **If OpenSTA becomes unmaintainable or disappears** — the submodule pin (ADR 0005) remains authoritative for the integrated version. A forked submodule can maintain any necessary patches.

## Consequences

- Jacquard's repository stops carrying a hand-rolled SDF parser as a reactive-maintenance target. Bugs in SDF interpretation between Phase 0 and Phase 3 are OpenSTA's problem (upstream) or `opensta-to-ir` post-processing's problem, not Jacquard's core codebase's problem.
- Pre-release ergonomic one-step workflow for contributors is preserved.
- Contributors running Jacquard on a *new* design (no pre-converted IR) must have OpenSTA installed during Phase 0 through Phase 3. For existing primary-corpus designs, pre-converted IR is checked in; no OpenSTA needed.
- Release-time check is unambiguous: either the runtime subprocess is replaced by native code, or it is removed entirely. Both outcomes satisfy the permissive-licensing constraint for the shipped binary.
- Test corpus regenerable: if OpenSTA updates change IR output, golden files are regenerated deliberately (reviewable diff), not silently.

## Links

- `../project-scope.md` — licensing constraint, preprocessing-tools pattern.
- `../timing-correctness.md` — P1 (oracle), R1 (IR).
- ADR 0001 — OpenSTA as oracle (subprocess model).
- ADR 0002 — timing IR (format consumed).
- ADR 0005 — OpenSTA vendoring (submodule for reproducibility + stress corpus).
- `../plans/phase-0-ir-and-oracle.md` — WS2 productionisation, WS3 deletion + interim hook.
