# Timing Correctness Requirements

**Status:** Draft — under review.

This document is the contract for Jacquard's timing correctness story. It defines *what* must be true; ADRs under `docs/adr/` define *how*; plans under `docs/plans/` execute them.

## Scope

Inherits constraints and non-goals from [`project-scope.md`](project-scope.md) — in particular the permissive-license requirement for linked code and the synchronous-only design assumption.

Addresses three known weaknesses from the April 2026 architecture review:

1. Timing-model pessimism from the packed-32 ALU (no single fix; needs refinement path for sign-off use).
2. Weak correctness observability (CPU reference and GPU kernel share Rust source, so representation bugs pass both sides).
3. Hand-rolled SDF parser fragility on real post-P&R output from production tools.

Out of scope (tracked elsewhere, addressed later): hard schema limits (no latches, static VCD), three-backend maintenance cost, GPU-side IO model scaling, silent-failure engineering patterns beyond parsers, partitioner non-determinism.

## Principles

### P1 — OpenSTA is the oracle

When Jacquard's results disagree with OpenSTA, Jacquard is wrong until demonstrated otherwise. In the shipped release, OpenSTA is never invoked from the `jacquard` runtime binary, and never linked (it is GPL; Jacquard stays permissive). Subprocess invocation from CI, test harnesses, and the standalone `opensta-to-ir` preprocessing tool is acceptable. During development before first release, a runtime subprocess invocation may exist as a contributor-ergonomics convenience (see ADR 0006) — this is explicitly interim and is removed before release. Divergence is never accepted silently: it is fixed, explicitly justified in-doc, or filed as a bug.

### P2 — No single parse path is its own reference

Every format we consume (SDF, Liberty, SPEF, Verilog) is validated against at least one third-party tool's parse of the same file. This is the primary defense against representation bugs that would otherwise affect Jacquard's primary and reference paths simultaneously.

### P3 — Fail loud on silent failures

Every parser and pipeline stage emits a success count (cells parsed, arcs matched, annotations applied). The test harness asserts thresholds. Zero-match is never silently acceptable. A pipeline that quietly succeeds with the wrong data is a bug.

### P4 — Multi-corner from day one

Any timing-value representation natively supports multiple PVT corners. Single-corner shortcuts become tech debt the moment commercial flows arrive; the shape is enforced upfront.

## Functional requirements

### R1 — Timing IR

A canonical intermediate representation (IR) for SDF-equivalent timing annotations exists. It is:

- Schema-versioned with compatible evolution rules.
- Lossless for the subset of information Jacquard consumes downstream.
- Preserves vendor-specific annotations as typed passthrough (never silently dropped).
- Multi-corner by default.
- Tags each arc with provenance: source tool, source file, and origin category (asserted / computed / defaulted).

Details: `docs/adr/0002-timing-ir.md`.

### R2 — In-process reference STA — *deferred*

The original requirement was an in-process STA engine that computes per-endpoint arrival/slack from Liberty + SPEF, linked directly (subject to the permissive-license constraint in `project-scope.md`), cross-checking Jacquard's SDF-derived timing at load time and on demand during sim.

Preferred implementation was OpenTimer; the SKY130 spike (`docs/spikes/opentimer-sky130.md`) found OpenTimer's input pipeline unfit for OpenROAD-flow outputs, and ADR 0003 was Superseded (commit `d002bde`). The cross-check role is now performed out of process by OpenSTA via `opensta-to-ir` (ADR 0001 — sole STA path); the in-process variant is parked until a fit-for-purpose permissive option appears (libreda-sta or in-house walker, both behind a future ADR).

### R3 — Oracle-backed CI

The OpenSTA test corpus is vendored (or submoduled) into the repository and used as a regression fixture. Every IR converter runs against the corpus; converter-to-converter diffs must be explained or fixed before merge.

Jacquard's own regression designs are run through OpenSTA (subprocess) and compared against Jacquard's output. Runs nightly or pre-release, not per-PR, due to runtime cost.

### R4 — Critical-path refinement reporting

For any sim run that reports timing violations, or for any user-requested critical-path report, top-K paths are reported with:

- Full per-stage path trace with per-edge delay.
- Pessimism delta: the gap between the packed-thread max arrival and the actual arrival along this specific path (exposes the magnitude of the packed-32 ALU's worst-case accounting).
- Provenance of each delay (SDF-asserted / Liberty-computed / defaulted).

### R5 — Private PDK testing

A private test track for commercial PDKs (initially GF130) exists, gated on environment variables such as `GF130_PDK_PATH`. Tests skip cleanly when PDK files are unavailable; CI runs with PDK access execute them. No PDK-derived artifacts are committed. Details: `docs/adr/0004-private-pdk-testing.md`.

## Non-functional requirements

### N1 — Startup parse speed

IR consumption at sim startup is effectively O(1) of IR size (binary, memory-mapped, zero-copy). The expensive SDF→IR conversion is a one-time preprocessing step, not an every-sim cost.

### N2 — Reproducibility

Given identical inputs and tool versions, every converter produces byte-identical IR. Tool version bumps may change output but must be explicit (recorded in IR metadata).

### N3 — Modularity

Each converter (SDF→IR, Liberty→IR, OpenSTA→IR, reference-STA→IR) is testable in isolation, without the full Jacquard build. Converters are separate crates or binaries. Consumers of IR need not know which converter produced it.

## Acceptance criteria — phase 0

Phase 0 (see `docs/plans/phase-0-ir-and-oracle.md`) is complete when:

1. Timing IR schema defined, with both a binary encoding and a JSON sidecar for human/CI diffs.
2. OpenSTA → IR converter implemented as a subprocess-driven tool.
3. Jacquard's existing SDF parser emits IR alongside its native representation.
4. CI harness runs both converters on the OpenSTA test corpus and reports structured diffs.
5. At least one representative Jacquard test design produces matching IR between both paths, within a declared tolerance.
6. Parser-success assertions (per P3) in place for the SDF and Liberty paths.

Phases 1 and beyond are planned at the start of each phase, not all up front.

## Open questions

Items not settled by this document; they resolve in ADRs, spike outcomes, or phase plans:

- Exact IR schema format (FlatBuffers / Cap'n Proto / other). Tracked in ADR 0002.
- ~~Whether OpenTimer handles SKY130 and GF130 Liberty robustly.~~ Resolved: spike failed Q2 on SKY130; ADR 0003 Superseded.
- Whether SPEF gets its own IR or is embedded in the timing IR. Deferred; likely separate.
- Whether the IR is Jacquard-local or shared across a broader tooling ecosystem. External decision; answer affects investment level and schema stability requirements.

## Supersession

This document supersedes the "±5% arrival tolerance" convention from `docs/timing-validation.md` once phase 0 ships. Until then, the existing convention remains in effect for designs not yet covered by oracle-backed CI.

## References

- `docs/simulation-architecture.md` — current pipeline.
- `docs/timing-simulation.md` — current timing-sim usage.
- `docs/timing-validation.md` — current validation methodology (to be superseded).
- `docs/adr/` — decisions executed through this document.
- `docs/plans/` — phased implementation.
- `docs/spikes/` — time-boxed experiments.

---

**Last updated:** 2026-04-23
