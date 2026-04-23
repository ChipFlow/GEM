# ADR 0004 — Private PDK testing track

**Status:** Accepted. Plumbing tracked in the phase-0 plan.

## Context

Some contributors and operators have access to commercial PDKs (GlobalFoundries, TSMC, and others) under NDA or licensing agreements that prohibit public redistribution of PDK files. Whether a given contributor has access is itself typically under NDA and not publicly known.

Commercial PDK Liberty libraries are substantially richer and quirkier than open-source alternatives — they include cell variants, conditional timing arcs, vendor-specific annotations, and characterization detail not present in SKY130 or AIGPDK. Several parser bugs live only on commercial PDK output.

SKY130-only coverage is insufficient for a sim tool used on commercial flows, and adding commercial PDK files to a public repository is not an option regardless of who operates the project.

The standard industry pattern for testing against proprietary PDKs is environment-gated test suites: tests run when the contributor has licensed access, and skip cleanly when they don't.

## Decision

Establish a private PDK test track gated on per-PDK environment variables (e.g. `GF130_PDK_PATH`, `TSMC_PDK_PATH`, and similar — one per PDK).

- Tests check for the required env var(s) and skip with a clear "PDK not available" message when unset.
- When env vars point to a readable PDK directory, tests execute fully.
- Only the test harness, expected *structural* outputs, and IR fixtures (where the PDK vendor licensing permits) are committed.
- No PDK-derived artifacts (`.lib`, `.sdf`, `.spef`, characterization data) are committed to the public repository under any circumstances.
- CI runners with configured PDK access execute the private track; public PRs from non-licensed contributors see the private tests as `skipped`, not as failures. Which runners have access is determined by whoever operates CI; this ADR does not name specific organisations.

The timing IR (ADR 0002) makes this feasible: converter output and diff results can be checked in as fixtures where they contain no PDK-licensed data. Expected behaviour can be asserted in terms of IR structure rather than in terms of specific cell timings that would leak characterization data.

## Consequences

- Contributors without PDK access cannot locally reproduce PDK-specific bugs. They rely on maintainer CI for validation.
- A separate setup doc for licensed contributors is required (not public). Points at env-var configuration, test runner invocation, and PDK-file staging expectations.
- Fixture schema must be PDK-agnostic enough that structural assertions don't implicitly leak cell-characterization data. Review process must check new fixtures against this rule before merge.
- Bugs found via private PDK testing are, where possible, distilled into minimal public reproducers. The private track is not a place to park unreviewable tests — every private test should ideally surface a public fixture once the bug's essence is extracted.
- CI cost rises (licensed runners). Runs are nightly or pre-release rather than per-PR.

## Links

- `../project-scope.md` — validation constraint.
- `../timing-correctness.md` — requirement R5.
- ADR 0002 — timing IR (enables portable fixtures).
