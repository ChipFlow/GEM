# ADR 0005 — OpenSTA vendoring and test-corpus strategy

**Status:** Accepted.

## Context

Under ADR 0001, OpenSTA is the ground-truth oracle for timing correctness, invoked as a subprocess. Phase 0 (`../plans/phase-0-ir-and-oracle.md`) requires:

- A reproducible, pinned OpenSTA reference so CI diffs are comparable run-to-run.
- Access to OpenSTA's test inputs for stress testing our OpenSTA-driven converters.
- Separately, a primary regression corpus representative of Jacquard's actual use cases.

Two questions were considered jointly: (a) how we pin / reference the OpenSTA codebase, and (b) how we use their test data.

On vendoring source: OpenSTA is licensed GPL-3.0. Copying its source into Jacquard's repository as committed code creates licensing ambiguity for a permissive-licensed project. Git submodules are conventionally treated differently — the parent repository pins a commit reference, does not incorporate the submodule's source into its own commits, and inherits no license obligations from the submodule's presence. This convention is widely relied on in permissive projects that depend on GPL tooling at arm's length.

On test data: OpenSTA's corpus exercises OpenSTA's concerns — Liberty parsing edge cases, SI-aware analysis, timing-check variants specific to its engine. Much of it does not exercise anything Jacquard does, and some of it exercises features Jacquard deliberately does not support. Using it as the primary regression corpus would optimise for the wrong target: our converters would be validated against files OpenSTA cares about, not files Jacquard actually encounters.

Its real value to Jacquard is as a **stress / robustness corpus**: a large bank of real-world-ish timing files that exercise parser edge cases and dialect variants. A converter that survives their entire corpus is more robust than one validated against a hand-curated subset.

## Decision

### Vendoring

- OpenSTA is vendored as a git submodule at `vendor/opensta/`.
- The submodule is **not built** from Jacquard's build. Jacquard's subprocess invocations use whatever OpenSTA binary is installed in the developer or CI environment.
- The submodule exists for two purposes only: (a) pinning a specific OpenSTA version for CI reproducibility, (b) providing in-tree access to its test corpus without redistribution.
- Licensing: by git-submodule convention, the submodule's GPL-3.0 licence does not extend to the parent repository. This is the standard interpretation; contributors redistributing binaries or compiled artefacts should nonetheless verify the interpretation applies to their specific jurisdiction and use.

### Test corpus split

Two corpora, two distinct roles:

- **Primary regression corpus** at `tests/timing_ir/corpus/`.
  - Jacquard-specific designs: SKY130 MCU SoC, NVDLA, AIGPDK examples, representative SDFs from the real Jacquard flow.
  - Small, curated, committed directly.
  - Run on every CI execution.
  - Exit criterion: every file converts cleanly and matches golden IR within declared tolerance.

- **Stress / robustness corpus** at `tests/timing_ir/stress/` as a manifest file listing paths into `vendor/opensta/<test-tree-subdir>/`.
  - Not committed as duplicated data; the manifest references submodule paths.
  - Large, whatever upstream maintains.
  - Run nightly or pre-release, not per-PR.
  - Exit criterion: no crashes, no hangs, no malformed IR. Numerical agreement with OpenSTA not required — this corpus is for robustness, not correctness.

### Copying from stress corpus into primary corpus

If a stress-corpus file exposes a bug, a minimal reproducer may be distilled and added to the primary corpus. When doing so:

- Verify the specific file's licence before copying. OpenSTA's overall GPL-3.0 licence does not imply every test input is GPL-3.0 — some test inputs are vendor-derived or public-domain.
- Prefer distilling a synthetic minimal reproducer over copying the original file wholesale.

## Consequences

- CI reproducibility: pinned submodule means we control when OpenSTA version changes land. Bumping the pin is an explicit, reviewable step.
- Repository size grows by OpenSTA's submodule size (multi-megabyte) but not by test-data duplication.
- Maintenance cadence: periodic submodule pin updates are a known maintenance item. Not frequent, but not zero.
- Primary regression corpus stays lean and directly relevant; developers can reproduce corpus-level failures locally without pulling the entire submodule.
- Stress-corpus failures are treated as bugs against our converter, never as bugs against OpenSTA's test inputs.
- Licensing posture is conventionally defensible; if stronger legal assurance is ever required, the submodule can be replaced by the external-install-only option (drop the submodule, rely purely on whatever OpenSTA is installed) at the cost of losing in-tree test access.

## Links

- `../project-scope.md` — licensing constraints.
- `../timing-correctness.md` — R3 (oracle-backed CI).
- ADR 0001 — OpenSTA as oracle (establishes the subprocess model).
- ADR 0002 — timing IR (the format being stress-tested).
- `../plans/phase-0-ir-and-oracle.md` — Phase 0 WS4 implements this split.
