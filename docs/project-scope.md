# Jacquard — Project Scope and Guarantees

**Status:** Draft — under review.

This document states what Jacquard is for, what it is deliberately not for, and the constraints under which it is built. It is the top-level contract that scoped requirements docs (`timing-correctness.md`, future equivalents) inherit from.

If you are a contributor deciding whether a feature or change fits Jacquard, start here.

## Purpose

Jacquard is a GPU-accelerated gate-level simulator for synthesized digital circuits. It exists to make pre-silicon functional verification of synthesized designs substantially faster than CPU-based simulators, by mapping the design onto a GPU's massively parallel execution model.

It is a descendant of NVIDIA Research's GEM project, maintained by Rob Taylor and community contributors.

Jacquard ships with **AIGPDK**, a simple and-inverter standard cell library aligned to the GPU schema's internal representation. It also accepts Liberty-described cells from open-source PDKs (SKY130) and commercial PDKs (under a private test track; see ADR 0004).

## In scope

- Gate-level simulation of synthesized Verilog netlists against AIGPDK, SKY130, and similar Liberty-described cell libraries.
- Stimulus from static VCD input, from co-simulated GPU-resident peripheral models (SPI flash, UART, and similar), or from combinations of both for SoC-scale functional verification.
- Synchronous designs clocked by one or more clocks of known frequency.
- GPU-backed simulation across CUDA (NVIDIA), HIP (AMD), and Metal (Apple Silicon).
- Back-annotated timing simulation from SDF, including setup/hold violation detection.
- Correctness validation against permissive third-party simulators and STA tools.

## Non-goals

Jacquard does not aim to cover the following areas. Feature work requiring cross-cutting changes to the GPU schema or core architecture is declined on scope grounds unless this document is first amended. Contributions that *interface* Jacquard with external tools covering these areas are welcome where noted.

- **Mixed-signal or analog simulation.** Digital gate-level only. Interfacing to external analog / mixed-signal simulators via cosim hooks is in scope for future contribution.
- **RTL-level simulation.** Input is a synthesized netlist, not behavioural RTL. Synthesis (via Yosys or commercial tools) happens upstream.
- **Sign-off STA.** Jacquard performs functional simulation with setup/hold guardrails. Sign-off timing analysis is delegated to dedicated STA tools; Jacquard validates its results against them but does not aim to replace them.
- **Incremental or interactive editing of running simulations.** Each Jacquard run starts from a complete netlist and stimulus. Contributions that embed Jacquard's engine into interactive or incremental workflows (REPLs, debuggers, IDE integrations) as an external driver are welcome and considered outside this project's direct scope rather than non-goals.

## Constraints

### Licensing

All code linked into the Jacquard binary must be under a permissive license (MIT / Apache-2 / BSD-3 or equivalent). GPL tools may be invoked as subprocesses. This keeps Jacquard commercially usable by downstream integrators.

### Platforms

Jacquard commits to three GPU backends: CUDA, HIP, and Metal. A change that lands on one backend without a plan for the others is a regression in product surface. Feature-parity timing is negotiable; eventual feature-parity is not.

### Design assumptions baked into the architecture

These are structural properties of the current GPU schema:

- Sequential logic is currently edge-triggered and synchronous. Latches and async sequential logic are not modelled today — the GPU schema's scheduling assumes synchronous clocking. Extending it to support async approaches is open territory; contributions with a viable approach are welcome.
- Circuits fit the boomerang block shape: 8191-signal input/output and 4095 intermediate-pin limits per partition, 64 SRAM output groups. Very wide designs may require manual `--level-split` tuning.
- Numerics are 4-state at partition granularity (X-capable or not), not per-bit.

### Validation

Results are validated against at least one independent third-party tool per format. No single parse path (Jacquard's or otherwise) is its own reference. See `timing-correctness.md` for the detailed validation contract on timing.

## Stability

Treat the following as honest characterisations, not marketing claims:

- **Stable.** Core GPU simulation of AIGPDK designs. NVDLA / Rocket / Gemmini regression path.
- **Stable with caveats.** SKY130 flow; known to work on the MCU SoC reference design, with a history of PDK-specific issues resolved over time.
- **Evolving.** Timing simulation (arrival tracking, setup/hold violations). Multi-clock scheduling. X-propagation semantics. SDF parser is hand-rolled and has received multiple reactive fixes.
- **Experimental.** GPU-resident peripheral models. HIP-on-NVIDIA path exists primarily to unblock CI.
- **Planned.** Private commercial-PDK test track (GF130).

Contributors can expect stable-tier behaviour to remain stable across releases. Evolving and experimental tiers may change shape between releases; reasonable migration notes will be provided.

## Decision principles

When scope conflicts arise:

1. **Product surface before performance.** Jacquard's speed advantage is its value proposition; optimizations that compromise correctness, validation, or portability are declined.
2. **Permissive-license contributors win.** Any change that pushes Jacquard toward GPL-contamination is rejected; subprocess-based integrations with GPL tools are acceptable.
3. **Non-goals hold until amended.** Extending into latches, interactive stimulus, or RTL simulation is not a bug fix; it is a scope change and requires this document to be updated first.
4. **Validation is not optional.** New parsers, new formats, new simulation modes require an independent third-party reference at least for representative cases.
5. **Honest stability labels.** Moving a feature from "evolving" to "stable" requires evidence (regression coverage, oracle-backed validation, absence of known silent-failure modes), not just time in the codebase.

## References

- `README.md` — project overview, quick start.
- `CLAUDE.md` — repository conventions and architecture overview for contributors working with AI assistance.
- `docs/simulation-architecture.md` — internal pipeline and data structures.
- `docs/timing-correctness.md` — scoped contract for timing accuracy, validation, and IR requirements.
- `docs/adr/` — architectural decision records.
- `docs/plans/` — phased implementation plans.

---

**Last updated:** 2026-04-23
