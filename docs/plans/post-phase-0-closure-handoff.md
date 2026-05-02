# Handoff — Post-Phase-0-Closure (Pillar B + WS4 + WS5 done)

**Created:** 2026-05-02
**Predecessor:** [`post-cosim-models-handoff.md`](post-cosim-models-handoff.md) (now resolved)
**Working tree:** clean. `main` matches `origin/main` (8 commits pushed this session, on top of `d002bde` "Supersede ADR 0003").

## Goal & next-up

**Goal of this session:** Land the OpenSTA-out-of-process commitment in code and docs after the OpenTimer spike Superseded ADR 0003 — specifically Pillar B (clock-tree skew) end-to-end, WS5 (parser-success assertions, which turned out to already be shipped), and WS4 (opensta-to-ir regression corpus + CI). Plus reconcile every doc that still talked about OpenTimer-as-future-primary.

**Next session should pick up:** **WS2.4** (multi-corner CLI flag in `opensta-to-ir`) is the only Phase-0-flavoured item with concrete work left. ~3 days from a standing start, scope laid out below. The other open items (sky130-based corpus entries, peripheral I²C/SPI wiring) are blocked on external decisions, not code. After that the natural next step is Phase 1 proper — ADR 0008 structured timing output, per the post-phase-0 roadmap.

**Verification command:**
```sh
cargo test --lib && \
(cd crates/opensta-to-ir && cargo test) && \
(cd crates/timing-ir && cargo test) && \
cargo clippy --workspace --no-deps -- -D warnings && \
cargo fmt --check && \
(cd crates/opensta-to-ir && cargo fmt --check) && \
bash scripts/regenerate-corpus-goldens.sh && git diff --exit-code tests/timing_ir/corpus/
# Expect: 138 root + 23 opensta-to-ir + 14 timing-ir tests pass; no clippy/fmt diffs;
# regen reproduces the checked-in goldens byte-for-byte.
```

## Done this session

8 commits on top of `d002bde`, all pushed to `origin/main`:

| Commit | Subject | Notes |
|---|---|---|
| `c403cc8` | opensta-to-ir: emit per-DFF CLOCK_ARRIVAL records (Pillar B Stage 1) | Schema + Tcl + builder + diff machinery + tests |
| `6767c3e` | flatten: fold per-DFF clock arrival into setup/hold (Pillar B Stage 2) | DFFConstraint extension + `effective_setup_hold` |
| `4aee1cc` | docs: reconcile post-supersedure roadmap and Pillar B status | 9-doc sweep replacing OpenTimer-as-future-primary with OpenSTA-only |
| `89f7aad` | docs: reframe WS4 as opensta-to-ir golden-IR regression corpus | Decision: Option A (corpus diff vs goldens) over B (cxxrtl diff) and C (vs native parser) |
| `6183bee` | docs: mark WS5 (parser-success assertions) as already shipped | Both halves were already in code — just hadn't been doc'd as done |
| `90558bb` | ws4: opensta-to-ir golden-IR regression corpus + runner + regen | Seed entry `aigpdk_dff_chain` covers all 4 record types |
| `6997096` | ci: opensta-to-ir-tests job — runs corpus + cli + integration tests | New CI job; CUDD source build + cache; OpenSTA build cached on submodule SHA |
| `9e25bc2` | ci: install libfl-dev so OpenSTA's find_package(FLEX) can locate FlexLexer.h | Fix-up for first CI run; libfl-dev carries FlexLexer.h on Ubuntu, not flex |

## Phase 0 status snapshot

| WS | Title | Status | Notes |
|---|---|---|---|
| WS1 | IR schema | ✓ Complete | `508baaf`. Schema bumped 0.1→0.2 this session for additive ClockArrival table. |
| WS2.1 | OpenSTA subprocess + scaffold | ✓ Complete | Pre-session |
| WS2.2 | Interconnect delays in IR | ✓ Complete | `67210c0` (predecessor session) |
| WS2.3 | Setup/hold checks | ✓ Complete | `8343b14` |
| WS2.4 | Multi-corner CLI flag | ✗ Open | Producer + consumer + fixture work; ~3 days. Unblocking notes below. |
| WS3 | Delete hand-rolled SDF parser | ✓ Complete | Pre-session |
| WS4 | Diff harness + CI corpus | **✓ Complete this session** | Reframed `89f7aad`, implemented `90558bb`, CI hookup `6997096`+`9e25bc2`. Sky130-based corpus entries pending separately. |
| WS5 | Parser-success assertions | **✓ Complete (was already shipped)** | `6183bee` retroactively marked it done in the docs |
| Pillar B Stage 1 | ClockArrival IR emission | **✓ Complete this session** | `c403cc8` |
| Pillar B Stage 2 | DFFConstraint fold-in | **✓ Complete this session** | `6767c3e` |
| Pillar B Stage 3 | Bucketed per-DFF constraint packing | ⚠ Conditional | Land only if Stage 1+2 measurement on a real design shows per-word collapse pessimism is material |
| ADR 0003 | OpenTimer as primary in-process STA | ✗ Superseded | `d002bde` (predecessor). All consequent doc reconciliation landed `4aee1cc`. |

## Open follow-ups (priority-ordered)

### 1. WS2.4 — multi-corner CLI flag (~3 days)

Concrete scope, established this session:

**Producer (Tcl + Rust binary):**
- `crates/opensta-to-ir/tcl/dump_timing.tcl`: replace single `read_liberty` + hardcoded `CORNER 0 default tt 1.0 25.0` with OpenSTA's `define_corners` + per-corner `read_liberty -corner $name`. Existing arc/check/wire/clock-arrival walks already key by `(cell, …)`; need to wrap them in a per-corner loop and call `[edge arc_delays $arc -corner $c]`.
- `crates/opensta-to-ir/src/main.rs`: rework `--liberty PATH` to accept `--corner NAME=PATH[,V=…,T=…,P=…]` repeats. Validate at least one corner.
- `crates/opensta-to-ir/src/builder.rs`: today each ARC/SETUP_HOLD/INTERCONNECT/CLOCK_ARRIVAL line lands as one IR record with one `TimingValue`. Multi-corner needs builder dedupe — emit multiple lines per `(cell, driver, load, corner_index)` from Tcl, merge into one IR record carrying a `[TimingValue]` vector. Mechanical.

**Consumer (jacquard root):**
- Add `--timing-corner <NAME>` to `SimArgs`/`CosimArgs` in `src/bin/jacquard.rs`; resolve to an index by walking `ir.corners()`.
- Replace `flatten.rs::ir_corner0_max(...)` (used in ~5 sites) with `ir_corner_max(idx)`. Thread the resolved index through `load_timing_from_ir`.

**Fixture (the real prerequisite):** sky130 ships multi-corner Liberty (`tt_025C_1v80`, `ss_-40C_1v62`, `ff_125C_1v95`) already on disk via volare per `~/.volare/...`. Wire two of those against the existing DFF/chain integration tests. Synthetic-but-real, no external decision needed.

**Risk:** OpenSTA's Tcl `arc_delays -corner` flag exists but the exact syntax wasn't verified locally yet — could need ~1h of probing similar to the spike done for clock arrival in `c403cc8`.

### 2. Sky130-based corpus entries (small once unblocked)

`tests/timing_ir/corpus/aigpdk_dff_chain/` is the seed entry; sky130 entries (`inv_chain_pnr`, mcu_soc subset) are documented as pending in `tests/timing_ir/corpus/README.md` § Entries. The blocker is **CI sky130-Liberty install strategy** — likely volare, but it's a multi-GB install so a custom GitHub Actions setup-action wrapper would be ideal. Inputs already exist under `tests/timing_test/inv_chain_pnr/` and `tests/mcu_soc/data/`; once CI can resolve sky130 Liberty, just call the regen script.

### 3. WS4 stress corpus (Phase 1 work, per the README)

`tests/timing_ir/stress/manifest.toml` is intentionally empty. The runner doesn't exist yet either. Per `tests/timing_ir/stress/README.md`, this is Phase 1 work — the primary corpus is the per-PR gate; stress is for nightly fuzz / robustness against vendor/opensta's own test corpus.

### 4. Peripheral wiring for I²C / SPI (medium, blocked)

Same as predecessor handoff. Models are ported; mcu_soc fixture lacks the ports. Blocked on a fuller chipflow mcu_soc fixture or a different test design.

### 5. Phase 1 proper

Per `docs/plans/post-phase-0-roadmap.md`: Phase 1 is now **structured timing output (ADR 0008 required items)** + Phase 0 carryover. WS-P1.1 (the four ADR-0008 required outputs) is the centrepiece — symbolic violation messages, `--timing-report path.json`, `--timing-summary` text, per-DFF worst-slack ranking. ~2 weeks total. Phase 2 (Pillar C Tier 1 wire-delay-per-receiver, optional Pillar B Stage 3) follows.

## Critical context to know (carried forward)

These facts surfaced during this session and remain relevant:

- **Pillar B Stage 1+2 design contract:** per-pair CRPR is intentionally not modelled. What Jacquard consumes is per-DFF capture-side arrival, treating launch as 0-reference. Stage 3 (bucketed per-word constraint packing) is the lever if the launch=0 pessimism turns out to matter on a real design — measure first. Documented in `crates/timing-ir/schemas/timing_ir.fbs` ClockArrival comment, `crates/opensta-to-ir/tcl/dump_timing.tcl` header comment, and `docs/timing-model-extensions.md` Part B.
- **OpenSTA Tcl APIs that work:** `[all_registers -clock_pins]` returns Pin handles; `[$pin vertices]` → list, take `[lindex $vertices 0]`; `[::sta::vertex_worst_arrival_path $vertex {min|max}]` returns Path; `[$path arrival]` returns delay in **seconds** (multiply by 1e12 for ps, same as `arc_delays`). `find_timing -full_update` doesn't accept the flag in the current OpenSTA build; use `::sta::find_timing_cmd 1` directly. All in `dump_timing.tcl`.
- **DFFConstraint is host-only despite `#[repr(C)]`.** The GPU contract is `to_u32_pair` (which now calls `effective_setup_hold` to fold clock arrival). Adding `clock_arrival_ps: i16` grew the struct but doesn't affect the kernel.
- **`flatten.rs::DFFConstraint::effective_setup_hold`** (`src/flatten.rs:107-117`) is the fold-in primitive: `setup - clock_arrival_ps` and `hold + clock_arrival_ps`, clamped to `[0, u16::MAX]`.
- **`scripts/build-opensta.sh`** now honours a `CUDD_DIR` env var. macOS dev machines (with cudd via brew's mht208/formal tap) leave it unset and behaviour is identical to before; CI sets it after building cudd from source.
- **Ubuntu's flex package does NOT carry FlexLexer.h** — `libfl-dev` does. Upstream OpenSTA's `Dockerfile.ubuntu22.04` doesn't list it but the GitHub-hosted ubuntu-latest image needs the explicit dep. Captured in the CI `Install OpenSTA build deps` step.
- **Corpus golden binary `.jtir` files are deliberately checked in** despite the global `*.jtir` gitignore. The exception rule is `!tests/timing_ir/corpus/**/*.jtir` in `.gitignore:38`. The JSON sidecar (`expected.json` via `flatc --json`) is for PR-time review; the binary is what `corpus_designs_match_golden_ir` diffs against.
- **opensta-to-ir is outside the root cargo workspace.** `cargo test --lib` from the repo root does NOT run its tests. The new `opensta-to-ir-tests` CI job is the only automated gate. Locally run via `cd crates/opensta-to-ir && cargo test`.

## Verification

```sh
# Code + tests + lint
cargo build --features metal 2>&1 | tail
cargo test --lib 2>&1 | grep "test result"   # expect 138 passed
(cd crates/opensta-to-ir && cargo test 2>&1 | grep "test result")  # expect 4+3+1+11+4 across 5 binaries
(cd crates/timing-ir && cargo test 2>&1 | grep "test result")      # expect 12+2

cargo clippy --workspace --no-deps -- -D warnings
cargo fmt --check
(cd crates/opensta-to-ir && cargo fmt --check)

# Intra-doc markdown link check (CI's docs job)
python3 -c "
import re, sys
from pathlib import Path
errors = []
for md in Path('docs').rglob('*.md'):
    text = md.read_text()
    for m in re.finditer(r'\]\(([^)]+\.md(?:#[^)]*)?)\)', text):
        target = m.group(1).split('#')[0]
        if target.startswith(('http://', 'https://', 'mailto:')): continue
        if not (md.parent / target).resolve().exists():
            errors.append(f'{md}: {target}')
if errors: print(f'BROKEN: {errors}'); sys.exit(1)
print('Links clean.')
"

# Corpus regeneration is idempotent (no diff means goldens reproduce byte-for-byte)
bash scripts/regenerate-corpus-goldens.sh
git diff --exit-code tests/timing_ir/corpus/

# CI: opensta-to-ir-tests job runs on every push to main (run 25245508493 was the first green one)
gh run list --branch main --limit 3
```

## References

- [`post-cosim-models-handoff.md`](post-cosim-models-handoff.md) — predecessor (resolved)
- [`phase-0-ir-and-oracle.md`](phase-0-ir-and-oracle.md) — Phase 0 work breakdown (§ WS4, § WS5 marked done this session)
- [`post-phase-0-roadmap.md`](post-phase-0-roadmap.md) — schedules Phase 1+ work
- [`../timing-model-extensions.md`](../timing-model-extensions.md) — Part B Pillar B status banner + Stage 1+2 commit refs
- [`../adr/0001-opensta-as-oracle.md`](../adr/0001-opensta-as-oracle.md) — bumped from "oracle" to "sole STA path" status
- [`../adr/0007-timing-model-fidelity-roadmap.md`](../adr/0007-timing-model-fidelity-roadmap.md) — Pillar B forward-looking sentences refreshed
- [`../spikes/opentimer-sky130.md`](../spikes/opentimer-sky130.md) — spike outcome (Q2 fail → ADR 0003 Superseded)

## Files created / modified

**Created (this session):**
- `tests/timing_ir/corpus/aigpdk_dff_chain/{inputs/dff_chain.{v,sdc,sdf}, expected.{jtir,json}, manifest.toml}`
- `crates/opensta-to-ir/tests/corpus.rs`
- `scripts/regenerate-corpus-goldens.sh`
- `docs/plans/post-phase-0-closure-handoff.md` (this file)

**Modified (substantive):**
- `crates/timing-ir/schemas/timing_ir.fbs` — ClockArrival table, root field, SCHEMA_MINOR 1→2
- `crates/timing-ir/src/lib.rs` — schema version bump
- `crates/timing-ir/src/timing_ir_generated.rs` — flatc regen
- `crates/timing-ir/src/diff.rs` — ClockArrivalKey + diff_clock_arrivals + DiffReport.clock_arrivals
- `crates/timing-ir/tests/{diff,roundtrip}.rs` — TimingIRArgs initializers
- `crates/opensta-to-ir/src/{dump,builder}.rs` — ClockArrivalRecord + parser + builder
- `crates/opensta-to-ir/tcl/dump_timing.tcl` — CLOCK_ARRIVAL emission via vertex_worst_arrival_path
- `crates/opensta-to-ir/Cargo.toml` — toml + serde dev-deps for corpus runner
- `crates/opensta-to-ir/tests/{build_ir,dump_parser,opensta_integration}.rs` — ClockArrival tests
- `src/flatten.rs` — DFFConstraint.clock_arrival_ps + effective_setup_hold + fold-in in build_timing_constraint_buffer + load_timing_from_ir population + tests
- `scripts/build-opensta.sh` — CUDD_DIR env passthrough
- `.github/workflows/ci.yml` — opensta-to-ir-tests job (CUDD + OpenSTA + cargo test)
- `.gitignore` — `!tests/timing_ir/corpus/**/*.jtir` exception
- `tests/timing_ir/corpus/README.md` — manifest schema documented
- `docs/plans/{phase-0-ir-and-oracle,post-phase-0-roadmap,post-cosim-models-handoff}.md` — WS4 + WS5 status updates
- `docs/timing-model-extensions.md` — Part B Pillar B status banner, Stage 1+2 commit refs
- `docs/why-jacquard.md` — OpenTimer→OpenSTA sweep, "zero-skew" claim updated, OpenSTA-as-runtime-dep note
- `docs/adr/0001-opensta-as-oracle.md` — scope bumped to sole STA path
- `docs/adr/0002-timing-ir.md` — ADR 0003 link marked Superseded
- `docs/adr/0007-timing-model-fidelity-roadmap.md` — Pillar B context refreshed; Stage 1+2 marked landed
- `docs/adr/0008-structured-timing-output.md` — `--sta-cross-reference` flag renamed
- `docs/timing-correctness.md` — R2 marked deferred (in-process STA was the OpenTimer goal)
- `docs/README.md` — roadmap entry refreshed

---

**Resume in a new session with:**
```
/resume_handoff docs/plans/post-phase-0-closure-handoff.md
```
