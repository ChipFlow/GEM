# Jacquard

![CI](https://github.com/ChipFlow/Jacquard/actions/workflows/ci.yml/badge.svg)
![License](https://img.shields.io/badge/license-Apache--2.0-blue)
![Rust](https://img.shields.io/badge/rust-edition%202021-orange)

Jacquard is a GPU-accelerated RTL logic simulator. Like a Jacquard loom weaving patterns from punched cards, Jacquard maps gate-level netlists onto a virtual manycore Boolean processor and executes them on GPUs, delivering 5-40X speedup over CPU-based RTL simulators.

Jacquard builds on the excellent [GEM](https://github.com/NVlabs/GEM) research by Zizheng Guo, Yanqing Zhang, Runsheng Wang, Yibo Lin, and Haoxing Ren at NVIDIA Research. [ChipFlow](https://chipflow.io) extends their work with:

- **Metal backend** for Apple Silicon Macs (in addition to the original CUDA backend)
- **Liberty timing support** — load real cell delays from Liberty files (e.g. SKY130) for timing-annotated simulation
- **SDF back-annotation** — post-layout timing from Standard Delay Format files
- **Setup/hold violation detection** — both CPU and GPU-side checking
- **Significant performance optimizations** to the partition mapping pipeline
- **CI/CD** with automated testing across all three backends

### Timing simulation status

GPU-accelerated gate-level simulation with real cell timing. What ships
today (Phase 1 closed 2026-05-02; see [`docs/plans/post-phase-0-roadmap.md`](docs/plans/post-phase-0-roadmap.md)):

| Capability | Status |
|---|---|
| Liberty parsing | ✅ |
| SDF back-annotation via `opensta-to-ir` | ✅ |
| Per-DFF clock-arrival folding (Pillar B Stages 1+2) | ✅ |
| GPU-side setup/hold violation detection | ✅ Metal; CUDA + HIP detect, route through `process_events` is a follow-up |
| **Symbolic violation messages** (`top/cpu/regs[7][bit 22] [word=42]`) | ✅ Metal |
| **`--timing-report <path.json>`** structured end-of-run report | ✅ Metal |
| **`--timing-summary`** human-readable text summary | ✅ Metal |
| OpenSTA detection + version check | ✅ |
| Per-receiver wire delay (Pillar C Tier 1) | ❌ Phase 2 (blocked on ADR 0007) |
| Multi-corner SDF (`--sdf-corner`) | ⚠ Single corner (typ) only; WS2.4 open |

See [`docs/timing-violations.md`](docs/timing-violations.md) and
[`docs/why-jacquard.md`](docs/why-jacquard.md) for the full output
interface and positioning. [`CHANGELOG.md`](CHANGELOG.md) tracks
released and unreleased changes.

## Dependencies

**Required for building**: the [Rust toolchain](https://rustup.rs/) (2021 edition). A GPU SDK is required only for the backend you build against.

**Optional tooling** used by specific workflows:

| Tool | Used for | macOS (Homebrew) | Linux (Debian/Ubuntu) |
|---|---|---|---|
| `flatc` | regenerating timing-IR bindings when editing `crates/timing-ir/schemas/timing_ir.fbs` | `brew install flatbuffers` | `apt install flatbuffers-compiler` |
| `mdbook` | building docs locally | `brew install mdbook` | `cargo install mdbook` |
| OpenSTA | building the vendored `vendor/opensta/` for use by `opensta-to-ir` and the timing-correctness CI corpus | `brew bundle --file vendor/opensta/Brewfile` then run `scripts/build-opensta.sh` | see `vendor/opensta/Dockerfile.ubuntu22.04`, then run `scripts/build-opensta.sh` |

Contributors editing only Rust / C++ / kernel sources do not need `flatc` or OpenSTA; the IR bindings are checked in and the OpenSTA build is only required when running the timing-correctness regression corpus.

## Quick Start

```sh
git clone https://github.com/ChipFlow/Jacquard.git
cd Jacquard
git submodule update --init --recursive
```

### Build (Metal - macOS)

```sh
cargo build -r --features metal --bin jacquard
```

### Build (CUDA - Linux)

Requires CUDA toolkit installed.

```sh
cargo build -r --features cuda --bin jacquard
```

## Usage

Simulate a gate-level netlist with a VCD input waveform:

```sh
# Metal (macOS) - use NUM_BLOCKS=1
cargo run -r --features metal --bin jacquard -- sim design.gv input.vcd output.vcd 1

# CUDA (Linux) - set NUM_BLOCKS to 2x your GPU's SM count
cargo run -r --features cuda --bin jacquard -- sim design.gv input.vcd output.vcd NUM_BLOCKS

# With SDF timing back-annotation:
cargo run -r --features metal --bin jacquard -- sim design.gv input.vcd output.vcd 1 \
  --sdf design.sdf --sdf-corner typ
```

Partitioning (mapping the design to GPU blocks) happens automatically at startup.

**See [docs/usage.md](./docs/usage.md) for full documentation** including synthesis preparation, VCD scope handling, and troubleshooting.

## Documentation

Browse the full documentation [online](https://chipflow.github.io/Jacquard/) or build it locally with [mdbook](https://rust-lang.github.io/mdBook/):

```sh
mdbook serve   # opens at http://localhost:3000
```

## Limitations

- Only supports non-interactive testbenches (static VCD input waveforms)
- Synchronous logic only (no latches or async sequential logic)
- Clock gates must use the `CKLNQD` module from `aigpdk.v`

## Benchmarks

Pre-synthesized benchmark designs are in `benchmarks/dataset/` (git submodule). See [benchmarks/README.md](benchmarks/README.md) for instructions.

Available designs: NVDLA, Rocket, Gemmini.

## Citation

Jacquard builds on the GEM research. Please cite the original paper if you find this work useful.

``` bibtex
@inproceedings{gem,
 author = {Guo, Zizheng and Zhang, Yanqing and Wang, Runsheng and Lin, Yibo and Ren, Haoxing},
 booktitle = {Proceedings of the 62nd Annual Design Automation Conference 2025},
 organization = {IEEE},
 title = {{GEM}: {GPU}-Accelerated Emulator-Inspired {RTL} Simulation},
 year = {2025}
}
```

## License

Apache-2.0. See [LICENSE](./LICENSE) for details.
