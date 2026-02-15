# Loom

Loom is a GPU-accelerated RTL logic simulator. Like a Jacquard loom weaving patterns from punched cards, Loom maps gate-level netlists onto a virtual manycore Boolean processor and executes them on GPUs, delivering 5-40X speedup over CPU-based RTL simulators.

Loom builds on the excellent [GEM](https://github.com/NVlabs/GEM) research by Zizheng Guo, Yanqing Zhang, Runsheng Wang, Yibo Lin, and Haoxing Ren at NVIDIA Research. [ChipFlow](https://chipflow.io) extends their work with:

- **Metal backend** for Apple Silicon Macs (in addition to the original CUDA backend)
- **Liberty timing support** - load real cell delays from Liberty files (e.g. SKY130) for timing analysis and CPU-based timing simulation. GPU timing-annotated simulation is on the roadmap (see below).
- **Significant performance optimizations** to the partition mapping pipeline
- **CI/CD** with automated testing across both backends

### Roadmap: Timing Simulation

The goal is GPU-accelerated gate-level simulation with real cell timing - a first for open source. Current status:

| Component | Status |
|-----------|--------|
| Liberty file parsing | Done - loads SKY130 HD cell delays |
| Gate delay computation | Done - per-AIG-pin delays from Liberty |
| CPU timing simulation | WIP - basic arrival time propagation works, hold checking needs fixes |
| GPU timing simulation | Not started - delay buffers not yet passed to GPU kernel |
| SKY130 timing test suite | WIP - small test circuits in `tests/timing_test/sky130_timing/` |

Next steps:
1. Fix CPU timing sim hold violation false positives (zero-delay paths through AIG inverter collapse)
2. Validate CPU timing against analytical expectations using SKY130 test circuits
3. Pass `gate_delays` buffer to GPU kernel and implement delay accumulation in the shader
4. Compare GPU timing results against validated CPU reference

## Quick Start

Requires the [Rust toolchain](https://rustup.rs/).

```sh
git clone https://github.com/ChipFlow/GEM.git
cd GEM
git submodule update --init --recursive
```

### Build (Metal - macOS)

```sh
cargo build -r --features metal --bin cut_map_interactive
cargo build -r --features metal --bin metal_test
```

### Build (CUDA - Linux)

Requires CUDA toolkit installed.

```sh
cargo build -r --features cuda --bin cut_map_interactive
cargo build -r --features cuda --bin cuda_test
```

## Usage

Loom operates in two phases:

1. **Map** your synthesized gate-level netlist to a `.gemparts` file (one-time cost):

```sh
cargo run -r --features metal --bin cut_map_interactive -- design.gv design.gemparts
```

2. **Simulate** with a VCD input waveform:

```sh
# Metal (macOS) - use NUM_BLOCKS=1
cargo run -r --features metal --bin metal_test -- design.gv design.gemparts input.vcd output.vcd 1

# CUDA (Linux) - set NUM_BLOCKS to 2x your GPU's SM count
cargo run -r --features cuda --bin cuda_test -- design.gv design.gemparts input.vcd output.vcd NUM_BLOCKS
```

**See [usage.md](./usage.md) for full documentation** including synthesis preparation, VCD scope handling, and troubleshooting.

## Limitations

- Only supports non-interactive testbenches (static VCD input waveforms)
- Synchronous logic only (no latches or async sequential logic)
- Clock gates must use the `CKLNQD` module from `aigpdk.v`

## Benchmarks

Pre-synthesized benchmark designs are in `benchmarks/dataset/` (git submodule). See [benchmarks/README.md](benchmarks/README.md) for instructions.

Available designs: NVDLA, Rocket, Gemmini.

## Citation

Loom builds on the GEM research. Please cite the original paper if you find this work useful.

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

See [LICENSE](./LICENSE) for details.
