# Jacquard CLI/API Audit

## Current Surface

### Subcommands

| Command | Purpose | Maturity |
|---------|---------|----------|
| `jacquard map` | Netlist → partition file (.gemparts) | Functional |
| `jacquard sim` | Batch VCD-in → VCD-out GPU simulation | Functional |
| `jacquard cosim` | Co-sim with SPI flash + UART (Metal only) | Functional |

### `jacquard map` options

| Option | Status | Notes |
|--------|--------|-------|
| `netlist_verilog` (positional) | OK | |
| `parts_out` (positional) | OK | |
| `--top-module` | OK | Auto-detect if omitted |
| `--level-split` | OK | Comma-separated thresholds |
| `--max-stage-degrad` | OK | Default 0 |
| `--xprop` | OK | Informational analysis |

### `jacquard sim` options

| Option | Status | Notes |
|--------|--------|-------|
| `netlist_verilog` (positional) | OK | |
| `gemparts` (positional) | OK | |
| `input_vcd` (positional) | OK | Should be PathBuf, currently String |
| `output_vcd` (positional) | OK | Should be PathBuf, currently String |
| `num_blocks` (positional) | **Pain point** | Users must know GPU SM/CU count. Should auto-detect or default. |
| `--top-module` | OK | |
| `--level-split` | **Pain point** | Must match map step exactly. Should be stored in .gemparts. |
| `--input-vcd-scope` | OK | |
| `--output-vcd-scope` | OK | |
| `--check-with-cpu` | OK | Debug/validation |
| `--max-cycles` | OK | |
| `--json-path` | **Workaround** | Display format sidecar — should use Verilog attrs instead |
| `--sdf` / `--sdf-corner` / `--sdf-debug` | OK | |
| `--xprop` | OK | |
| `--enable-timing` / `--timing-*` | OK | Post-sim analysis |
| `--liberty` | OK | |

### `jacquard cosim` options

| Option | Status | Notes |
|--------|--------|-------|
| `--config` (required) | OK | JSON testbench config |
| `--num-blocks` | OK | Has default (64) unlike `sim` |
| `--max-cycles` | OK | |
| `--flash-verbose` | OK | |
| `--clock-period` | OK | |
| `--check-with-cpu` | OK | |
| `--gpu-profile` | OK | Metal only |
| `--sdf` / `--sdf-corner` / `--sdf-debug` | OK | |

## Identified Gaps

### 1. Usability Pain Points

- **`num_blocks` is a positional required arg in `sim`** — users need GPU hardware knowledge.
  Should auto-detect from GPU query, or default to a sensible value per backend.
- **`--level-split` must be repeated between `map` and `sim`** — error-prone.
  The partition file should embed these parameters.
- **`input_vcd`/`output_vcd` are `String` not `PathBuf`** — inconsistent with other args.
- **No `--version` flag** — should show Jacquard version.
- **No `--quiet`/`--verbose` flags** — clilog is always active, no user control over verbosity.
- **No output format control** — always VCD. No option for compressed output, no signal filtering.
- **`--json-path` is a workaround** — gem_format/gem_args_width should come from Verilog attrs (now parsed).

### 2. Missing Features (vs. Commercial Tools)

#### Configuration & Workflow
- **No project/config file** — every run requires all args on command line.
  VCS has `synopsys_sim.setup`, filelists (`.f` files). Jacquard should support a config file.
- **No filelist support** — single netlist file only.
- **Partition params not embedded** — .gemparts doesn't store the level-split/xprop settings used.
- **No plusarg-style passthrough** — VCS has `+define+`, `+arg=val`. Useful for parameterizing testbenches.

#### Signal Selection & Output
- **No signal filtering** — output VCD contains all signals. VCS has `$dumpvars(N, scope)` for
  hierarchy depth control, and selective probing.
- **No output format options** — VCD only. Could support compressed VCD, or a more compact format.
- **No signal watchlist** — can't specify "only dump these signals".

#### Reporting & Diagnostics
- **No simulation summary** — no cycle count, wall-clock time, throughput, partition utilization.
  VCS prints simulation statistics at end.
- **No machine-readable output** — all output is human-readable log. No JSON/structured output
  option for CI/automation integration.
- **Assertion messages lack source location** — cell_attrs now available but not wired to messages.
- **No performance profiling summary** — cosim has `--gpu-profile` but sim doesn't.

#### Automation & Scripting
- **No Python API** — can only use as CLI. VCS has TCL/UCLI, cocotb integrates via VPI.
  Jacquard should expose a library API for Python bindings.
- **No regression/batch mode** — no multi-run support, no parameter sweep, no seed management.

#### Library vs Binary Separation
- **Almost everything is in `jacquard.rs` bin** — 1438 lines of business logic in the binary.
  `sim::setup` and `sim::vcd_io` are library code, but the actual simulation dispatch (GPU
  kernel invocation, VCD reading/writing, event processing) is all in the binary.
- **No reusable simulation API** — can't programmatically: load design, set inputs, step, read outputs.

### 3. Consistency Issues

- `sim` takes `num_blocks` as positional; `cosim` takes it as `--num-blocks` with default.
- `sim` takes VCD paths as positional Strings; other paths are PathBuf.
- `cosim` is Metal-only but this isn't enforced at compile time (cfg feature gating).

## Recommended Priority

### Quick Wins (improve current UX)
1. Auto-detect `num_blocks` from GPU (query SM/CU count, default to 2x)
2. Embed level-split/xprop in .gemparts so sim doesn't need them repeated
3. Add `--version` and `--verbose`/`--quiet` flags
4. Add simulation summary at end (cycles, wall-clock, throughput)
5. Fix String → PathBuf for VCD paths

### Medium Term (enable automation)
6. Signal filtering (`--dump-signals`, `--dump-depth`, `--dump-scope`)
7. Machine-readable output mode (`--output-format json`)
8. Wire source annotations to assertion/error messages
9. Config file support (TOML or similar)

### Longer Term (API foundation)
10. Extract simulation dispatch from binary into library API
11. Python bindings (PyO3) for scripting
12. Regression runner / batch mode
