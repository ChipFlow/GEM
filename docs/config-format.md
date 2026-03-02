# Jacquard Configuration File Format

## Overview

Jacquard supports an optional `jacquard.toml` project configuration file that stores
design parameters, mapping settings, and simulation options. This eliminates the need
to pass all arguments on every command invocation.

**Key principles:**
- CLI arguments always override config file values
- All paths are relative to the config file's directory
- The config file is optional; all existing CLI workflows continue to work
- Auto-discovery: Jacquard searches CWD and parent directories for `jacquard.toml`

## File Discovery

When no `--config` flag is given, Jacquard searches for `jacquard.toml` starting
from the current directory and walking up to parent directories (similar to how
`cargo` discovers `Cargo.toml`). If found, it is loaded automatically.

Explicit path: `jacquard --config path/to/jacquard.toml map`

## Format

TOML was chosen over JSON/YAML because:
- Human-readable and easy to edit (unlike JSON's strict quoting)
- Already familiar to Rust developers (Cargo.toml)
- Supports comments (unlike JSON)
- Strong typing with clear section boundaries

## Sections

### `[design]` — Shared design parameters

Parameters shared across all subcommands.

```toml
[design]
netlist = "gatelevel.gv"        # Gate-level Verilog path
top_module = "my_top"           # Top module name (auto-detected if omitted)
liberty = "path/to/lib.lib"     # Liberty timing library (optional)
```

### `[map]` — Partition mapping settings

Parameters for `jacquard map`. The `output` field defaults to `<netlist>.gemparts`
if not specified.

```toml
[map]
output = "result.gemparts"      # Output partition file path
level_split = [20, 40]          # Pipeline stage thresholds
max_stage_degrad = 0            # Max degradation layers (default: 0)
xprop = true                    # Enable X-propagation analysis
```

### `[sim]` — Simulation settings

Parameters for `jacquard sim`.

```toml
[sim]
gemparts = "result.gemparts"    # Partition file (defaults to map.output)
input_vcd = "input.vcd"         # Input VCD path
output_vcd = "output.vcd"       # Output VCD path
num_blocks = 128                # GPU blocks (omit for auto-detect)
input_vcd_scope = "tb.dut"      # Input VCD scope path
output_vcd_scope = "tb.dut"     # Output VCD scope path
max_cycles = 1000               # Cycle limit
check_with_cpu = false          # Verify against CPU baseline
xprop = true                    # Enable X-propagation
json_path = "display.json"      # Display format strings (legacy)
```

### `[sim.sdf]` — SDF timing back-annotation

```toml
[sim.sdf]
file = "design.sdf"             # SDF file path
corner = "typ"                  # min, typ, or max
debug = false                   # Enable SDF debug output
```

### `[sim.timing]` — Post-simulation timing analysis

```toml
[sim.timing]
enabled = true                  # Enable timing analysis
clock_period = 1000             # Clock period in picoseconds
report_violations = true        # Report all violations (not just summary)
```

### `[cosim]` — Co-simulation settings

Co-simulation uses the existing testbench JSON config for peripheral definitions
(flash, UART, GPIO mappings). The TOML config references the JSON file and provides
overrides for common parameters.

```toml
[cosim]
config = "sim_config.json"      # Testbench JSON config path
num_blocks = 64                 # GPU blocks
max_cycles = 500000             # Cycle limit
clock_period = 40000            # Clock period in ps
```

## Complete Example

```toml
# jacquard.toml — NVDLA benchmark project

[design]
netlist = "benchmarks/dataset/nvdlaAIG.gv"

[map]
output = "benchmarks/nvdla.gemparts"

[sim]
input_vcd = "benchmarks/dataset/nvdla.pdp_16x6x16_4x2_split_max_int8_0.vcd"
output_vcd = "benchmarks/nvdla_output.vcd"
```

With this config, the workflow becomes:

```bash
# Instead of: jacquard map benchmarks/dataset/nvdlaAIG.gv benchmarks/nvdla.gemparts
jacquard map

# Instead of: jacquard sim benchmarks/dataset/nvdlaAIG.gv benchmarks/nvdla.gemparts \
#   benchmarks/dataset/nvdla...vcd benchmarks/nvdla_output.vcd --num-blocks 128
jacquard sim
```

## Merging Rules

1. CLI positional/flag arguments override config file values
2. If a required value is missing from both CLI and config, an error is reported
3. `[map].level_split` and `[map].xprop` are also embedded in the `.gemparts` file;
   the config file provides the initial values, but `jacquard sim` reads them from
   the partition file at runtime
4. `[sim].gemparts` defaults to `[map].output` if both are in the same config file

## Path Resolution

All paths in the config file are resolved relative to the directory containing
`jacquard.toml`. Absolute paths are used as-is.

```toml
# If jacquard.toml is in /project/
[design]
netlist = "build/gatelevel.gv"  # → /project/build/gatelevel.gv
liberty = "/opt/pdk/lib.lib"    # → /opt/pdk/lib.lib (absolute, unchanged)
```
