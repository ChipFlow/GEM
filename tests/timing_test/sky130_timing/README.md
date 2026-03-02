# SKY130 Timing Test Suite

Small test circuits using SKY130 HD standard cells with analytically
verifiable timing properties. These serve as ground truth for validating
Jacquard's timing simulation (both CPU and GPU paths).

## Test Circuits

### 1. `inv_chain.v` - Inverter Chain
- **Circuit**: DFF -> 16 inverters (sky130_fd_sc_hd__inv_1) -> DFF
- **Logic function**: Identity (even number of inversions)
- **Expected combo delay**: 16 x inv_delay (from Liberty)
- **Expected arrival at capture DFF**: clk_to_q + 16 x inv_delay
- **Purpose**: Validates basic delay accumulation through a linear chain

### 2. `logic_cone.v` - Convergent Logic Cone
- **Circuit**: 4 DFFs -> tree of nand2/nor2/and2 gates -> DFF
- **Logic function**: Specific Boolean function of 4 inputs
- **Expected combo delay**: Critical path through deepest branch
- **Purpose**: Validates max-of-inputs arrival time propagation

### 3. `setup_violation.v` - Setup Time Violation
- **Circuit**: Same as inv_chain but designed to violate setup at tight clock
- **Expected**: TIMING PASSED at 10ns clock, TIMING FAILED at 1ns clock
- **Purpose**: Validates setup/hold checking

## Generating Reference Values

The expected timing values are computed analytically from the SKY130 HD
Liberty file (sky130_fd_sc_hd__tt_025C_1v80.lib). Key values at typical corner:

| Parameter | Value (ps) |
|-----------|-----------|
| inv_1 tpd (rise) | ~28 |
| inv_1 tpd (fall) | ~18 |
| nand2_1 tpd | ~30-40 |
| and2_1 tpd | ~50-60 |
| dfxtp_1 clk->Q | ~310 |
| dfxtp_1 setup | ~80 |
| dfxtp_1 hold | ~-40 |

Note: Actual values depend on load capacitance and input transition time.
The defaults in `TimingLibrary::default_sky130()` are approximate.

## Pre-Layout (Liberty-Only) Timing Validation

To validate timing with pre-layout designs (synthesized but not placed/routed):

### Generate Liberty-Only SDF Files

```sh
# Generate SDF with Liberty cell timing (no P&R routing parasitics)
python3 gen_liberty_sdf.py inv_chain.v
python3 gen_liberty_sdf.py logic_cone.v
```

This creates `inv_chain.sdf` and `logic_cone.sdf` with delay values extracted
from the SKY130 Liberty file. Pre-layout SDF has no detailed routing delays,
only combinational path delays and setup/hold times from library timing models.

### Run with CVC

```sh
# Run with CVC (requires open-src-cvc Docker image: loom-cvc)
cvc64 +typdelays tb_inv_chain.v inv_chain.v
./cvcsim

cvc64 +typdelays tb_logic_cone.v logic_cone.v
./cvcsim
```

### Compare Loom vs CVC

Use the comparison script in the parent directory to validate that Loom's
GPU timing simulation matches CVC's reference timing:

```sh
bash ../inv_chain_pnr/../compare_timing.py cvc_output.vcd loom_output.vcd
```

## Files

- `inv_chain.v` - Inverter chain (pre-layout RTL with SKY130 cells)
- `logic_cone.v` - Convergent logic tree (pre-layout RTL with SKY130 cells)
- `inv_chain.vcd` - Primary input stimulus for inv_chain
- `logic_cone.vcd` - Primary input stimulus for logic_cone
- `gen_liberty_sdf.py` - Script to generate Liberty-only SDF from RTL
- `tb_inv_chain.v` - CVC testbench for inv_chain timing measurements
- `tb_logic_cone.v` - CVC testbench for logic_cone timing measurements
- `*.sdf` - Generated Liberty-only SDF files (post-generation)
