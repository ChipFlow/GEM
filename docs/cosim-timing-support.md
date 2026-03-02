# Adding --timing-vcd Support to Cosim Mode

**Status**: Implementation plan for Goal step 8 (final step)

**Objective**: Enable arrival time readback in cosim mode so timing-annotated VCD can be produced without a separate `loom sim` replay. Completes timing validation feature parity with `loom sim`.

## Current State

### loom sim ✅ (Complete)
- Metal: `--timing-vcd` fully functional with arrival time readback
- CUDA/HIP: In progress (step 7 - kernel FFI bindings)
- Produces timing-annotated VCD with arrival times for all signals
- Validated against CVC reference simulator

### loom cosim ❌ (No timing support)
- Supports co-simulation with external testbenches (via Verilog VCD)
- **Does NOT support** `--timing-vcd` flag currently
- CPU-GPU synchronized stepping; peripheral models run on CPU
- Missing: `timing_arrivals_enabled`, `arrival_state_offset` threading

## Why This Matters

**Current workflow** (suboptimal):
```bash
# Step 1: Run cosim to get functional outputs
loom cosim --input stimulus.vcd --output functional.vcd

# Step 2: Run again with timing to get arrival times (slow!)
loom sim --input stimulus.vcd --output timed.vcd --sdf design.sdf --timing-vcd
```

**Desired workflow** (step 8):
```bash
# Single command produces both functional + timing outputs
loom cosim --input stimulus.vcd --output timed.vcd \
    --sdf design.sdf --timing-vcd
```

## Architecture Overview

### Current cosim_metal.rs Structure

```
CosimOpts { timing_vcd: bool }
    ↓
run_cosim()
    ↓
[Clock scheduling + cycle dispatch]
    ↓
cosim_evaluate_boomerang()
    ├─ GPU kernel dispatch (Metal)
    └─ Peripheral model stepping (CPU)
    ↓
[Output generation]
```

### Timing Data Flow (sim path - reference)

```
FlattenedScript::enable_timing_arrivals()
    ↓ Sets timing_arrivals_enabled=true, computes arrival_state_offset
    ↓
expand_states_for_arrivals() [expand input state buffer]
    ↓
GPU kernel writes arrival times
    ↓
extract_arrivals_from_states() [read back after kernel]
    ↓
VCD writer includes $var declarations for arrival signals
```

## Implementation Steps

### Step 1: Thread timing_vcd Flag Through Cosim

**File**: `src/bin/jacquard.rs`, function `cmd_cosim()`

```rust
// Current: No timing_vcd support in CosimArgs
// Add:
let timing_vcd: bool = args.timing_vcd;  // From CLI

// Pass to run_cosim
jacquard::sim::cosim_metal::run_cosim(
    &mut design,
    &config,
    &opts,
    &timing_constraints,
    timing_vcd,  // NEW parameter
);
```

**Related CLI changes**:
- Add `timing_vcd: bool` field to `CosimArgs` struct
- Require `--sdf` when `--timing-vcd` is specified (like sim mode)

### Step 2: Enable Timing in Design Setup

**File**: `src/sim/cosim_metal.rs`, function `run_cosim()`

```rust
// Early in run_cosim, after loading design:
if timing_vcd {
    if sdf_path.is_none() {
        eprintln!("Error: --timing-vcd requires --sdf");
        return;
    }
    design.script.enable_timing_arrivals();
}
```

**Rationale**: Same validation logic as `cmd_sim`; prevents nonsensical timing queries without delay data.

### Step 3: Expand State Buffers for Arrivals

**File**: `src/sim/cosim_metal.rs`, function `run_cosim()`, after initializing state buffers

```rust
// Current: state buffers are xprop-expanded only
let mut input_states = if script.xprop_enabled {
    expand_states_for_xprop(...)
} else {
    input_vcd_states.to_vec()
};

// NEW: Also expand for timing arrivals
if script.timing_arrivals_enabled {
    input_states = expand_states_for_arrivals(&input_states, &script);
}

let mut gpu_states: UVec<u32> = input_states.into();
```

**Challenge**: Cosim maintains continuous state vector across all clock cycles, unlike sim which pre-allocates all states. Must handle incremental state buffer expansion per cycle.

### Step 4: Wire Timing State Offset to GPU Kernel

**File**: `src/sim/cosim_metal.rs`, function `cosim_evaluate_boomerang()`

Current SimParams struct (used in kernel dispatch):

```rust
#[repr(C)]
struct SimParams {
    num_blocks: u64,
    num_major_stages: u64,
    // ... other fields ...
    arrival_state_offset: u64,  // ADD THIS FIELD
}
```

When building params for each cycle:

```rust
let params = SimParams {
    // ... existing fields ...
    arrival_state_offset: if script.timing_arrivals_enabled {
        script.arrival_state_offset as u64
    } else {
        0
    },
};
```

### Step 5: Extract Arrival Data After Simulation

**File**: `src/sim/cosim_metal.rs`, in output processing loop

After GPU kernel completes for each cycle:

```rust
// Current: Extract functional signals only
let cycle_signals = extract_functional_signals(&gpu_states, script, cycle_i);

// NEW: Also extract arrival times
let arrival_signals = if script.timing_arrivals_enabled {
    extract_arrival_signals(&gpu_states, script, cycle_i)
} else {
    Vec::new()
};

// Write both to output VCD
for (signal, value) in cycle_signals {
    writer.emit(signal, value);
}
for (signal, arrival) in arrival_signals {
    writer.emit(signal, arrival);
}
```

Helper function `extract_arrival_signals()`:

```rust
fn extract_arrival_signals(
    states: &[u32],
    script: &FlattenedScript,
    cycle_i: usize,
) -> Vec<(SignalId, u64)> {
    let arrival_base = cycle_i * script.effective_state_size() as usize
        + script.arrival_state_offset as usize;

    let mut results = Vec::new();
    for signal_id in 0..script.arrival_signal_count {
        let byte_offset = arrival_base + (signal_id / 4) as usize;
        let bit_offset = (signal_id % 4) * 16;  // 16-bit arrival per signal
        let arrival_ps = ((states[byte_offset] >> bit_offset) & 0xFFFF) as u64;
        results.push((signal_id, arrival_ps));
    }
    results
}
```

### Step 6: Update VCD Header with Timing Signals

**File**: `src/sim/vcd_io.rs`, function `setup_output_vcd()`

When `timing_arrivals_enabled`, add timing signal variables to VCD header:

```rust
// NEW: Add timing signal variables
if script.timing_arrivals_enabled {
    for signal_id in 0..script.arrival_signal_count {
        let signal_name = format!("{}__arrival_ps", get_signal_name(signal_id));
        writer.register_var("arrival", "wire", &signal_name);
    }
}
```

## Testing

### Unit Tests
- `test_cosim_with_timing`: Load design with SDF, enable timing, verify `arrival_state_offset` is properly set
- `test_cosim_arrival_extraction`: Verify arrival values are correctly extracted from state buffer

### Integration Test
```rust
#[test]
fn test_cosim_timing_vcd_generation() {
    // Use inv_chain_pnr design (simple reference)
    let design = load_inv_chain_design();
    let config = create_test_config();
    let opts = CosimOpts { timing_vcd: true, ... };

    // Run cosim with timing
    run_cosim(&mut design, &config, &opts, &timing_constraints, true);

    // Verify output VCD contains arrival times
    let vcd_content = read_vcd_output();
    assert!(vcd_content.contains("__arrival_ps"));

    // Compare arrival times against --sim-only baseline
    // Both should produce identical timing annotations
}
```

### End-to-End Test (CI)
```bash
# Single cosim run produces both functional + timing VCD
cargo run --features metal --bin jacquard -- cosim \
    --input tests/timing_test/inv_chain_pnr/stimulus.vcd \
    --output cosim_timed.vcd \
    --sdf tests/timing_test/inv_chain_pnr/6_final.sdf \
    --timing-vcd

# Compare against separate sim run
cargo run --features metal --bin jacquard -- sim \
    ... --timing-vcd

# VCD diff should show identical timing annotations
diff_timing_signals cosim_timed.vcd sim_timed.vcd
```

## Risk Factors

1. **State buffer layout complexity**: Cosim maintains running state; adding timing section requires careful offset calculation per cycle. Unlike sim which pre-allocates all state upfront, cosim grows incrementally.

2. **Peripheral timing interactions**: CPU-based peripheral models (SPI flash, UART) may not properly interact with GPU-computed arrival times. May need to sync peripheral state with arrival tracking.

3. **Performance**: Enabling timing expands state buffer by ~2-3× (if X-propagation also enabled: 4-5×). Cosim's incremental dispatch pattern may amplify this memory overhead.

## Timeline Estimate

- **Step 1-2** (CLI + enable): 1 hour
- **Step 3-4** (Buffer expansion + kernel wiring): 2 hours (careful offset handling)
- **Step 5-6** (Extraction + VCD output): 1-2 hours (parallelize with step 3)
- **Testing**: 2 hours (unit + integration)

**Total**: ~6-8 hours (requires careful testing)

## Success Criteria

- [ ] CosimArgs accepts `--timing-vcd` and `--sdf` flags
- [ ] `enable_timing_arrivals()` called when timing_vcd=true
- [ ] State buffers properly expanded for arrival storage
- [ ] `arrival_state_offset` correctly threaded to Metal kernel
- [ ] Arrival signals extracted per cycle and written to VCD
- [ ] VCD header includes arrival signal declarations
- [ ] inv_chain_pnr cosim + timing VCD produces same results as sim
- [ ] No regressions: existing cosim tests (without timing) still pass
- [ ] CI validates cosim timing path

## References

- Current cosim implementation: `src/sim/cosim_metal.rs` (full architecture)
- sim timing reference: `src/bin/jacquard.rs` lines 464-800
- State expansion helpers: `src/sim/vcd_io.rs` (expand_states_for_* functions)
- VCD writing: `src/sim/vcd_io.rs` (setup_output_vcd, emit_change functions)
