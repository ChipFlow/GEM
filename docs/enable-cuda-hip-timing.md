# Enabling --timing-vcd on CUDA/HIP Backends

**Status**: Implementation plan for Goal step 7

**Objective**: Enable GPU-side arrival time tracking and VCD output for CUDA and HIP backends, matching Metal capability.

## Current State

### Metal ✅ (Complete)
- Metal kernel (`kernel_v1.metal`) fully supports timing arrivals
- `arrival_state_offset` properly wired from Rust to GPU
- VCD output includes arrival time annotations
- Validation: inv_chain_pnr + MCU SoC both passing CVC comparison

### CUDA ❌ (In Progress)
- Kernel has `simulate_v1_noninteractive_timed_cuda` function defined in `csrc/kernel_v1.cu`
- **NOT exposed** to Rust via FFI bindings
- Rust side uses only `simulate_v1_noninteractive_simple_scan` (non-timed variant)
- Warning logged when `--timing-vcd` requested: "Timing constraints requested but CUDA timed kernel not yet wired"

### HIP ❌ (In Progress)
- Kernel code in `csrc/kernel_v1.hip.cpp` shares timed implementation with CUDA
- Same FFI binding gap: timed variant not exposed to Rust
- Uses same placeholder path as CUDA

## Implementation Steps

### 1. Expose CUDA Timed Kernel to Rust

**File**: `csrc/kernel_v1.cu`

```cuda
// Current: only simple_scan exposed to FFI
// extern "C" void simulate_v1_noninteractive_simple_scan_cuda(...)

// Add timed variant to FFI exports:
extern "C" void simulate_v1_noninteractive_timed_cuda(
    int num_blocks,
    int num_major_stages,
    const size_t *blocks_start,
    const uint32_t *blocks_data,
    uint32_t *sram_storage,
    uint32_t *sram_xmask,
    uint32_t *arrival_state,  // Input/output arrival time tracking
    uint32_t arrival_state_offset,  // Offset into state buffer
    const uint32_t *timing_constraints,  // Timing delay data (optional)
    int num_cycles,
    size_t state_size,
    uint32_t *state
);
```

**Status**: Function already exists in kernel, just needs FFI binding.

### 2. Update build.rs to Generate FFI Bindings

**File**: `build.rs`

Currently uses `bindgen()` to parse CUDA headers and generate Rust FFI bindings. The timed kernel signature must be included in the generated bindings.

```rust
// In build.rs ucc config:
// Make sure simulate_v1_noninteractive_timed_cuda is in the whitelist
let bindings = bindgen_builder
    .allowlist_function("simulate_v1_noninteractive_timed_cuda")
    .allowlist_function("simulate_v1_noninteractive_simple_scan_cuda")
    // ... other config ...
    .generate()
    .expect("Unable to generate bindings");
```

### 3. Wire Timed Kernel Call in Rust

**File**: `src/bin/jacquard.rs`, function `sim_cuda()`

Replace the simple_scan call (line 862) with conditional logic:

```rust
if script.timing_arrivals_enabled && timing_constraints.is_some() {
    // Use timed kernel with arrival state tracking
    let arrival_states = UVec::<u32>::new_zeroed(
        num_cycles * script.arrival_state_size() as usize,
        device
    );

    ucci::simulate_v1_noninteractive_timed_cuda(
        script.num_blocks,
        script.num_major_stages,
        &script.blocks_start,
        &script.blocks_data,
        &mut sram_storage,
        &mut sram_xmask,
        &mut arrival_states,
        script.arrival_state_offset,
        timing_constraints.as_ref().map(|v| &v[..]),
        num_cycles,
        script.effective_state_size() as usize,
        &mut input_states_uvec,
    );

    // Extract arrivals from arrival_states and append to input_states_uvec
    // (matching Metal's expand_states_for_arrivals pattern)
} else {
    // Fall back to simple_scan (non-timed)
    ucci::simulate_v1_noninteractive_simple_scan(...)
}
```

### 4. Add Arrival State Readback

Once timing kernel runs, arrival data must be extracted from GPU and written to VCD.

**Pattern** (from Metal):
- Expand state buffer upfront to include arrival section
- GPU kernel writes arrival times to this region during simulation
- Read back from GPU after simulation completes
- VCD writer accesses arrival data like any other signal

**File affected**: `src/bin/jacquard.rs` around line 700-750 (VCD writing logic)

### 5. Repeat for HIP

**File**: `csrc/kernel_v1.hip.cpp`

HIP kernel shares Metal's core logic via `kernel_v1_impl.cuh`. Follow same pattern:
1. Expose `simulate_v1_noninteractive_timed_hip` in FFI
2. Update `src/bin/jacquard.rs` function `sim_hip()` with conditional kernel call
3. Wire `arrival_state_offset` parameter

**Expected change**: ~20-30 lines in build.rs, ~40-50 lines in jacquard.rs sim_hip function

## Testing

### Unit Tests (in-kernel)
- CUDA kernel test: verify arrival times match expected gate delays
- HIP kernel test: same (shares implementation)
- Arrival state readback: ensure GPU→CPU transfer preserves timing data

### Integration Tests
1. Run `cargo test --lib --features cuda` with timing-enabled inv_chain
2. Compare CUDA timing VCD against Metal timing VCD
3. Both should match CVC reference within ±5% tolerance

### CI Integration
Add CUDA timing test to `.github/workflows/ci.yml`:
```yaml
cuda-timing:
  runs-on: ubuntu-latest
  steps:
    - # ... checkout, build ...
    - name: Run CUDA timing validation
      run: |
        cargo run --features cuda --bin jacquard -- sim \
          tests/timing_test/inv_chain_pnr/inv_chain.v \
          tests/timing_test/inv_chain_pnr/stimulus.vcd \
          cuda_output.vcd 1 \
          --sdf inv_chain.sdf \
          --timing-vcd

        # Compare against Metal baseline
        # (requires running metal variant separately or storing expected output)
```

## Risk Factors

1. **EventBuffer struct**: Currently causes compile errors when used with UVec on CUDA. The TODO (line 852 in jacquard.rs) mentions "EventBuffer doesn't impl Copy". This may require refactoring event handling for CUDA.

2. **GPU memory layout**: CUDA and Metal use different memory models. Ensure `arrival_state_offset` calculation matches both architectures.

3. **Kernel divergence**: If CUDA and HIP kernels diverge from Metal during this work, maintainability suffers. Prefer shared header patterns.

## Timeline Estimate

- **Step 1-2** (FFI exposure): 1-2 hours (straightforward bindgen config)
- **Step 3-4** (Rust integration): 2-3 hours (conditional logic + readback)
- **Step 5** (HIP): 1 hour (mirror CUDA pattern)
- **Testing**: 2-3 hours (integration tests + CI validation)

**Total**: ~8-12 hours (one development session)

## Success Criteria

- [ ] CUDA kernel call uses `simulate_v1_noninteractive_timed_cuda` when `--timing-vcd` flag set
- [ ] `arrival_state_offset` parameter properly threaded to CUDA/HIP kernels
- [ ] inv_chain_pnr timing VCD generated successfully on CUDA
- [ ] inv_chain_pnr timing matches Metal output within ±5%
- [ ] MCU SoC timing VCD generates (may differ due to size, but no crashes)
- [ ] CI test validates CUDA timing call path
- [ ] No regressions: existing tests (non-timed) still pass

## References

- Metal implementation (reference): `src/bin/jacquard.rs` lines 464-800
- CUDA kernel (timed variant): `csrc/kernel_v1.cu` lines 48-71
- HIP kernel: `csrc/kernel_v1.hip.cpp` (mirrors CUDA)
- Rust FFI binding: Generated in `target/*/uccbind/kernel_v1.rs` (build.rs artifact)
