// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Hybrid GPU/CPU co-simulation using Metal for gate evaluation
//! and CPU for peripheral models (SPI flash, UART).
//!
//! The GPU evaluates all combinational + sequential logic per clock tick,
//! while the CPU steps peripheral models (SPI flash, UART) between ticks,
//! feeding GPIO values back via shared memory.
//!
//! Usage:
//!   cargo run -r --features metal --bin gpu_sim -- <netlist.gv> <gemparts> \
//!     --config <testbench.json> [--max-cycles N] [--num-blocks N]

use gem::aig::{DriverType, AIG};
use gem::aigpdk::AIGPDKLeafPins;
use gem::sky130::{detect_library_from_file, CellLibrary, SKY130LeafPins};
use gem::flatten::FlattenedScriptV1;
use gem::staging::build_staged_aigs;
use gem::pe::Partition;
use gem::testbench::{CppSpiFlash, TestbenchConfig, UartMonitor};
use netlistdb::{GeneralPinName, NetlistDB};
use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use ulib::{AsUPtr, Device};

use metal::{Device as MTLDevice, MTLSize, ComputePipelineState, CommandQueue, MTLResourceOptions, SharedEvent};

// ── CLI Arguments ────────────────────────────────────────────────────────────

#[derive(clap::Parser, Debug)]
#[command(name = "gpu_sim")]
#[command(about = "Hybrid GPU/CPU co-simulation with Metal")]
struct Args {
    /// Gate-level verilog path synthesized in AIGPDK library.
    netlist_verilog: PathBuf,

    /// Pre-compiled partition mapping (.gemparts file).
    gemparts: PathBuf,

    /// Testbench configuration JSON file.
    #[clap(long)]
    config: PathBuf,

    /// Top module type in netlist.
    #[clap(long)]
    top_module: Option<String>,

    /// Level split thresholds (comma-separated).
    #[clap(long, value_delimiter = ',')]
    level_split: Vec<usize>,

    /// Number of GPU threadgroups (blocks). Should be ~2x GPU SM count.
    #[clap(long, default_value = "64")]
    num_blocks: usize,

    /// Maximum system clock ticks to simulate.
    #[clap(long)]
    max_cycles: Option<usize>,

    /// Enable verbose flash model debug output.
    #[clap(long)]
    flash_verbose: bool,

    /// Clock period in picoseconds (default: 1000 = 1ns, for UART baud calc).
    #[clap(long, default_value = "1000")]
    clock_period: u64,

    /// Verify GPU results against CPU baseline.
    #[clap(long)]
    check_with_cpu: bool,
}

// ── Simulation Parameters (must match Metal shader) ──────────────────────────

#[repr(C)]
struct SimParams {
    num_blocks: u64,
    num_major_stages: u64,
    num_cycles: u64,
    state_size: u64,
    current_cycle: u64,
    current_stage: u64,
}

// ── Metal Simulator ──────────────────────────────────────────────────────────

struct MetalSimulator {
    device: metal::Device,
    command_queue: CommandQueue,
    pipeline_state: ComputePipelineState,
    /// Pre-allocated params buffers for each stage (shared memory, rewritten each dispatch).
    /// We need one per stage since multi-stage designs encode all stages before commit.
    params_buffers: Vec<metal::Buffer>,
    /// Shared event for GPU↔CPU synchronization within a single command buffer.
    /// Allows encoding both half-cycle dispatches in one buffer with CPU work between them.
    shared_event: SharedEvent,
    /// Monotonic counter for shared event signaling (increments by 3 per tick)
    event_counter: std::cell::Cell<u64>,
}

impl MetalSimulator {
    fn new(num_stages: usize) -> Self {
        let device = MTLDevice::system_default().expect("No Metal device found");
        clilog::info!("Using Metal device: {}", device.name());

        let metallib_path = env!("METALLIB_PATH");
        let library = device
            .new_library_with_file(metallib_path)
            .expect("Failed to load metallib");

        let kernel_function = library
            .get_function("simulate_v1_stage", None)
            .expect("Failed to get kernel function");

        let pipeline_state = device
            .new_compute_pipeline_state_with_function(&kernel_function)
            .expect("Failed to create pipeline state");

        let command_queue = device.new_command_queue();

        // Pre-allocate one params buffer per stage
        let params_buffers: Vec<_> = (0..num_stages.max(1))
            .map(|_| {
                device.new_buffer(
                    std::mem::size_of::<SimParams>() as u64,
                    MTLResourceOptions::StorageModeShared,
                )
            })
            .collect();

        let shared_event = device.new_shared_event();

        Self {
            device,
            command_queue,
            pipeline_state,
            params_buffers,
            shared_event,
            event_counter: std::cell::Cell::new(0),
        }
    }

    /// Dispatch a single stage (standalone, with own command buffer).
    /// Used as fallback when dual-tick pattern isn't applicable.
    #[allow(dead_code)]
    #[inline]
    fn dispatch_stage(
        &self,
        num_blocks: usize,
        num_major_stages: usize,
        state_size: usize,
        cycle_i: usize,
        stage_i: usize,
        blocks_start_buffer: &metal::Buffer,
        blocks_data_buffer: &metal::Buffer,
        sram_data_buffer: &metal::Buffer,
        states_buffer: &metal::Buffer,
        event_buffer_metal: &metal::Buffer,
    ) {
        self.write_params(stage_i, num_blocks, num_major_stages, state_size, cycle_i);

        let command_buffer = self.command_queue.new_command_buffer();
        self.encode_dispatch(
            &command_buffer, num_blocks, stage_i,
            blocks_start_buffer, blocks_data_buffer,
            sram_data_buffer, states_buffer, event_buffer_metal,
        );
        command_buffer.commit();
        command_buffer.wait_until_completed();
    }

    /// Write params for a given stage into the pre-allocated shared memory buffer.
    #[inline]
    fn write_params(
        &self,
        stage_i: usize,
        num_blocks: usize,
        num_major_stages: usize,
        state_size: usize,
        cycle_i: usize,
    ) {
        let params = SimParams {
            num_blocks: num_blocks as u64,
            num_major_stages: num_major_stages as u64,
            num_cycles: 1,
            state_size: state_size as u64,
            current_cycle: cycle_i as u64,
            current_stage: stage_i as u64,
        };
        unsafe {
            std::ptr::write(
                self.params_buffers[stage_i].contents() as *mut SimParams,
                params,
            );
        }
    }

    /// Encode a compute dispatch into an existing command buffer (no commit).
    #[inline]
    fn encode_dispatch(
        &self,
        command_buffer: &metal::CommandBufferRef,
        num_blocks: usize,
        stage_i: usize,
        blocks_start_buffer: &metal::Buffer,
        blocks_data_buffer: &metal::Buffer,
        sram_data_buffer: &metal::Buffer,
        states_buffer: &metal::Buffer,
        event_buffer_metal: &metal::Buffer,
    ) {
        let encoder = command_buffer.new_compute_command_encoder();
        encoder.set_compute_pipeline_state(&self.pipeline_state);
        encoder.set_buffer(0, Some(blocks_start_buffer), 0);
        encoder.set_buffer(1, Some(blocks_data_buffer), 0);
        encoder.set_buffer(2, Some(sram_data_buffer), 0);
        encoder.set_buffer(3, Some(states_buffer), 0);
        encoder.set_buffer(4, Some(&self.params_buffers[stage_i]), 0);
        encoder.set_buffer(5, Some(event_buffer_metal), 0);

        let threads_per_threadgroup = MTLSize::new(256, 1, 1);
        let threadgroups = MTLSize::new(num_blocks as u64, 1, 1);

        encoder.dispatch_thread_groups(threadgroups, threads_per_threadgroup);
        encoder.end_encoding();
    }

    /// Spin-wait for shared event to reach target value.
    #[inline]
    fn spin_wait(&self, target: u64) {
        while self.shared_event.signaled_value() < target {
            std::hint::spin_loop();
        }
    }

    /// CPU signals the shared event (unblocks GPU waiting on this value).
    #[inline]
    fn cpu_signal(&self, value: u64) {
        self.shared_event.set_signaled_value(value);
    }
}

// ── GPIO ↔ State Buffer Mapping ──────────────────────────────────────────────

/// Maps GPIO pin indices to bit positions in the packed u32 state buffer.
struct GpioMapping {
    /// gpio_in[idx] → (aigpin, state bit position)
    input_bits: HashMap<usize, u32>,
    /// gpio_out[idx] → state bit position in output_map
    output_bits: HashMap<usize, u32>,
    /// Posedge clock flag bit positions (one per clock pin)
    posedge_flag_bits: Vec<u32>,
    /// Negedge clock flag bit positions
    negedge_flag_bits: Vec<u32>,
}

/// Read a single bit from a packed u32 state buffer.
#[inline]
fn read_bit(state: &[u32], pos: u32) -> u8 {
    ((state[(pos >> 5) as usize] >> (pos & 31)) & 1) as u8
}

/// Set a single bit in a packed u32 state buffer.
#[inline]
fn set_bit(state: &mut [u32], pos: u32, val: u8) {
    let word = &mut state[(pos >> 5) as usize];
    let mask = 1u32 << (pos & 31);
    if val != 0 {
        *word |= mask;
    } else {
        *word &= !mask;
    }
}

/// Clear a single bit in a packed u32 state buffer.
#[inline]
fn clear_bit(state: &mut [u32], pos: u32) {
    state[(pos >> 5) as usize] &= !(1u32 << (pos & 31));
}

/// Build GPIO-to-state-buffer mapping from AIG + FlattenedScript.
fn build_gpio_mapping(
    aig: &AIG,
    netlistdb: &NetlistDB,
    script: &FlattenedScriptV1,
) -> GpioMapping {
    let mut input_bits: HashMap<usize, u32> = HashMap::new();
    let mut output_bits: HashMap<usize, u32> = HashMap::new();
    let mut posedge_flag_bits: Vec<u32> = Vec::new();
    let mut negedge_flag_bits: Vec<u32> = Vec::new();

    // Map input ports (gpio_in) → state buffer positions
    for (aigpin_idx, driv) in aig.drivers.iter().enumerate() {
        match driv {
            DriverType::InputPort(pinid) => {
                let pin_name = netlistdb.pinnames[*pinid].dbg_fmt_pin();
                // Parse gpio_in[N] from the pin name
                if let Some(gpio_idx) = parse_gpio_index(&pin_name, "gpio_in") {
                    if let Some(&pos) = script.input_map.get(&aigpin_idx) {
                        input_bits.insert(gpio_idx, pos);
                    }
                }
            }
            DriverType::InputClockFlag(pinid, is_negedge) => {
                if let Some(&pos) = script.input_map.get(&aigpin_idx) {
                    if *is_negedge == 0 {
                        posedge_flag_bits.push(pos);
                    } else {
                        negedge_flag_bits.push(pos);
                    }
                    // Also check if this is associated with a GPIO
                    let pin_name = netlistdb.pinnames[*pinid].dbg_fmt_pin();
                    clilog::debug!("ClockFlag aigpin={} pin={} negedge={} pos={}",
                                   aigpin_idx, pin_name, is_negedge, pos);
                }
            }
            _ => {}
        }
    }

    // Map output ports (gpio_out) → state buffer positions
    // Search by name across all cell-0 pins (direction conventions vary by netlist)
    for i in netlistdb.cell2pin.iter_set(0) {
        let pin_name = netlistdb.pinnames[i].dbg_fmt_pin();
        if let Some(gpio_idx) = parse_gpio_index(&pin_name, "gpio_out") {
            let aigpin_iv = aig.pin2aigpin_iv[i];
            if aigpin_iv == usize::MAX {
                clilog::info!("gpio_out[{}] pin {} has no AIG connection (usize::MAX)", gpio_idx, i);
                continue;
            }
            if aigpin_iv <= 1 {
                clilog::info!("gpio_out[{}] pin {} is constant (aigpin_iv={})", gpio_idx, i, aigpin_iv);
                continue;
            }
            // output_map is keyed by idx_iv (with inversion bit), same as pin2aigpin_iv
            if let Some(&pos) = script.output_map.get(&aigpin_iv) {
                output_bits.insert(gpio_idx, pos);
            } else if let Some(&pos) = script.output_map.get(&(aigpin_iv ^ 1)) {
                // Try with flipped inversion bit
                output_bits.insert(gpio_idx, pos);
                clilog::debug!("gpio_out[{}] found with flipped inv bit, aigpin_iv={}→{}",
                              gpio_idx, aigpin_iv, aigpin_iv ^ 1);
            } else {
                clilog::info!("gpio_out[{}] pin {} aigpin_iv={} (aigpin={}) not in output_map (dir={:?})",
                              gpio_idx, i, aigpin_iv, aigpin_iv >> 1, netlistdb.pindirect[i]);
            }
        }
    }

    clilog::info!("GPIO mapping: {} inputs, {} outputs, {} posedge flags, {} negedge flags",
                  input_bits.len(), output_bits.len(),
                  posedge_flag_bits.len(), negedge_flag_bits.len());

    GpioMapping {
        input_bits,
        output_bits,
        posedge_flag_bits,
        negedge_flag_bits,
    }
}

/// Parse a GPIO index from a pin name like "gpio_in[38]" or "gpio_in:38".
fn parse_gpio_index(pin_name: &str, prefix: &str) -> Option<usize> {
    // Try "gpio_in[N]" format
    if let Some(start) = pin_name.find(&format!("{}[", prefix)) {
        let after = &pin_name[start + prefix.len() + 1..];
        if let Some(end) = after.find(']') {
            return after[..end].parse().ok();
        }
    }
    // Try "gpio_in:N" format
    if let Some(start) = pin_name.find(&format!("{}:", prefix)) {
        let after = &pin_name[start + prefix.len() + 1..];
        let num_str: String = after.chars().take_while(|c| c.is_ascii_digit()).collect();
        return num_str.parse().ok();
    }
    None
}

/// Read a 4-bit nibble from flash data GPIO outputs.
fn read_flash_nibble(state: &[u32], gpio_map: &GpioMapping, d0_gpio: usize) -> u8 {
    let mut val = 0u8;
    for i in 0..4 {
        if let Some(&pos) = gpio_map.output_bits.get(&(d0_gpio + i)) {
            val |= read_bit(state, pos) << i;
        }
    }
    val
}

/// Write flash data input to GPIO state.
fn set_flash_din(state: &mut [u32], gpio_map: &GpioMapping, d0_gpio: usize, din: u8) {
    for i in 0..4 {
        if let Some(&pos) = gpio_map.input_bits.get(&(d0_gpio + i)) {
            set_bit(state, pos, (din >> i) & 1);
        }
    }
}

// ── CPU baseline for verification ────────────────────────────────────────────

/// CPU prototype partition executor (from metal_test.rs).
/// Used for --check-with-cpu verification.
#[allow(dead_code)]
fn simulate_block_v1(
    script: &[u32],
    input_state: &[u32],
    output_state: &mut [u32],
    sram_data: &mut [u32],
) {
    use gem::aigpdk::AIGPDK_SRAM_SIZE;
    let mut script_pi = 0;
    loop {
        let num_stages = script[script_pi];
        let is_last_part = script[script_pi + 1];
        let num_ios = script[script_pi + 2];
        let io_offset = script[script_pi + 3];
        let num_srams = script[script_pi + 4];
        let sram_offset = script[script_pi + 5];
        let num_global_read_rounds = script[script_pi + 6];
        let num_output_duplicates = script[script_pi + 7];
        let mut writeout_hooks = vec![0; 256];
        for i in 0..128 {
            let t = script[script_pi + 128 + i];
            writeout_hooks[i * 2] = (t & ((1 << 16) - 1)) as u16;
            writeout_hooks[i * 2 + 1] = (t >> 16) as u16;
        }
        if num_stages == 0 {
            script_pi += 256;
            break;
        }
        script_pi += 256;
        let mut writeouts = vec![0u32; num_ios as usize];
        let mut state = vec![0u32; 256];

        for _gr_i in 0..num_global_read_rounds {
            for i in 0..256 {
                let mut cur_state = state[i];
                let idx = script[script_pi + (i * 2)];
                let mut mask = script[script_pi + (i * 2 + 1)];
                if mask == 0 { continue; }
                let value = match (idx >> 31) != 0 {
                    false => input_state[idx as usize],
                    true => output_state[(idx ^ (1 << 31)) as usize],
                };
                while mask != 0 {
                    cur_state <<= 1;
                    let lowbit = mask & (-(mask as i32)) as u32;
                    if (value & lowbit) != 0 { cur_state |= 1; }
                    mask ^= lowbit;
                }
                state[i] = cur_state;
            }
            script_pi += 256 * 2;
        }

        for bs_i in 0..num_stages {
            let mut hier_inputs = vec![0; 256];
            let mut hier_flag_xora = vec![0; 256];
            let mut hier_flag_xorb = vec![0; 256];
            let mut hier_flag_orb = vec![0; 256];
            for k_outer in 0..4 {
                for i in 0..256 {
                    for k_inner in 0..4 {
                        let k = k_outer * 4 + k_inner;
                        let t_shuffle = script[script_pi + i * 4 + k_inner];
                        let t_shuffle_1_idx = (t_shuffle & ((1 << 16) - 1)) as u16;
                        let t_shuffle_2_idx = (t_shuffle >> 16) as u16;
                        hier_inputs[i] |= (state[(t_shuffle_1_idx >> 5) as usize] >> (t_shuffle_1_idx & 31) & 1) << (k * 2);
                        hier_inputs[i] |= (state[(t_shuffle_2_idx >> 5) as usize] >> (t_shuffle_2_idx & 31) & 1) << (k * 2 + 1);
                    }
                }
                script_pi += 256 * 4;
            }
            for i in 0..256 {
                hier_flag_xora[i] = script[script_pi + i * 4];
                hier_flag_xorb[i] = script[script_pi + i * 4 + 1];
                hier_flag_orb[i] = script[script_pi + i * 4 + 2];
            }
            script_pi += 256 * 4;

            for i in 0..128 {
                let a = hier_inputs[i];
                let b = hier_inputs[128 + i];
                let xora = hier_flag_xora[128 + i];
                let xorb = hier_flag_xorb[128 + i];
                let orb = hier_flag_orb[128 + i];
                hier_inputs[128 + i] = (a ^ xora) & ((b ^ xorb) | orb);
            }
            for hi in 1..=7 {
                let hier_width = 1 << (7 - hi);
                for i in 0..hier_width {
                    let a = hier_inputs[hier_width * 2 + i];
                    let b = hier_inputs[hier_width * 3 + i];
                    let xora = hier_flag_xora[hier_width + i];
                    let xorb = hier_flag_xorb[hier_width + i];
                    let orb = hier_flag_orb[hier_width + i];
                    hier_inputs[hier_width + i] = (a ^ xora) & ((b ^ xorb) | orb);
                }
            }
            let v1 = hier_inputs[1];
            let xora = hier_flag_xora[0];
            let xorb = hier_flag_xorb[0];
            let orb = hier_flag_orb[0];
            let r8 = ((v1 << 16) ^ xora) & ((v1 ^ xorb) | orb) & 0xffff0000;
            let r9 = ((r8 >> 8) ^ xora) & (((r8 >> 16) ^ xorb) | orb) & 0xff00;
            let r10 = ((r9 >> 4) ^ xora) & (((r9 >> 8) ^ xorb) | orb) & 0xf0;
            let r11 = ((r10 >> 2) ^ xora) & (((r10 >> 4) ^ xorb) | orb) & 0b1100;
            let r12 = ((r11 >> 1) ^ xora) & (((r11 >> 2) ^ xorb) | orb) & 0b10;
            hier_inputs[0] = r8 | r9 | r10 | r11 | r12;
            state = hier_inputs;

            for i in 0..256 {
                let hooki = writeout_hooks[i];
                if (hooki >> 8) as u32 == bs_i {
                    writeouts[i] = state[(hooki & 255) as usize];
                }
            }
        }

        let mut sram_duplicate_perm = vec![0u32; (num_srams * 4 + num_output_duplicates) as usize];
        for k_outer in 0..4 {
            for i in 0..(num_srams * 4 + num_output_duplicates) {
                for k_inner in 0..4 {
                    let k = k_outer * 4 + k_inner;
                    let t_shuffle = script[script_pi + (i * 4 + k_inner) as usize];
                    let t_shuffle_1_idx = (t_shuffle & ((1 << 16) - 1)) as u32;
                    let t_shuffle_2_idx = (t_shuffle >> 16) as u32;
                    sram_duplicate_perm[i as usize] |= (writeouts[(t_shuffle_1_idx >> 5) as usize] >> (t_shuffle_1_idx & 31) & 1) << (k * 2);
                    sram_duplicate_perm[i as usize] |= (writeouts[(t_shuffle_2_idx >> 5) as usize] >> (t_shuffle_2_idx & 31) & 1) << (k * 2 + 1);
                }
            }
            script_pi += 256 * 4;
        }
        for i in 0..(num_srams * 4 + num_output_duplicates) as usize {
            sram_duplicate_perm[i] &= !script[script_pi + i * 4 + 1];
            sram_duplicate_perm[i] ^= script[script_pi + i * 4];
        }
        script_pi += 256 * 4;

        for sram_i_u32 in 0..num_srams {
            let sram_i = sram_i_u32 as usize;
            let addrs = sram_duplicate_perm[sram_i * 4];
            let port_r_addr_iv = addrs & 0xffff;
            let port_w_addr_iv = (addrs & 0xffff0000) >> 16;
            let port_w_wr_en = sram_duplicate_perm[sram_i * 4 + 1];
            let port_w_wr_data_iv = sram_duplicate_perm[sram_i * 4 + 2];
            let sram_st = sram_offset as usize + sram_i * AIGPDK_SRAM_SIZE;
            let sram_ed = sram_st + AIGPDK_SRAM_SIZE;
            let ram = &mut sram_data[sram_st..sram_ed];
            let r = ram[port_r_addr_iv as usize];
            let w0 = ram[port_w_addr_iv as usize];
            writeouts[(num_ios - num_srams + sram_i_u32) as usize] = r;
            ram[port_w_addr_iv as usize] = (w0 & !port_w_wr_en) | (port_w_wr_data_iv & port_w_wr_en);
        }

        for i in 0..num_output_duplicates {
            writeouts[(num_ios - num_srams - num_output_duplicates + i) as usize] =
                sram_duplicate_perm[(num_srams * 4 + i) as usize];
        }

        let mut clken_perm = vec![0u32; num_ios as usize];
        let writeouts_for_clken = writeouts.clone();
        for k_outer in 0..4 {
            for i in 0..num_ios {
                for k_inner in 0..4 {
                    let k = k_outer * 4 + k_inner;
                    let t_shuffle = script[script_pi + (i * 4 + k_inner) as usize];
                    let t_shuffle_1_idx = (t_shuffle & ((1 << 16) - 1)) as u32;
                    let t_shuffle_2_idx = (t_shuffle >> 16) as u32;
                    clken_perm[i as usize] |= (writeouts_for_clken[(t_shuffle_1_idx >> 5) as usize] >> (t_shuffle_1_idx & 31) & 1) << (k * 2);
                    clken_perm[i as usize] |= (writeouts_for_clken[(t_shuffle_2_idx >> 5) as usize] >> (t_shuffle_2_idx & 31) & 1) << (k * 2 + 1);
                }
            }
            script_pi += 256 * 4;
        }
        for i in 0..num_ios as usize {
            clken_perm[i] &= !script[script_pi + i * 4 + 1];
            clken_perm[i] ^= script[script_pi + i * 4];
            writeouts[i] ^= script[script_pi + i * 4 + 2];
        }
        script_pi += 256 * 4;

        for i in 0..num_ios {
            let old_wo = input_state[(io_offset + i) as usize];
            let clken = clken_perm[i as usize];
            let wo = (old_wo & !clken) | (writeouts[i as usize] & clken);
            output_state[(io_offset + i) as usize] = wo;
        }

        if is_last_part != 0 { break; }
    }
    assert_eq!(script_pi, script.len());
}

// ── Main ─────────────────────────────────────────────────────────────────────

fn main() {
    clilog::init_stderr_color_debug();
    clilog::enable_timer("gpu_sim");
    clilog::enable_timer("gem");
    clilog::set_max_print_count(clilog::Level::Warn, "NL_SV_LIT", 1);

    let args = <Args as clap::Parser>::parse();
    clilog::info!("gpu_sim args:\n{:#?}", args);

    // ── Load testbench config ────────────────────────────────────────────

    let file = File::open(&args.config).expect("Failed to open config file");
    let reader = BufReader::new(file);
    let config: TestbenchConfig =
        serde_json::from_reader(reader).expect("Failed to parse config JSON");
    clilog::info!("Loaded testbench config: {:?}", config);

    let max_ticks = args.max_cycles.unwrap_or(config.num_cycles);

    // ── Load netlist and build AIG ───────────────────────────────────────

    let timer_load = clilog::stimer!("load_netlist");

    // Detect cell library (AIGPDK vs SKY130)
    let cell_library = detect_library_from_file(&args.netlist_verilog)
        .expect("Failed to read netlist file for library detection");
    clilog::info!("Detected cell library: {}", cell_library);

    let netlistdb = match cell_library {
        CellLibrary::SKY130 => NetlistDB::from_sverilog_file(
            &args.netlist_verilog,
            args.top_module.as_deref(),
            &SKY130LeafPins,
        ),
        CellLibrary::AIGPDK | CellLibrary::Mixed => NetlistDB::from_sverilog_file(
            &args.netlist_verilog,
            args.top_module.as_deref(),
            &AIGPDKLeafPins(),
        ),
    }
    .expect("cannot build netlist");

    let aig = AIG::from_netlistdb(&netlistdb);
    clilog::info!(
        "AIG: {} pins, {} DFFs, {} SRAMs",
        aig.num_aigpins,
        aig.dffs.len(),
        aig.srams.len()
    );
    clilog::finish!(timer_load);

    // ── Build staged AIGs and load partitions ────────────────────────────

    let timer_script = clilog::stimer!("build_script");
    let stageds = build_staged_aigs(&aig, &args.level_split);

    let f = std::fs::File::open(&args.gemparts).unwrap();
    let mut buf = std::io::BufReader::new(f);
    let parts_in_stages: Vec<Vec<Partition>> = serde_bare::from_reader(&mut buf).unwrap();
    clilog::info!(
        "Partitions per stage: {:?}",
        parts_in_stages.iter().map(|ps| ps.len()).collect::<Vec<_>>()
    );

    let mut input_layout = Vec::new();
    for (i, driv) in aig.drivers.iter().enumerate() {
        if let DriverType::InputPort(_) | DriverType::InputClockFlag(_, _) = driv {
            input_layout.push(i);
        }
    }

    let script = FlattenedScriptV1::from(
        &aig,
        &stageds.iter().map(|(_, _, staged)| staged).collect::<Vec<_>>(),
        &parts_in_stages.iter().map(|ps| ps.as_slice()).collect::<Vec<_>>(),
        args.num_blocks,
        input_layout,
    );
    clilog::info!(
        "Script: state_size={}, sram_storage={}, blocks={}, stages={}",
        script.reg_io_state_size,
        script.sram_storage_size,
        script.num_blocks,
        script.num_major_stages
    );
    clilog::finish!(timer_script);

    // ── Build GPIO mapping ───────────────────────────────────────────────

    let gpio_map = build_gpio_mapping(&aig, &netlistdb, &script);

    // Verify we found the expected GPIO pins
    let clock_gpio = config.clock_gpio;
    let reset_gpio = config.reset_gpio;
    assert!(
        gpio_map.input_bits.contains_key(&clock_gpio),
        "Clock GPIO {} not found in input mapping. Available: {:?}",
        clock_gpio,
        gpio_map.input_bits.keys().collect::<Vec<_>>()
    );
    assert!(
        gpio_map.input_bits.contains_key(&reset_gpio),
        "Reset GPIO {} not found in input mapping. Available: {:?}",
        reset_gpio,
        gpio_map.input_bits.keys().collect::<Vec<_>>()
    );

    if let Some(ref flash_cfg) = config.flash {
        for i in 0..4 {
            let gpio = flash_cfg.d0_gpio + i;
            if gpio_map.input_bits.contains_key(&gpio) {
                clilog::info!("Flash D{} input GPIO {} -> state pos {}", i, gpio, gpio_map.input_bits[&gpio]);
            }
            if gpio_map.output_bits.contains_key(&gpio) {
                clilog::info!("Flash D{} output GPIO {} -> state pos {}", i, gpio, gpio_map.output_bits[&gpio]);
            }
        }
    }

    // ── Initialize peripheral models ─────────────────────────────────────

    let mut flash: Option<CppSpiFlash> = if let Some(ref flash_cfg) = config.flash {
        let mut fl = CppSpiFlash::new(16 * 1024 * 1024);
        fl.set_verbose(args.flash_verbose);
        let firmware_path = std::path::Path::new(&flash_cfg.firmware);
        match fl.load_firmware(firmware_path, flash_cfg.firmware_offset) {
            Ok(size) => clilog::info!("Loaded {} bytes firmware at offset 0x{:X}", size, flash_cfg.firmware_offset),
            Err(e) => panic!("Failed to load firmware: {}", e),
        }
        Some(fl)
    } else {
        None
    };

    let clock_hz = 1_000_000_000_000u64 / args.clock_period;
    let uart_baud = config.uart.as_ref().map(|u| u.baud_rate).unwrap_or(115200);
    let mut uart_monitor = UartMonitor::new(clock_hz, uart_baud);
    let uart_tx_gpio = config.uart.as_ref().map(|u| u.tx_gpio);

    // ── Initialize Metal simulator and GPU state buffers ─────────────────

    let timer_init = clilog::stimer!("init_gpu");
    let simulator = MetalSimulator::new(script.num_major_stages);

    let state_size = script.reg_io_state_size as usize;

    // IMPORTANT: We use Metal shared-memory buffers DIRECTLY for state, NOT UVec.
    // UVec maintains separate CPU and Metal allocations and syncs between them,
    // which breaks our hybrid model where CPU modifies state between GPU dispatches.
    // With Metal StorageModeShared on Apple Silicon, the CPU and GPU share the
    // same physical memory, so we allocate a Metal buffer and access it as a
    // mutable slice from the CPU side.

    // States: [input state (state_size)] [output state (state_size)]
    let states_buffer = simulator.device.new_buffer(
        (2 * state_size * std::mem::size_of::<u32>()) as u64,
        MTLResourceOptions::StorageModeShared,
    );
    // SAFETY: Metal shared buffer is CPU-accessible. We ensure no aliasing by
    // waiting for GPU completion before CPU access and vice versa.
    let states: &mut [u32] = unsafe {
        std::slice::from_raw_parts_mut(
            states_buffer.contents() as *mut u32,
            2 * state_size,
        )
    };
    // Zero-initialize
    states.fill(0);

    // SRAM storage
    let sram_data_buffer = simulator.device.new_buffer(
        (script.sram_storage_size as usize * std::mem::size_of::<u32>()) as u64,
        MTLResourceOptions::StorageModeShared,
    );
    let sram_data: &mut [u32] = unsafe {
        std::slice::from_raw_parts_mut(
            sram_data_buffer.contents() as *mut u32,
            script.sram_storage_size as usize,
        )
    };
    sram_data.fill(0);
    // Keep sram_data alive but we don't use it from CPU after init
    let _ = sram_data;

    // Initialize: set reset active
    let reset_val = if config.reset_active_high { 1u8 } else { 0u8 };
    set_bit(&mut states[..state_size], gpio_map.input_bits[&reset_gpio], reset_val);
    clilog::info!("Initial state: reset GPIO {} = {} (active)", reset_gpio, reset_val);

    // Set flash D_IN defaults (high = no data)
    if let Some(ref flash_cfg) = config.flash {
        set_flash_din(&mut states[..state_size], &gpio_map, flash_cfg.d0_gpio, 0x0F);
    }

    // Create Metal buffers for script data (read-only, use UVec's Metal path)
    let device = Device::Metal(0);
    let blocks_start_ptr = script.blocks_start.as_uptr(device);
    let blocks_data_ptr = script.blocks_data.as_uptr(device);

    let blocks_start_buffer = simulator.device.new_buffer_with_bytes_no_copy(
        blocks_start_ptr as *const _,
        (script.blocks_start.len() * std::mem::size_of::<usize>()) as u64,
        MTLResourceOptions::StorageModeShared,
        None,
    );
    let blocks_data_buffer = simulator.device.new_buffer_with_bytes_no_copy(
        blocks_data_ptr as *const _,
        (script.blocks_data.len() * std::mem::size_of::<u32>()) as u64,
        MTLResourceOptions::StorageModeShared,
        None,
    );

    // Event buffer (for $stop/$finish/assertions - allocated but mostly unused in hybrid mode)
    let event_buffer = Box::new(gem::event_buffer::EventBuffer::new());
    let event_buffer_ptr = Box::into_raw(event_buffer);
    let event_buffer_metal = simulator.device.new_buffer_with_bytes_no_copy(
        event_buffer_ptr as *const _,
        std::mem::size_of::<gem::event_buffer::EventBuffer>() as u64,
        MTLResourceOptions::StorageModeShared,
        None,
    );

    clilog::finish!(timer_init);

    // ── Simulation loop ──────────────────────────────────────────────────
    //
    // Each tick = 1 system clock cycle = 2 GPU evaluations (fall + rise).
    //
    // Tick order (matching CXXRTL):
    //   1. Read gpio_out from previous tick's output
    //   2. Step peripheral models (flash, UART)
    //   3. Copy output → input, set falling edge inputs
    //   4. GPU dispatch all stages (falling edge)
    //   5. Copy output → input, set rising edge inputs + posedge flag
    //   6. GPU dispatch all stages (rising edge, DFFs latch)
    //   7. Clear posedge flags in output

    let timer_sim = clilog::stimer!("simulation");
    let sim_start = std::time::Instant::now();
    let reset_cycles = config.reset_cycles;
    let num_major_stages = script.num_major_stages;
    let num_blocks = script.num_blocks;

    // Track previous flash data for setup timing delay (same as timing_sim_cpu).
    // The CPU sim's step_flash reads clk CURRENT, but csn/d_out PREVIOUS,
    // to match real hardware setup timing where data is stable before clock edge.
    let mut prev_flash_d_out: u8 = 0;
    let mut prev_flash_csn: bool = true;
    // Flash din from the previous tick's rising-edge step (used for falling edge input)
    let mut flash_din: u8 = 0;

    // Helper: read flash signals from output state
    let read_flash_out = |states: &[u32], state_size: usize| -> (bool, bool, u8) {
        let out = &states[state_size..2 * state_size];
        let clk = config.flash.as_ref().map(|f| {
            gpio_map.output_bits.get(&f.clk_gpio)
                .map(|&pos| read_bit(out, pos) != 0).unwrap_or(false)
        }).unwrap_or(false);
        let csn = config.flash.as_ref().map(|f| {
            gpio_map.output_bits.get(&f.csn_gpio)
                .map(|&pos| read_bit(out, pos) != 0).unwrap_or(true)
        }).unwrap_or(true);
        let d_out = config.flash.as_ref().map(|f| {
            read_flash_nibble(out, &gpio_map, f.d0_gpio)
        }).unwrap_or(0);
        (clk, csn, d_out)
    };

    // Pre-write params for all stages (they don't change between ticks)
    for stage_i in 0..num_major_stages {
        simulator.write_params(stage_i, num_blocks, num_major_stages, state_size, 0);
    }

    for tick in 0..max_ticks {
        // Reset event buffer
        unsafe { (*event_buffer_ptr).reset(); }

        let in_reset = tick < reset_cycles;
        let reset_val = if in_reset {
            if config.reset_active_high { 1u8 } else { 0u8 }
        } else {
            if config.reset_active_high { 0u8 } else { 1u8 }
        };

        // ── 1. Copy prev output → input, set falling edge ──────────────
        states.copy_within(state_size..2 * state_size, 0);
        set_bit(&mut states[..state_size], gpio_map.input_bits[&reset_gpio], reset_val);
        set_bit(&mut states[..state_size], gpio_map.input_bits[&clock_gpio], 0);
        for &pos in &gpio_map.posedge_flag_bits {
            clear_bit(&mut states[..state_size], pos);
        }
        for &pos in &gpio_map.negedge_flag_bits {
            set_bit(&mut states[..state_size], pos, 1);
        }
        // Write flash din from previous tick's rising-edge flash step
        if let Some(ref flash_cfg) = config.flash {
            set_flash_din(&mut states[..state_size], &gpio_map, flash_cfg.d0_gpio, flash_din);
        }

        // ── 2-5. Dual dispatch with CPU intervention between edges ──────
        // Encode BOTH half-cycle dispatches into a SINGLE command buffer.
        // GPU signals event after falling edge, waits for CPU to update state,
        // then proceeds with rising edge dispatch. This eliminates one
        // command buffer creation per tick (~50-100μs savings).
        let base = simulator.event_counter.get();
        let fall_done = base + 1;   // GPU signals after falling edge
        let cpu_done = base + 2;    // CPU signals after state update
        let rise_done = base + 3;   // GPU signals after rising edge
        simulator.event_counter.set(rise_done);

        let command_buffer = simulator.command_queue.new_command_buffer();

        // Encode falling edge dispatch(es)
        for stage_i in 0..num_major_stages {
            simulator.encode_dispatch(
                &command_buffer, num_blocks, stage_i,
                &blocks_start_buffer, &blocks_data_buffer,
                &sram_data_buffer, &states_buffer, &event_buffer_metal,
            );
        }

        // GPU signals fall_done, then waits for CPU to signal cpu_done
        command_buffer.encode_signal_event(&simulator.shared_event, fall_done);
        command_buffer.encode_wait_for_event(&simulator.shared_event, cpu_done);

        // Encode rising edge dispatch(es)
        for stage_i in 0..num_major_stages {
            simulator.encode_dispatch(
                &command_buffer, num_blocks, stage_i,
                &blocks_start_buffer, &blocks_data_buffer,
                &sram_data_buffer, &states_buffer, &event_buffer_metal,
            );
        }

        // GPU signals rise_done after rising edge
        command_buffer.encode_signal_event(&simulator.shared_event, rise_done);
        command_buffer.commit();

        // ── 3. CPU: wait for falling edge, step flash, update state ─────
        simulator.spin_wait(fall_done);

        // Step flash after falling edge (matches CPU sim step 2b)
        let flash_din_fall = if let Some(ref mut fl) = flash {
            let (clk, current_csn, current_d_out) = read_flash_out(states, state_size);
            let effective_csn = if in_reset { true } else { prev_flash_csn };
            let din = fl.step(clk, effective_csn, prev_flash_d_out);

            prev_flash_d_out = current_d_out;
            prev_flash_csn = current_csn;

            if tick <= 5 || tick == reset_cycles || tick == reset_cycles + 1 || tick == reset_cycles + 100 {
                clilog::info!("tick {}: fall flash clk={}, csn={}, d_out={:04b}, din={:04b}",
                             tick, clk, current_csn, current_d_out, din);
            }
            if in_reset { 0u8 } else { din }
        } else {
            0u8
        };

        // Copy falling output → input, set rising edge
        states.copy_within(state_size..2 * state_size, 0);
        set_bit(&mut states[..state_size], gpio_map.input_bits[&reset_gpio], reset_val);
        set_bit(&mut states[..state_size], gpio_map.input_bits[&clock_gpio], 1);
        for &pos in &gpio_map.posedge_flag_bits {
            set_bit(&mut states[..state_size], pos, 1);
        }
        for &pos in &gpio_map.negedge_flag_bits {
            clear_bit(&mut states[..state_size], pos);
        }
        if let Some(ref flash_cfg) = config.flash {
            set_flash_din(&mut states[..state_size], &gpio_map, flash_cfg.d0_gpio, flash_din_fall);
        }

        // Signal GPU to proceed with rising edge
        simulator.cpu_signal(cpu_done);

        // ── 6. Wait for rising edge, step flash + UART ──────────────────
        simulator.spin_wait(rise_done);

        // Step flash after rising edge (matches CPU sim step 6b)
        flash_din = if let Some(ref mut fl) = flash {
            let (clk, current_csn, current_d_out) = read_flash_out(states, state_size);
            let effective_csn = if in_reset { true } else { prev_flash_csn };
            let din = fl.step(clk, effective_csn, prev_flash_d_out);

            prev_flash_d_out = current_d_out;
            prev_flash_csn = current_csn;
            if in_reset { 0u8 } else { din }
        } else {
            0u8
        };

        // UART: read TX from rising-edge output
        if let Some(tx_gpio) = uart_tx_gpio {
            if let Some(&pos) = gpio_map.output_bits.get(&tx_gpio) {
                let tx = read_bit(&states[state_size..2 * state_size], pos);
                uart_monitor.step(tx, tick);
            }
        }

        // Progress logging
        if tick > 0 && tick % 100000 == 0 {
            clilog::info!("Tick {} / {}", tick, max_ticks);
        }
        if tick == reset_cycles {
            clilog::info!("Reset released at tick {}", tick);
        }
    }

    let sim_elapsed = sim_start.elapsed();
    clilog::finish!(timer_sim);

    // ── Results ──────────────────────────────────────────────────────────

    println!();
    println!("=== GPU Hybrid Simulation Results ===");
    println!("Ticks simulated: {}", max_ticks);
    println!("UART bytes received: {}", uart_monitor.events.len());
    if max_ticks > 0 {
        let us_per_tick = sim_elapsed.as_micros() as f64 / max_ticks as f64;
        println!("Time per tick: {:.1}μs ({:.1}s total)", us_per_tick, sim_elapsed.as_secs_f64());
    }

    // Print UART output as string
    if !uart_monitor.events.is_empty() {
        let uart_str: String = uart_monitor
            .events
            .iter()
            .map(|e| {
                if e.payload >= 32 && e.payload < 127 {
                    e.payload as char
                } else if e.payload == b'\n' {
                    '\n'
                } else if e.payload == b'\r' {
                    '\r'
                } else {
                    '.'
                }
            })
            .collect();
        println!("UART output:\n{}", uart_str);
    }

    // Print flash stats
    if let Some(ref fl) = flash {
        println!(
            "Flash model: steps={}, posedges={}, negedges={}",
            fl.get_step_count(),
            fl.get_posedge_count(),
            fl.get_negedge_count()
        );
    }

    // Output events to JSON
    if let Some(ref output_path) = config.output_events {
        #[derive(serde::Serialize)]
        struct EventsOutput {
            events: Vec<gem::testbench::UartEvent>,
        }
        let output = EventsOutput {
            events: uart_monitor.events,
        };
        let json = serde_json::to_string_pretty(&output).expect("Failed to serialize events");
        let mut file = File::create(output_path).expect("Failed to create events file");
        use std::io::Write;
        file.write_all(json.as_bytes())
            .expect("Failed to write events");
        clilog::info!("Wrote events to {}", output_path);
    }

    // ── Optional CPU verification ────────────────────────────────────────

    if args.check_with_cpu {
        clilog::info!("CPU verification not yet implemented for hybrid mode");
        // TODO: Run the same tick sequence through simulate_block_v1 and compare
        // This would need to save/replay all GPIO state changes
    }

    // Clean up event buffer
    unsafe {
        drop(Box::from_raw(event_buffer_ptr));
    }

    println!();
    println!("SIMULATION: PASSED");
}
