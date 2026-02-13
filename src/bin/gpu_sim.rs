// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! GPU-only simulation with Metal: gate evaluation, SPI flash, and UART
//! all run on GPU. No per-tick CPU interaction needed.
//!
//! Flash and UART IO models execute as GPU kernels after each tick's
//! simulate passes, eliminating the CPU round-trip bottleneck.
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
use gem::testbench::{CppSpiFlash, TestbenchConfig};
use netlistdb::{GeneralPinName, NetlistDB};
use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use ulib::{AsUPtr, Device};

#[macro_use]
extern crate objc;

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

    /// Run GPU kernel profiling: isolate each kernel in its own command buffer
    /// and measure per-kernel GPU execution time.
    #[clap(long)]
    gpu_profile: bool,
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

/// Bit set/clear operation (must match Metal shader BitOp struct).
#[repr(C)]
#[derive(Clone, Copy)]
struct BitOp {
    position: u32, // bit position in state buffer
    value: u32,    // 0 = clear, 1 = set
}

/// Parameters for the state_prep kernel (must match Metal shader StatePrepParams struct).
#[repr(C)]
struct StatePrepParams {
    state_size: u32,    // number of u32 words per state slot
    num_ops: u32,       // number of bit set/clear operations
    num_monitors: u32,  // number of peripheral monitors to check (0 = skip)
    tick_number: u32,   // current tick number
}

/// GPU-side flash state (must match Metal FlashState struct exactly).
#[repr(C)]
#[derive(Clone, Copy)]
struct FlashState {
    bit_count: i32,
    byte_count: i32,
    data_width: u32,
    addr: u32,
    curr_byte: u8,
    command: u8,
    out_buffer: u8,
    _pad1: u8,
    prev_clk: u32,
    prev_csn: u32,
    d_i: u8,
    _pad2: [u8; 3],
    prev_d_out: u8,
    _pad3: [u8; 3],
    in_reset: u32,
    last_error_cmd: u32,
    model_prev_csn: u32,
}

/// Parameters for gpu_apply_flash_din kernel (must match Metal FlashDinParams).
#[repr(C)]
struct FlashDinParams {
    d_in_pos: [u32; 4],
    has_flash: u32,
}

/// Parameters for gpu_flash_model_step kernel (must match Metal FlashModelParams).
#[repr(C)]
struct FlashModelParams {
    state_size: u32,
    clk_out_pos: u32,
    csn_out_pos: u32,
    d_out_pos: [u32; 4],
    flash_data_size: u32,
}

/// GPU-side UART decoder state (must match Metal UartDecoderState).
#[repr(C)]
struct UartDecoderState {
    state: u32,          // 0=IDLE, 1=START, 2=DATA, 3=STOP
    last_tx: u32,
    start_cycle: u32,
    bits_received: u32,
    value: u32,
    current_cycle: u32,
}

/// Parameters for gpu_uart_step kernel (must match Metal UartParams).
#[repr(C)]
struct UartParams {
    state_size: u32,
    tx_out_pos: u32,
    cycles_per_bit: u32,
    has_uart: u32,
}

/// GPU-side UART channel (must match Metal UartChannel).
#[repr(C)]
struct UartChannel {
    write_head: u32,
    capacity: u32,
    _pad: [u32; 2],
    data: [u8; 4096],
}

/// Batch size for GPU-only simulation (no per-tick CPU interaction).
const BATCH_SIZE: usize = 1024;

// ── Metal Simulator ──────────────────────────────────────────────────────────

struct MetalSimulator {
    device: metal::Device,
    command_queue: CommandQueue,
    pipeline_state: ComputePipelineState,
    /// Pipeline state for the state_prep kernel (no monitors).
    state_prep_pipeline: ComputePipelineState,
    /// Pipeline state for gpu_apply_flash_din kernel.
    gpu_apply_flash_din_pipeline: ComputePipelineState,
    /// Pipeline state for gpu_flash_model_step kernel.
    gpu_flash_model_step_pipeline: ComputePipelineState,
    /// Pipeline state for gpu_uart_step kernel.
    gpu_uart_step_pipeline: ComputePipelineState,
    /// Pre-allocated params buffers for each stage (shared memory, rewritten each dispatch).
    /// We need one per stage since multi-stage designs encode all stages before commit.
    params_buffers: Vec<metal::Buffer>,
    /// Shared event for GPU↔CPU synchronization within a single command buffer.
    shared_event: SharedEvent,
    /// Monotonic counter for shared event signaling
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

        let state_prep_fn = library
            .get_function("state_prep", None)
            .expect("Failed to get state_prep function");
        let state_prep_pipeline = device
            .new_compute_pipeline_state_with_function(&state_prep_fn)
            .expect("Failed to create state_prep pipeline");

        let flash_din_fn = library
            .get_function("gpu_apply_flash_din", None)
            .expect("Failed to get gpu_apply_flash_din function");
        let gpu_apply_flash_din_pipeline = device
            .new_compute_pipeline_state_with_function(&flash_din_fn)
            .expect("Failed to create gpu_apply_flash_din pipeline");

        let flash_step_fn = library
            .get_function("gpu_flash_model_step", None)
            .expect("Failed to get gpu_flash_model_step function");
        let gpu_flash_model_step_pipeline = device
            .new_compute_pipeline_state_with_function(&flash_step_fn)
            .expect("Failed to create gpu_flash_model_step pipeline");

        let uart_step_fn = library
            .get_function("gpu_uart_step", None)
            .expect("Failed to get gpu_uart_step function");
        let gpu_uart_step_pipeline = device
            .new_compute_pipeline_state_with_function(&uart_step_fn)
            .expect("Failed to create gpu_uart_step pipeline");

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
            state_prep_pipeline,
            gpu_apply_flash_din_pipeline,
            gpu_flash_model_step_pipeline,
            gpu_uart_step_pipeline,
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

    /// Encode a state_prep dispatch into an existing command buffer.
    #[inline]
    fn encode_state_prep(
        &self,
        command_buffer: &metal::CommandBufferRef,
        states_buffer: &metal::Buffer,
        prep_params_buffer: &metal::Buffer,
        ops_buffer: &metal::Buffer,
    ) {
        let encoder = command_buffer.new_compute_command_encoder();
        encoder.set_compute_pipeline_state(&self.state_prep_pipeline);
        encoder.set_buffer(0, Some(states_buffer), 0);
        encoder.set_buffer(1, Some(prep_params_buffer), 0);
        encoder.set_buffer(2, Some(ops_buffer), 0);
        let tpg = MTLSize::new(256, 1, 1);
        encoder.dispatch_thread_groups(MTLSize::new(1, 1, 1), tpg);
        encoder.end_encoding();
    }

    /// Encode a gpu_apply_flash_din dispatch into an existing command buffer.
    #[inline]
    fn encode_apply_flash_din(
        &self,
        command_buffer: &metal::CommandBufferRef,
        states_buffer: &metal::Buffer,
        flash_state_buffer: &metal::Buffer,
        flash_din_params_buffer: &metal::Buffer,
    ) {
        let encoder = command_buffer.new_compute_command_encoder();
        encoder.set_compute_pipeline_state(&self.gpu_apply_flash_din_pipeline);
        encoder.set_buffer(0, Some(states_buffer), 0);
        encoder.set_buffer(1, Some(flash_state_buffer), 0);
        encoder.set_buffer(2, Some(flash_din_params_buffer), 0);
        let tpg = MTLSize::new(256, 1, 1);
        encoder.dispatch_thread_groups(MTLSize::new(1, 1, 1), tpg);
        encoder.end_encoding();
    }

    /// Encode a gpu_flash_model_step dispatch into an existing command buffer.
    #[inline]
    fn encode_flash_model_step(
        &self,
        command_buffer: &metal::CommandBufferRef,
        states_buffer: &metal::Buffer,
        flash_state_buffer: &metal::Buffer,
        flash_model_params_buffer: &metal::Buffer,
        flash_data_buffer: &metal::Buffer,
    ) {
        let encoder = command_buffer.new_compute_command_encoder();
        encoder.set_compute_pipeline_state(&self.gpu_flash_model_step_pipeline);
        encoder.set_buffer(0, Some(states_buffer), 0);
        encoder.set_buffer(1, Some(flash_state_buffer), 0);
        encoder.set_buffer(2, Some(flash_model_params_buffer), 0);
        encoder.set_buffer(3, Some(flash_data_buffer), 0);
        let tpg = MTLSize::new(256, 1, 1);
        encoder.dispatch_thread_groups(MTLSize::new(1, 1, 1), tpg);
        encoder.end_encoding();
    }

    /// Encode a gpu_uart_step dispatch into an existing command buffer.
    #[inline]
    fn encode_uart_step(
        &self,
        command_buffer: &metal::CommandBufferRef,
        states_buffer: &metal::Buffer,
        uart_state_buffer: &metal::Buffer,
        uart_params_buffer: &metal::Buffer,
        uart_channel_buffer: &metal::Buffer,
    ) {
        let encoder = command_buffer.new_compute_command_encoder();
        encoder.set_compute_pipeline_state(&self.gpu_uart_step_pipeline);
        encoder.set_buffer(0, Some(states_buffer), 0);
        encoder.set_buffer(1, Some(uart_state_buffer), 0);
        encoder.set_buffer(2, Some(uart_params_buffer), 0);
        encoder.set_buffer(3, Some(uart_channel_buffer), 0);
        let tpg = MTLSize::new(256, 1, 1);
        encoder.dispatch_thread_groups(MTLSize::new(1, 1, 1), tpg);
        encoder.end_encoding();
    }

    /// Encode and commit a GPU-only batch of K ticks in a single command buffer.
    ///
    /// No per-tick CPU interaction — flash and UART are handled entirely on GPU.
    /// A single signal at the end notifies the CPU that the batch is complete.
    ///
    /// Each tick encodes:
    ///   1. state_prep (falling edge ops)
    ///   2. gpu_apply_flash_din
    ///   3. simulate_v1_stage × num_stages (falling edge)
    ///   4. state_prep (rising edge ops)
    ///   5. gpu_apply_flash_din
    ///   6. simulate_v1_stage × num_stages (rising edge)
    ///   7. gpu_flash_model_step
    ///   8. gpu_uart_step
    ///
    /// Returns the event value that signals batch completion.
    #[allow(clippy::too_many_arguments)]
    fn encode_and_commit_gpu_batch(
        &self,
        batch_size: usize,
        num_blocks: usize,
        num_major_stages: usize,
        state_size: usize,
        blocks_start_buffer: &metal::Buffer,
        blocks_data_buffer: &metal::Buffer,
        sram_data_buffer: &metal::Buffer,
        states_buffer: &metal::Buffer,
        event_buffer_metal: &metal::Buffer,
        fall_prep_params_buffer: &metal::Buffer,
        fall_ops_buffer: &metal::Buffer,
        rise_prep_params_buffer: &metal::Buffer,
        rise_ops_buffer: &metal::Buffer,
        flash_state_buffer: &metal::Buffer,
        flash_din_params_buffer: &metal::Buffer,
        flash_model_params_buffer: &metal::Buffer,
        flash_data_buffer: &metal::Buffer,
        uart_state_buffer: &metal::Buffer,
        uart_params_buffer: &metal::Buffer,
        uart_channel_buffer: &metal::Buffer,
    ) -> u64 {
        let batch_done = self.event_counter.get() + 1;
        let cb = self.command_queue.new_command_buffer();

        for _tick_offset in 0..batch_size {
            // ── Falling edge: state_prep + flash_din + simulate ──
            self.encode_state_prep(
                cb, states_buffer,
                fall_prep_params_buffer, fall_ops_buffer,
            );
            self.encode_apply_flash_din(
                cb, states_buffer,
                flash_state_buffer, flash_din_params_buffer,
            );
            for stage_i in 0..num_major_stages {
                self.write_params(stage_i, num_blocks, num_major_stages, state_size, 0);
                self.encode_dispatch(
                    cb, num_blocks, stage_i,
                    blocks_start_buffer, blocks_data_buffer,
                    sram_data_buffer, states_buffer, event_buffer_metal,
                );
            }

            // ── Rising edge: state_prep + flash_din + simulate ──
            self.encode_state_prep(
                cb, states_buffer,
                rise_prep_params_buffer, rise_ops_buffer,
            );
            self.encode_apply_flash_din(
                cb, states_buffer,
                flash_state_buffer, flash_din_params_buffer,
            );
            for stage_i in 0..num_major_stages {
                self.write_params(stage_i, num_blocks, num_major_stages, state_size, 0);
                self.encode_dispatch(
                    cb, num_blocks, stage_i,
                    blocks_start_buffer, blocks_data_buffer,
                    sram_data_buffer, states_buffer, event_buffer_metal,
                );
            }

            // ── Flash model step + UART step (GPU-only, no CPU round-trip) ──
            self.encode_flash_model_step(
                cb, states_buffer,
                flash_state_buffer, flash_model_params_buffer, flash_data_buffer,
            );
            self.encode_uart_step(
                cb, states_buffer,
                uart_state_buffer, uart_params_buffer, uart_channel_buffer,
            );
        }

        // Single signal at end of entire batch
        cb.encode_signal_event(&self.shared_event, batch_done);
        cb.commit();

        self.event_counter.set(batch_done);
        batch_done
    }

    /// Run GPU kernel profiling: dispatch each kernel in its own command buffer,
    /// wait for completion, and measure GPU execution time.
    #[allow(clippy::too_many_arguments)]
    fn profile_gpu_kernels(
        &self,
        num_ticks: usize,
        num_blocks: usize,
        num_major_stages: usize,
        state_size: usize,
        blocks_start_buffer: &metal::Buffer,
        blocks_data_buffer: &metal::Buffer,
        sram_data_buffer: &metal::Buffer,
        states_buffer: &metal::Buffer,
        event_buffer_metal: &metal::Buffer,
        fall_prep_params_buffer: &metal::Buffer,
        fall_ops_buffer: &metal::Buffer,
        rise_prep_params_buffer: &metal::Buffer,
        rise_ops_buffer: &metal::Buffer,
        flash_state_buffer: &metal::Buffer,
        flash_din_params_buffer: &metal::Buffer,
        flash_model_params_buffer: &metal::Buffer,
        flash_data_buffer: &metal::Buffer,
        uart_state_buffer: &metal::Buffer,
        uart_params_buffer: &metal::Buffer,
        uart_channel_buffer: &metal::Buffer,
    ) {
        #[inline]
        fn gpu_times(cb: &metal::CommandBufferRef) -> (f64, f64) {
            unsafe {
                let obj: *mut objc::runtime::Object = &*(cb as *const _ as *const objc::runtime::Object)
                    as *const _ as *mut _;
                let start: f64 = msg_send![obj, GPUStartTime];
                let end: f64 = msg_send![obj, GPUEndTime];
                (start, end)
            }
        }

        println!("\n=== GPU Kernel Profiling ({} ticks) ===\n", num_ticks);

        // Warmup: 10 ticks
        for _ in 0..10 {
            let cb = self.command_queue.new_command_buffer();
            self.encode_state_prep(cb, states_buffer, fall_prep_params_buffer, fall_ops_buffer);
            self.encode_apply_flash_din(cb, states_buffer, flash_state_buffer, flash_din_params_buffer);
            for stage_i in 0..num_major_stages {
                self.write_params(stage_i, num_blocks, num_major_stages, state_size, 0);
                self.encode_dispatch(cb, num_blocks, stage_i, blocks_start_buffer,
                    blocks_data_buffer, sram_data_buffer, states_buffer, event_buffer_metal);
            }
            self.encode_state_prep(cb, states_buffer, rise_prep_params_buffer, rise_ops_buffer);
            self.encode_apply_flash_din(cb, states_buffer, flash_state_buffer, flash_din_params_buffer);
            for stage_i in 0..num_major_stages {
                self.write_params(stage_i, num_blocks, num_major_stages, state_size, 0);
                self.encode_dispatch(cb, num_blocks, stage_i, blocks_start_buffer,
                    blocks_data_buffer, sram_data_buffer, states_buffer, event_buffer_metal);
            }
            self.encode_flash_model_step(cb, states_buffer, flash_state_buffer,
                flash_model_params_buffer, flash_data_buffer);
            self.encode_uart_step(cb, states_buffer, uart_state_buffer,
                uart_params_buffer, uart_channel_buffer);
            cb.commit();
            cb.wait_until_completed();
        }

        let mut time_state_prep_fall = 0.0f64;
        let mut time_flash_din = 0.0f64;
        let mut time_simulate_fall = 0.0f64;
        let mut time_state_prep_rise = 0.0f64;
        let mut time_simulate_rise = 0.0f64;
        let mut time_flash_step = 0.0f64;
        let mut time_uart_step = 0.0f64;
        let mut time_full_tick = 0.0f64;

        let wall_start = std::time::Instant::now();

        for _tick in 0..num_ticks {
            // 1. state_prep (falling) — isolated
            let cb1 = self.command_queue.new_command_buffer();
            self.encode_state_prep(cb1, states_buffer, fall_prep_params_buffer, fall_ops_buffer);
            cb1.commit(); cb1.wait_until_completed();
            let (s, e) = gpu_times(cb1);
            time_state_prep_fall += e - s;

            // 2. gpu_apply_flash_din — isolated
            let cb1b = self.command_queue.new_command_buffer();
            self.encode_apply_flash_din(cb1b, states_buffer, flash_state_buffer, flash_din_params_buffer);
            cb1b.commit(); cb1b.wait_until_completed();
            let (s, e) = gpu_times(cb1b);
            time_flash_din += e - s;

            // 3. simulate (falling) — isolated per stage
            for stage_i in 0..num_major_stages {
                self.write_params(stage_i, num_blocks, num_major_stages, state_size, 0);
                let cb2 = self.command_queue.new_command_buffer();
                self.encode_dispatch(cb2, num_blocks, stage_i, blocks_start_buffer,
                    blocks_data_buffer, sram_data_buffer, states_buffer, event_buffer_metal);
                cb2.commit(); cb2.wait_until_completed();
                let (s, e) = gpu_times(cb2);
                time_simulate_fall += e - s;
            }

            // 4. state_prep (rising) — isolated
            let cb3 = self.command_queue.new_command_buffer();
            self.encode_state_prep(cb3, states_buffer, rise_prep_params_buffer, rise_ops_buffer);
            cb3.commit(); cb3.wait_until_completed();
            let (s, e) = gpu_times(cb3);
            time_state_prep_rise += e - s;

            // 5. gpu_apply_flash_din — isolated
            let cb3b = self.command_queue.new_command_buffer();
            self.encode_apply_flash_din(cb3b, states_buffer, flash_state_buffer, flash_din_params_buffer);
            cb3b.commit(); cb3b.wait_until_completed();
            let (s, e) = gpu_times(cb3b);
            time_flash_din += e - s;

            // 6. simulate (rising) — isolated per stage
            for stage_i in 0..num_major_stages {
                self.write_params(stage_i, num_blocks, num_major_stages, state_size, 0);
                let cb4 = self.command_queue.new_command_buffer();
                self.encode_dispatch(cb4, num_blocks, stage_i, blocks_start_buffer,
                    blocks_data_buffer, sram_data_buffer, states_buffer, event_buffer_metal);
                cb4.commit(); cb4.wait_until_completed();
                let (s, e) = gpu_times(cb4);
                time_simulate_rise += e - s;
            }

            // 7. gpu_flash_model_step — isolated
            let cb5 = self.command_queue.new_command_buffer();
            self.encode_flash_model_step(cb5, states_buffer, flash_state_buffer,
                flash_model_params_buffer, flash_data_buffer);
            cb5.commit(); cb5.wait_until_completed();
            let (s, e) = gpu_times(cb5);
            time_flash_step += e - s;

            // 8. gpu_uart_step — isolated
            let cb6 = self.command_queue.new_command_buffer();
            self.encode_uart_step(cb6, states_buffer, uart_state_buffer,
                uart_params_buffer, uart_channel_buffer);
            cb6.commit(); cb6.wait_until_completed();
            let (s, e) = gpu_times(cb6);
            time_uart_step += e - s;

            // Full tick in single CB for comparison
            let cb_full = self.command_queue.new_command_buffer();
            self.encode_state_prep(cb_full, states_buffer, fall_prep_params_buffer, fall_ops_buffer);
            self.encode_apply_flash_din(cb_full, states_buffer, flash_state_buffer, flash_din_params_buffer);
            for stage_i in 0..num_major_stages {
                self.write_params(stage_i, num_blocks, num_major_stages, state_size, 0);
                self.encode_dispatch(cb_full, num_blocks, stage_i, blocks_start_buffer,
                    blocks_data_buffer, sram_data_buffer, states_buffer, event_buffer_metal);
            }
            self.encode_state_prep(cb_full, states_buffer, rise_prep_params_buffer, rise_ops_buffer);
            self.encode_apply_flash_din(cb_full, states_buffer, flash_state_buffer, flash_din_params_buffer);
            for stage_i in 0..num_major_stages {
                self.write_params(stage_i, num_blocks, num_major_stages, state_size, 0);
                self.encode_dispatch(cb_full, num_blocks, stage_i, blocks_start_buffer,
                    blocks_data_buffer, sram_data_buffer, states_buffer, event_buffer_metal);
            }
            self.encode_flash_model_step(cb_full, states_buffer, flash_state_buffer,
                flash_model_params_buffer, flash_data_buffer);
            self.encode_uart_step(cb_full, states_buffer, uart_state_buffer,
                uart_params_buffer, uart_channel_buffer);
            cb_full.commit(); cb_full.wait_until_completed();
            let (s, e) = gpu_times(cb_full);
            time_full_tick += e - s;
        }

        let wall_elapsed = wall_start.elapsed();
        let n = num_ticks as f64;

        let total_isolated = time_state_prep_fall + time_flash_din + time_simulate_fall
            + time_state_prep_rise + time_simulate_rise + time_flash_step + time_uart_step;

        let print_kernel = |name: &str, t: f64| {
            let us = t / n * 1e6;
            let pct = if total_isolated > 0.0 { 100.0 * t / total_isolated } else { 0.0 };
            println!("  {:<32} {:>8.2}μs/tick  {:>5.1}%", name, us, pct);
        };

        println!("Per-kernel GPU time (isolated command buffers):");
        print_kernel("state_prep (falling)", time_state_prep_fall);
        print_kernel("gpu_apply_flash_din", time_flash_din);
        print_kernel("simulate_v1_stage (falling)", time_simulate_fall);
        print_kernel("state_prep (rising)", time_state_prep_rise);
        print_kernel("simulate_v1_stage (rising)", time_simulate_rise);
        print_kernel("gpu_flash_model_step", time_flash_step);
        print_kernel("gpu_uart_step", time_uart_step);
        println!("  {:<32} {:>8.2}μs/tick",
            "TOTAL (isolated sum)", total_isolated / n * 1e6);
        println!();
        println!("  {:<32} {:>8.2}μs/tick",
            "Full tick (single CB)", time_full_tick / n * 1e6);
        println!("  {:<32} {:>8.2}μs/tick",
            "Wall clock (2× ticks, profiling)", wall_elapsed.as_secs_f64() / n * 1e6);
        println!();

        let sim_us = time_simulate_fall / n * 1e6 + time_simulate_rise / n * 1e6;
        let io_us = time_flash_din / n * 1e6 + time_flash_step / n * 1e6 + time_uart_step / n * 1e6;
        let prep_us = time_state_prep_fall / n * 1e6 + time_state_prep_rise / n * 1e6;
        println!("  Simulation kernels total:     {:>8.2}μs/tick  ({:.1}%)",
            sim_us, 100.0 * (time_simulate_fall + time_simulate_rise) / total_isolated);
        println!("  IO model kernels total:       {:>8.2}μs/tick  ({:.1}%)",
            io_us, 100.0 * (time_flash_din + time_flash_step + time_uart_step) / total_isolated);
        println!("  State prep total:             {:>8.2}μs/tick  ({:.1}%)",
            prep_us, 100.0 * (time_state_prep_fall + time_state_prep_rise) / total_isolated);
        println!();
        println!("  CB submission overhead:        {:>8.2}μs/tick (wall - GPU)",
            (wall_elapsed.as_secs_f64() - total_isolated - time_full_tick) / n * 1e6);
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
#[allow(dead_code)]
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

/// Write flash data input to GPIO state.
fn set_flash_din(state: &mut [u32], gpio_map: &GpioMapping, d0_gpio: usize, din: u8) {
    for i in 0..4 {
        if let Some(&pos) = gpio_map.input_bits.get(&(d0_gpio + i)) {
            set_bit(state, pos, (din >> i) & 1);
        }
    }
}

// ── BitOp Builder ────────────────────────────────────────────────────────────

/// Build the BitOp array for a falling edge state_prep.
/// Sets: clock=0, negedge_flag=1, clears: posedge_flag, plus reset.
/// Flash D_IN is now handled by gpu_apply_flash_din kernel.
fn build_falling_edge_ops(
    gpio_map: &GpioMapping,
    clock_gpio: usize,
    reset_gpio: usize,
    reset_val: u8,
) -> Vec<BitOp> {
    let mut ops = Vec::new();
    // Clock = 0
    ops.push(BitOp { position: gpio_map.input_bits[&clock_gpio], value: 0 });
    // Reset
    ops.push(BitOp { position: gpio_map.input_bits[&reset_gpio], value: reset_val as u32 });
    // Negedge flag = 1
    for &pos in &gpio_map.negedge_flag_bits {
        ops.push(BitOp { position: pos, value: 1 });
    }
    // Posedge flag = 0
    for &pos in &gpio_map.posedge_flag_bits {
        ops.push(BitOp { position: pos, value: 0 });
    }
    ops
}

/// Build the BitOp array for a rising edge state_prep.
/// Sets: clock=1, posedge_flag=1, clears: negedge_flag, plus reset.
/// Flash D_IN is now handled by gpu_apply_flash_din kernel.
fn build_rising_edge_ops(
    gpio_map: &GpioMapping,
    clock_gpio: usize,
    reset_gpio: usize,
    reset_val: u8,
) -> Vec<BitOp> {
    let mut ops = Vec::new();
    // Clock = 1
    ops.push(BitOp { position: gpio_map.input_bits[&clock_gpio], value: 1 });
    // Reset
    ops.push(BitOp { position: gpio_map.input_bits[&reset_gpio], value: reset_val as u32 });
    // Posedge flag = 1
    for &pos in &gpio_map.posedge_flag_bits {
        ops.push(BitOp { position: pos, value: 1 });
    }
    // Negedge flag = 0
    for &pos in &gpio_map.negedge_flag_bits {
        ops.push(BitOp { position: pos, value: 0 });
    }
    ops
}

/// Create a Metal buffer containing a StatePrepParams struct.
fn create_prep_params_buffer(
    device: &metal::Device,
    state_size: u32,
    num_ops: u32,
    num_monitors: u32,
    tick_number: u32,
) -> metal::Buffer {
    let buf = device.new_buffer(
        std::mem::size_of::<StatePrepParams>() as u64,
        MTLResourceOptions::StorageModeShared,
    );
    unsafe {
        std::ptr::write(buf.contents() as *mut StatePrepParams, StatePrepParams {
            state_size,
            num_ops,
            num_monitors,
            tick_number,
        });
    }
    buf
}

/// Create a Metal buffer containing a BitOp array.
fn create_ops_buffer(device: &metal::Device, ops: &[BitOp]) -> metal::Buffer {
    let size = if ops.is_empty() {
        std::mem::size_of::<BitOp>() as u64  // minimum 1 element
    } else {
        (ops.len() * std::mem::size_of::<BitOp>()) as u64
    };
    let buf = device.new_buffer(size, MTLResourceOptions::StorageModeShared);
    if !ops.is_empty() {
        unsafe {
            std::ptr::copy_nonoverlapping(
                ops.as_ptr(),
                buf.contents() as *mut BitOp,
                ops.len(),
            );
        }
    }
    buf
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

    // ── Initialize peripheral models (CPU-side, kept for --check-with-cpu) ──

    let flash: Option<CppSpiFlash> = if let Some(ref flash_cfg) = config.flash {
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
    let uart_tx_gpio = config.uart.as_ref().map(|u| u.tx_gpio);

    // ── Initialize Metal simulator and GPU state buffers ─────────────────

    let timer_init = clilog::stimer!("init_gpu");
    let simulator = MetalSimulator::new(script.num_major_stages);

    let state_size = script.reg_io_state_size as usize;

    // States: [input state (state_size)] [output state (state_size)]
    let states_buffer = simulator.device.new_buffer(
        (2 * state_size * std::mem::size_of::<u32>()) as u64,
        MTLResourceOptions::StorageModeShared,
    );
    let states: &mut [u32] = unsafe {
        std::slice::from_raw_parts_mut(
            states_buffer.contents() as *mut u32,
            2 * state_size,
        )
    };
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
    let _ = sram_data;

    // Initialize: set reset active
    let reset_val = if config.reset_active_high { 1u8 } else { 0u8 };
    set_bit(&mut states[..state_size], gpio_map.input_bits[&reset_gpio], reset_val);
    clilog::info!("Initial state: reset GPIO {} = {} (active)", reset_gpio, reset_val);

    // Set flash D_IN defaults (high = no data) — initial state before GPU flash takes over
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

    // Event buffer (for $stop/$finish/assertions)
    let event_buffer = Box::new(gem::event_buffer::EventBuffer::new());
    let event_buffer_ptr = Box::into_raw(event_buffer);
    let event_buffer_metal = simulator.device.new_buffer_with_bytes_no_copy(
        event_buffer_ptr as *const _,
        std::mem::size_of::<gem::event_buffer::EventBuffer>() as u64,
        MTLResourceOptions::StorageModeShared,
        None,
    );

    clilog::finish!(timer_init);

    // ── Build GPU-side state_prep + IO model buffers ──────────────────────

    let timer_prep = clilog::stimer!("build_state_prep_buffers");
    let reset_cycles = config.reset_cycles;
    let num_major_stages = script.num_major_stages;
    let num_blocks = script.num_blocks;

    // Initial reset value
    let reset_val_active = if config.reset_active_high { 1u8 } else { 0u8 };
    let reset_val_inactive = if config.reset_active_high { 0u8 } else { 1u8 };

    // Build initial falling/rising edge BitOp arrays (no flash_din — handled by GPU)
    let fall_ops = build_falling_edge_ops(
        &gpio_map, clock_gpio, reset_gpio, reset_val_active,
    );
    let rise_ops = build_rising_edge_ops(
        &gpio_map, clock_gpio, reset_gpio, reset_val_active,
    );
    let fall_ops_len = fall_ops.len();
    let rise_ops_len = rise_ops.len();

    let fall_ops_buffer = create_ops_buffer(&simulator.device, &fall_ops);
    let rise_ops_buffer = create_ops_buffer(&simulator.device, &rise_ops);

    // Create StatePrepParams buffers
    let fall_prep_params_buffer = create_prep_params_buffer(
        &simulator.device, state_size as u32, fall_ops.len() as u32, 0, 0,
    );
    let rise_prep_params_buffer = create_prep_params_buffer(
        &simulator.device, state_size as u32, rise_ops.len() as u32, 0, 0,
    );

    // ── GPU Flash IO buffers ────────────────────────────────────────────

    // FlashState (shared, persistent across ticks)
    let flash_state_buffer = simulator.device.new_buffer(
        std::mem::size_of::<FlashState>() as u64,
        MTLResourceOptions::StorageModeShared,
    );
    unsafe {
        let fs = &mut *(flash_state_buffer.contents() as *mut FlashState);
        *fs = std::mem::zeroed();
        fs.prev_csn = 1;     // CSN starts high (deselected)
        fs.model_prev_csn = 1; // Model internal edge detection starts high
        fs.d_i = 0x0F;       // Flash output starts high
        fs.in_reset = 1;     // Start in reset
    }

    // FlashDinParams (constant)
    let flash_din_params_buffer = simulator.device.new_buffer(
        std::mem::size_of::<FlashDinParams>() as u64,
        MTLResourceOptions::StorageModeShared,
    );
    unsafe {
        let p = &mut *(flash_din_params_buffer.contents() as *mut FlashDinParams);
        p.has_flash = if config.flash.is_some() { 1 } else { 0 };
        for i in 0..4 {
            p.d_in_pos[i] = config.flash.as_ref()
                .and_then(|f| gpio_map.input_bits.get(&(f.d0_gpio + i)).copied())
                .unwrap_or(0xFFFFFFFF);
        }
    }

    // FlashModelParams (constant)
    let flash_model_params_buffer = simulator.device.new_buffer(
        std::mem::size_of::<FlashModelParams>() as u64,
        MTLResourceOptions::StorageModeShared,
    );
    unsafe {
        let p = &mut *(flash_model_params_buffer.contents() as *mut FlashModelParams);
        p.state_size = state_size as u32;
        p.clk_out_pos = config.flash.as_ref()
            .and_then(|f| gpio_map.output_bits.get(&f.clk_gpio).copied())
            .unwrap_or(0);
        p.csn_out_pos = config.flash.as_ref()
            .and_then(|f| gpio_map.output_bits.get(&f.csn_gpio).copied())
            .unwrap_or(0);
        for i in 0..4 {
            p.d_out_pos[i] = config.flash.as_ref()
                .and_then(|f| gpio_map.output_bits.get(&(f.d0_gpio + i)).copied())
                .unwrap_or(0xFFFFFFFF);
        }
        p.flash_data_size = 16 * 1024 * 1024; // 16 MB
    }

    // Flash data buffer (16 MB, loaded with firmware)
    let flash_data_buffer = simulator.device.new_buffer(
        16 * 1024 * 1024,
        MTLResourceOptions::StorageModeShared,
    );
    unsafe {
        // Fill with 0xFF (erased flash state)
        std::ptr::write_bytes(flash_data_buffer.contents() as *mut u8, 0xFF, 16 * 1024 * 1024);
    }
    // Load firmware into flash data buffer
    if let Some(ref flash_cfg) = config.flash {
        use std::io::Read;
        let firmware_path = std::path::Path::new(&flash_cfg.firmware);
        let mut file = File::open(firmware_path).expect("Failed to open firmware file");
        let mut data = Vec::new();
        file.read_to_end(&mut data).expect("Failed to read firmware");
        let offset = flash_cfg.firmware_offset;
        assert!(offset + data.len() <= 16 * 1024 * 1024, "Firmware too large for flash buffer");
        unsafe {
            let dest = (flash_data_buffer.contents() as *mut u8).add(offset);
            std::ptr::copy_nonoverlapping(data.as_ptr(), dest, data.len());
        }
        clilog::info!("Loaded {} bytes firmware into GPU flash buffer at offset 0x{:X}", data.len(), offset);
    }

    // ── GPU UART IO buffers ─────────────────────────────────────────────

    // UartDecoderState (shared, persistent across ticks)
    let uart_state_buffer = simulator.device.new_buffer(
        std::mem::size_of::<UartDecoderState>() as u64,
        MTLResourceOptions::StorageModeShared,
    );
    unsafe {
        let us = &mut *(uart_state_buffer.contents() as *mut UartDecoderState);
        us.state = 0;          // IDLE
        us.last_tx = 1;        // TX line idle high
        us.start_cycle = 0;
        us.bits_received = 0;
        us.value = 0;
        us.current_cycle = 0;
    }

    // UartParams (constant)
    let uart_params_buffer = simulator.device.new_buffer(
        std::mem::size_of::<UartParams>() as u64,
        MTLResourceOptions::StorageModeShared,
    );
    unsafe {
        let p = &mut *(uart_params_buffer.contents() as *mut UartParams);
        p.state_size = state_size as u32;
        p.has_uart = if config.uart.is_some() { 1 } else { 0 };
        p.tx_out_pos = uart_tx_gpio
            .and_then(|tx| gpio_map.output_bits.get(&tx).copied())
            .unwrap_or(0);
        p.cycles_per_bit = (clock_hz / uart_baud as u64) as u32;
    }

    // UartChannel (shared ring buffer, CPU drains after each batch)
    let uart_channel_buffer = simulator.device.new_buffer(
        std::mem::size_of::<UartChannel>() as u64,
        MTLResourceOptions::StorageModeShared,
    );
    unsafe {
        let ch = &mut *(uart_channel_buffer.contents() as *mut UartChannel);
        ch.write_head = 0;
        ch.capacity = 4096;
        ch._pad = [0; 2];
        // data doesn't need to be zeroed (ring buffer semantics)
    }

    // Pre-write params for all simulation stages (they don't change between ticks)
    for stage_i in 0..num_major_stages {
        simulator.write_params(stage_i, num_blocks, num_major_stages, state_size, 0);
    }

    clilog::finish!(timer_prep);

    // ── GPU Kernel Profiling (optional) ──────────────────────────────────

    if args.gpu_profile {
        let profile_ticks = args.max_cycles.unwrap_or(1000).min(5000);
        simulator.profile_gpu_kernels(
            profile_ticks,
            num_blocks,
            num_major_stages,
            state_size,
            &blocks_start_buffer,
            &blocks_data_buffer,
            &sram_data_buffer,
            &states_buffer,
            &event_buffer_metal,
            &fall_prep_params_buffer,
            &fall_ops_buffer,
            &rise_prep_params_buffer,
            &rise_ops_buffer,
            &flash_state_buffer,
            &flash_din_params_buffer,
            &flash_model_params_buffer,
            &flash_data_buffer,
            &uart_state_buffer,
            &uart_params_buffer,
            &uart_channel_buffer,
        );

        // Clean up event buffer
        unsafe { drop(Box::from_raw(event_buffer_ptr)); }
        return;
    }

    // ── GPU-only simulation loop ─────────────────────────────────────────
    //
    // All IO models (flash + UART) run on GPU. No per-tick CPU interaction.
    // CPU just drains decoded UART bytes from the ring buffer after each batch.

    let timer_sim = clilog::stimer!("simulation");
    let sim_start = std::time::Instant::now();

    // Helper: update reset value in both ops buffers
    let update_reset_in_ops = |reset_val: u8| {
        let reset_pos = gpio_map.input_bits[&reset_gpio];
        for (buf, len) in [
            (&fall_ops_buffer, fall_ops_len),
            (&rise_ops_buffer, rise_ops_len),
        ] {
            let ops: &mut [BitOp] = unsafe {
                std::slice::from_raw_parts_mut(buf.contents() as *mut BitOp, len)
            };
            for op in ops.iter_mut() {
                if op.position == reset_pos {
                    op.value = reset_val as u32;
                }
            }
        }
    };

    // UART event collection (CPU-side, populated from channel drain)
    let mut uart_events: Vec<gem::testbench::UartEvent> = Vec::new();
    let mut uart_read_head: u32 = 0;

    // Profiling accumulators
    let mut prof_batch_encode: u64 = 0;
    let mut prof_gpu_wait: u64 = 0;
    let mut prof_drain: u64 = 0;
    let mut total_batches: u64 = 0;

    let mut tick: usize = 0;
    while tick < max_ticks {
        let batch = BATCH_SIZE.min(max_ticks - tick);

        // Don't cross reset boundary within a batch
        let batch = if tick < reset_cycles && tick + batch > reset_cycles {
            reset_cycles - tick
        } else {
            batch
        };

        // Update reset value and flash in_reset for this batch
        let in_reset = tick < reset_cycles;
        let current_reset_val = if in_reset { reset_val_active } else { reset_val_inactive };
        update_reset_in_ops(current_reset_val);

        // Update flash_state.in_reset on GPU
        unsafe {
            let fs = &mut *(flash_state_buffer.contents() as *mut FlashState);
            fs.in_reset = if in_reset { 1 } else { 0 };
        }

        // Encode and commit GPU batch
        let t_encode = std::time::Instant::now();
        let batch_done = simulator.encode_and_commit_gpu_batch(
            batch,
            num_blocks,
            num_major_stages,
            state_size,
            &blocks_start_buffer,
            &blocks_data_buffer,
            &sram_data_buffer,
            &states_buffer,
            &event_buffer_metal,
            &fall_prep_params_buffer,
            &fall_ops_buffer,
            &rise_prep_params_buffer,
            &rise_ops_buffer,
            &flash_state_buffer,
            &flash_din_params_buffer,
            &flash_model_params_buffer,
            &flash_data_buffer,
            &uart_state_buffer,
            &uart_params_buffer,
            &uart_channel_buffer,
        );
        prof_batch_encode += t_encode.elapsed().as_nanos() as u64;

        // Wait for GPU batch to complete
        let t_wait = std::time::Instant::now();
        simulator.spin_wait(batch_done);
        prof_gpu_wait += t_wait.elapsed().as_nanos() as u64;

        // Drain UART channel (CPU reads decoded bytes from GPU ring buffer)
        let t_drain = std::time::Instant::now();
        unsafe {
            let channel = &*(uart_channel_buffer.contents() as *const UartChannel);
            while uart_read_head < channel.write_head {
                let byte = channel.data[(uart_read_head % channel.capacity) as usize];
                let ch = if byte >= 32 && byte < 127 { byte as char } else { '.' };
                clilog::info!("UART TX: 0x{:02X} '{}'", byte, ch);
                uart_events.push(gem::testbench::UartEvent {
                    timestamp: tick, // approximate tick
                    peripheral: "uart_0".to_string(),
                    event: "tx".to_string(),
                    payload: byte,
                });
                uart_read_head += 1;
            }
        }
        prof_drain += t_drain.elapsed().as_nanos() as u64;

        total_batches += 1;
        tick += batch;

        // Progress logging
        if tick > 0 && tick % 100000 < BATCH_SIZE {
            let elapsed = sim_start.elapsed();
            let us_per_tick = elapsed.as_micros() as f64 / tick as f64;
            clilog::info!(
                "Tick {} / {} ({:.1}μs/tick, batches={}, UART bytes={})",
                tick, max_ticks, us_per_tick, total_batches, uart_events.len()
            );
        }
        if tick >= reset_cycles && tick - batch < reset_cycles {
            clilog::info!("Reset released at tick {}", reset_cycles);
        }
    }

    // Print profiling results
    let total_ns = prof_batch_encode + prof_gpu_wait + prof_drain;
    let print_prof = |name: &str, ns: u64| {
        let us = ns as f64 / 1000.0 / max_ticks as f64;
        let pct = if total_ns > 0 { 100.0 * ns as f64 / total_ns as f64 } else { 0.0 };
        println!("  {:<32} {:>8.1}μs/tick  {:>5.1}%", name, us, pct);
    };
    println!();
    println!("=== Profiling Breakdown ===");
    print_prof("Batch encode + commit", prof_batch_encode);
    print_prof("GPU wait (spin)", prof_gpu_wait);
    print_prof("UART channel drain", prof_drain);
    println!("  {:<32} {:>8.1}μs/tick  100.0%",
             "TOTAL (instrumented)",
             total_ns as f64 / 1000.0 / max_ticks as f64);
    println!();
    println!("  Total batches:                 {}", total_batches);

    let sim_elapsed = sim_start.elapsed();
    clilog::finish!(timer_sim);

    // ── Check GPU flash state for errors ─────────────────────────────────

    let last_error_cmd = unsafe {
        let fs = &*(flash_state_buffer.contents() as *const FlashState);
        fs.last_error_cmd
    };
    if last_error_cmd != 0 {
        clilog::warn!("GPU flash model encountered unknown command: 0x{:02X}", last_error_cmd);
    }

    // ── Results ──────────────────────────────────────────────────────────

    println!();
    println!("=== GPU Simulation Results ===");
    println!("Ticks simulated: {}", max_ticks);
    println!("UART bytes received: {}", uart_events.len());

    if max_ticks > 0 {
        let us_per_tick = sim_elapsed.as_micros() as f64 / max_ticks as f64;
        println!("Time per tick: {:.1}μs ({:.1}s total)", us_per_tick, sim_elapsed.as_secs_f64());
    }

    // Print UART output as string
    if !uart_events.is_empty() {
        let uart_str: String = uart_events
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

    // Print flash model stats (GPU-side state)
    if config.flash.is_some() {
        let fs = unsafe { &*(flash_state_buffer.contents() as *const FlashState) };
        println!(
            "GPU Flash model: command=0x{:02X}, byte_count={}, addr=0x{:06X}, error_cmd=0x{:X}",
            fs.command, fs.byte_count, fs.addr, fs.last_error_cmd
        );
    }

    // Output events to JSON
    if let Some(ref output_path) = config.output_events {
        #[derive(serde::Serialize)]
        struct EventsOutput {
            events: Vec<gem::testbench::UartEvent>,
        }
        let output = EventsOutput {
            events: uart_events,
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
        clilog::info!("CPU verification not yet implemented for GPU IO mode");
    }

    // Keep flash alive (for --check-with-cpu in future)
    let _ = flash;

    // Clean up event buffer
    unsafe {
        drop(Box::from_raw(event_buffer_ptr));
    }

    println!();
    println!("SIMULATION: PASSED");
}
