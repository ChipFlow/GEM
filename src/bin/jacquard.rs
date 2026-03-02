// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Unified CLI for the Jacquard GPU-accelerated RTL simulator.

use std::path::PathBuf;

use clap::{Parser, Subcommand};
use jacquard::aig::AIG;
use jacquard::aigpdk::AIGPDKLeafPins;
use jacquard::config::JacquardConfig;
use jacquard::pe::{process_partitions, Partition};
use jacquard::repcut::RCHyperGraph;
use jacquard::sim::setup::{DesignArgs, PartitionFile};
use jacquard::sky130::{detect_library_from_file, CellLibrary, SKY130LeafPins};
use jacquard::staging::build_staged_aigs;
use netlistdb::NetlistDB;
use rayon::prelude::*;

#[derive(Parser)]
#[command(name = "jacquard", about = "Jacquard — GPU-accelerated RTL logic simulator", version, long_version = concat!(env!("CARGO_PKG_VERSION"), "\n\n", "See https://github.com/chipflow/jacquard for documentation"))]
struct Cli {
    #[command(subcommand)]
    command: Commands,

    /// Path to project configuration file.
    ///
    /// If not specified, Jacquard searches for `jacquard.toml` in the current
    /// directory and parent directories. CLI arguments override config file values.
    #[clap(short, long, global = true)]
    config: Option<PathBuf>,

    /// Increase verbosity (can be repeated: -vv for trace level).
    #[clap(short, long, global = true, action = clap::ArgAction::Count)]
    verbose: u8,

    /// Decrease verbosity (can be repeated: -qq for error level).
    #[clap(short, long, global = true, action = clap::ArgAction::Count)]
    quiet: u8,
}

#[derive(Subcommand)]
enum Commands {
    /// Map a synthesized gate-level netlist to a .gemparts partition file.
    ///
    /// This is the first step in the Jacquard workflow. The resulting .gemparts file
    /// is then used by `jacquard sim` to run the design on a GPU.
    Map(MapArgs),

    /// Run a GPU simulation with VCD input/output.
    ///
    /// Reads a gate-level netlist and partition file, processes a VCD input
    /// waveform through the GPU simulator, and writes the output VCD.
    /// Requires building with `--features metal` (macOS), `--features cuda` (Linux/NVIDIA),
    /// or `--features hip` (Linux/AMD).
    Sim(SimArgs),

    /// Run a GPU co-simulation with SPI flash and UART models (Metal only).
    ///
    /// Reads a gate-level netlist and testbench configuration JSON, then runs
    /// a cycle-accurate co-simulation with GPU-side SPI flash and UART models.
    /// Requires building with `--features metal`.
    Cosim(CosimArgs),
}

#[derive(Parser)]
struct MapArgs {
    /// Gate-level Verilog path synthesized with the AIGPDK or SKY130 library.
    ///
    /// If your design is still at RTL level, you must synthesize it first.
    /// See usage.md for synthesis instructions.
    /// Can also be set via `design.netlist` in jacquard.toml.
    netlist_verilog: Option<PathBuf>,

    /// Output path for the serialized partition file (.gemparts).
    ///
    /// Can also be set via `map.output` in jacquard.toml.
    parts_out: Option<PathBuf>,

    /// Top module name in the netlist.
    ///
    /// If not specified, Jacquard guesses the top module from the hierarchy.
    #[clap(long)]
    top_module: Option<String>,

    /// Level split thresholds for deep circuits.
    ///
    /// If mapping fails because a single endpoint cannot be partitioned,
    /// add level-split thresholds (e.g. --level-split 30 or --level-split 20,40).
    /// Remember to pass the same thresholds when simulating.
    #[clap(long, value_delimiter = ',')]
    level_split: Vec<usize>,

    /// Maximum stage degradation layers allowed during partition merging.
    ///
    /// Default is 0, meaning no degradation is allowed.
    #[clap(long, default_value_t = 0)]
    max_stage_degrad: usize,

    /// Enable selective X-propagation analysis (informational only at map time).
    ///
    /// Reports how many pins and partitions would be X-capable. The actual
    /// X-propagation is enabled at simulation time with `jacquard sim --xprop`.
    #[clap(long)]
    xprop: bool,
}

#[derive(Parser)]
struct SimArgs {
    /// Gate-level Verilog path synthesized with AIGPDK or SKY130 library.
    ///
    /// Can also be set via `design.netlist` in jacquard.toml.
    netlist_verilog: Option<PathBuf>,

    /// Pre-compiled partition mapping (.gemparts file).
    ///
    /// Can also be set via `sim.gemparts` or `map.output` in jacquard.toml.
    gemparts: Option<PathBuf>,

    /// VCD input signal path.
    ///
    /// Can also be set via `sim.input_vcd` in jacquard.toml.
    input_vcd: Option<PathBuf>,

    /// Output VCD path (must be writable).
    ///
    /// Can also be set via `sim.output_vcd` in jacquard.toml.
    output_vcd: Option<PathBuf>,

    /// Number of GPU blocks to use.
    ///
    /// For CUDA: set to 2x the number of GPU SMs.
    /// For HIP: set to 2x the number of GPU CUs.
    /// For Metal: set to 1.
    /// If not provided, defaults to 64 (suitable for many NVIDIA/AMD GPUs).
    #[clap(long)]
    num_blocks: Option<usize>,

    /// Top module name in the netlist.
    #[clap(long)]
    top_module: Option<String>,

    /// Level split thresholds (must match values used during mapping).
    #[clap(long, value_delimiter = ',')]
    level_split: Vec<usize>,

    /// The scope path of top module in the input VCD.
    #[clap(long)]
    input_vcd_scope: Option<String>,

    /// The scope path of top module in the output VCD.
    #[clap(long)]
    output_vcd_scope: Option<String>,

    /// Verify GPU results against CPU baseline.
    #[clap(long)]
    check_with_cpu: bool,

    /// Limit the number of simulated cycles.
    #[clap(long)]
    max_cycles: Option<usize>,

    /// JSON file path for extracting display format strings.
    #[clap(long)]
    json_path: Option<PathBuf>,

    /// Path to SDF file for per-instance back-annotated delays.
    #[clap(long)]
    sdf: Option<PathBuf>,

    /// SDF corner selection: min, typ, or max.
    #[clap(long, default_value = "typ")]
    sdf_corner: String,

    /// Enable SDF debug output.
    #[clap(long)]
    sdf_debug: bool,

    /// Enable selective X-propagation.
    ///
    /// Tracks unknown (X) values through DFF and SRAM outputs. Only partitions
    /// containing X-capable signals pay the overhead. X values appear in the
    /// output VCD as 'x'.
    #[clap(long)]
    xprop: bool,

    /// Enable timing analysis after simulation.
    #[clap(long)]
    enable_timing: bool,

    /// Clock period in picoseconds for timing analysis.
    #[clap(long, default_value = "1000")]
    timing_clock_period: u64,

    /// Report all timing violations (not just summary).
    #[clap(long)]
    timing_report_violations: bool,

    /// Path to Liberty library file for timing data.
    #[clap(long)]
    liberty: Option<PathBuf>,
}

#[derive(Parser)]
struct CosimArgs {
    /// Gate-level Verilog path synthesized with AIGPDK or SKY130 library.
    ///
    /// Can also be set via `design.netlist` in jacquard.toml.
    netlist_verilog: Option<PathBuf>,

    /// Pre-compiled partition mapping (.gemparts file).
    ///
    /// Can also be set via `sim.gemparts` or `map.output` in jacquard.toml.
    gemparts: Option<PathBuf>,

    /// Testbench configuration JSON file.
    ///
    /// Can also be set via `cosim.config` in jacquard.toml.
    #[clap(long)]
    config: Option<PathBuf>,

    /// Top module name in the netlist.
    #[clap(long)]
    top_module: Option<String>,

    /// Level split thresholds (comma-separated).
    #[clap(long, value_delimiter = ',')]
    level_split: Vec<usize>,

    /// Number of GPU threadgroups (blocks).
    #[clap(long, default_value = "64")]
    num_blocks: usize,

    /// Maximum system clock ticks to simulate.
    #[clap(long)]
    max_cycles: Option<usize>,

    /// Enable verbose flash model debug output.
    #[clap(long)]
    flash_verbose: bool,

    /// Clock period in picoseconds.
    #[clap(long)]
    clock_period: Option<u64>,

    /// Verify GPU results against CPU baseline.
    #[clap(long)]
    check_with_cpu: bool,

    /// Run GPU kernel profiling.
    #[clap(long)]
    gpu_profile: bool,

    /// Path to SDF file for per-instance back-annotated delays.
    #[clap(long)]
    sdf: Option<PathBuf>,

    /// SDF corner selection: min, typ, or max.
    #[clap(long, default_value = "typ")]
    sdf_corner: String,

    /// Enable SDF debug output.
    #[clap(long)]
    sdf_debug: bool,
}

/// Invoke the mt-kahypar partitioner.
fn run_par(hg: &RCHyperGraph, num_parts: usize) -> Vec<Vec<usize>> {
    clilog::debug!("invoking partitioner (#parts {})", num_parts);
    // mt-kahypar requires k >= 2, handle k=1 manually
    if num_parts == 1 {
        return vec![(0..hg.num_vertices()).collect()];
    }

    let parts_ids = hg.partition(num_parts);
    let mut parts = vec![vec![]; num_parts];
    for (i, part_id) in parts_ids.into_iter().enumerate() {
        parts[part_id].push(i);
    }
    parts
}

fn cmd_map(args: MapArgs, config: &Option<JacquardConfig>) {
    // Merge CLI args with config file values (CLI wins)
    let netlist_verilog = args.netlist_verilog
        .or_else(|| config.as_ref().and_then(|c| c.design.netlist.clone()))
        .unwrap_or_else(|| {
            eprintln!("Error: netlist_verilog is required (positional arg or design.netlist in jacquard.toml)");
            std::process::exit(1);
        });
    let parts_out = args.parts_out
        .or_else(|| config.as_ref().and_then(|c| c.map.output.clone()))
        .unwrap_or_else(|| {
            // Default to netlist path with .gemparts extension
            netlist_verilog.with_extension("gemparts")
        });
    let top_module = args.top_module
        .or_else(|| config.as_ref().and_then(|c| c.design.top_module.clone()));
    let level_split = if args.level_split.is_empty() {
        config.as_ref().map(|c| c.map.level_split.clone()).unwrap_or_default()
    } else {
        args.level_split
    };
    let max_stage_degrad = if args.max_stage_degrad != 0 {
        args.max_stage_degrad
    } else {
        config.as_ref().and_then(|c| c.map.max_stage_degrad).unwrap_or(0)
    };
    let xprop = args.xprop || config.as_ref().and_then(|c| c.map.xprop).unwrap_or(false);

    clilog::info!("Jacquard map args:\n{:#?}", netlist_verilog);

    // Detect cell library
    let lib = detect_library_from_file(&netlist_verilog).expect("Failed to read netlist file");
    clilog::info!("Detected cell library: {}", lib);

    if lib == CellLibrary::Mixed {
        panic!("Mixed AIGPDK and SKY130 cells in netlist not supported");
    }

    let netlistdb = match lib {
        CellLibrary::SKY130 => NetlistDB::from_sverilog_file(
            &netlist_verilog,
            top_module.as_deref(),
            &SKY130LeafPins,
        )
        .expect("cannot build netlist"),
        CellLibrary::AIGPDK | CellLibrary::Mixed => NetlistDB::from_sverilog_file(
            &netlist_verilog,
            top_module.as_deref(),
            &AIGPDKLeafPins(),
        )
        .expect("cannot build netlist"),
    };

    let aig = AIG::from_netlistdb(&netlistdb);
    println!(
        "netlist has {} pins, {} aig pins, {} and gates",
        netlistdb.num_pins,
        aig.num_aigpins,
        aig.and_gate_cache.len()
    );

    if xprop {
        let (_x_capable, stats) = aig.compute_x_capable_pins();
        println!(
            "X-propagation analysis: {}/{} pins ({:.1}%) X-capable, {} X-sources, {} fixpoint iterations",
            stats.num_x_capable_pins,
            stats.total_pins,
            if stats.total_pins > 0 {
                stats.num_x_capable_pins as f64 / stats.total_pins as f64 * 100.0
            } else {
                0.0
            },
            stats.num_x_sources,
            stats.fixpoint_iterations,
        );
    }

    let stageds = build_staged_aigs(&aig, &level_split);

    let stages_effective_parts = stageds
        .iter()
        .map(|&(l, r, ref staged)| {
            clilog::info!(
                "interactive partitioning stage {}-{}",
                l,
                match r {
                    usize::MAX => "max".to_string(),
                    r => format!("{}", r),
                }
            );

            let mut parts_good: Vec<(Vec<usize>, Partition)> = Vec::new();
            let mut unrealized_endpoints =
                (0..staged.num_endpoint_groups()).collect::<Vec<_>>();
            let mut division = 600;

            while !unrealized_endpoints.is_empty() {
                division = (division / 2).max(1);
                let num_parts = (unrealized_endpoints.len() + division - 1) / division;
                clilog::info!(
                    "current: {} endpoints, try {} parts",
                    unrealized_endpoints.len(),
                    num_parts
                );
                let staged_ur = staged.to_endpoint_subset(&unrealized_endpoints);
                let hg_ur = RCHyperGraph::from_staged_aig(&aig, &staged_ur);
                let mut parts_indices = run_par(&hg_ur, num_parts);
                for idcs in &mut parts_indices {
                    for i in idcs {
                        *i = unrealized_endpoints[*i];
                    }
                }
                let parts_try = parts_indices
                    .par_iter()
                    .map(|endpts| Partition::build_one(&aig, staged, endpts))
                    .collect::<Vec<_>>();
                let mut new_unrealized_endpoints = Vec::new();
                for (idx, part_opt) in parts_indices.into_iter().zip(parts_try.into_iter()) {
                    match part_opt {
                        Some(part) => {
                            parts_good.push((idx, part));
                        }
                        None => {
                            if idx.len() == 1 {
                                panic!("A single endpoint still cannot map, you need to increase level cut granularity.");
                            }
                            for endpt_i in idx {
                                new_unrealized_endpoints.push(endpt_i);
                            }
                        }
                    }
                }
                new_unrealized_endpoints.sort_unstable();
                unrealized_endpoints = new_unrealized_endpoints;
            }

            clilog::info!(
                "interactive partition completed: {} in total. merging started.",
                parts_good.len()
            );

            let (parts_indices_good, prebuilt): (Vec<_>, Vec<_>) =
                parts_good.into_iter().unzip();
            let effective_parts = process_partitions(
                &aig,
                staged,
                parts_indices_good,
                Some(prebuilt),
                max_stage_degrad,
            )
            .unwrap();
            clilog::info!("after merging: {} parts.", effective_parts.len());
            effective_parts
        })
        .collect::<Vec<_>>();

    let partition_file = PartitionFile {
        level_split: level_split.clone(),
        xprop_analyzed: xprop,
        partitions: stages_effective_parts,
    };

    let f = std::fs::File::create(&parts_out).unwrap();
    let mut buf = std::io::BufWriter::new(f);
    serde_bare::to_writer(&mut buf, &partition_file).unwrap();
    clilog::info!("Wrote partition file with {} stages", partition_file.partitions.len());
}

#[allow(unused_variables)]
fn cmd_sim(args: SimArgs, config: &Option<JacquardConfig>) {
    use jacquard::sim::setup;
    use jacquard::sim::vcd_io;

    // Merge CLI args with config file (CLI wins)
    let netlist_verilog = args.netlist_verilog
        .or_else(|| config.as_ref().and_then(|c| c.design.netlist.clone()))
        .unwrap_or_else(|| {
            eprintln!("Error: netlist_verilog is required (positional arg or design.netlist in jacquard.toml)");
            std::process::exit(1);
        });
    let gemparts = args.gemparts
        .or_else(|| config.as_ref().and_then(|c| c.effective_gemparts().cloned()))
        .unwrap_or_else(|| {
            eprintln!("Error: gemparts is required (positional arg or sim.gemparts/map.output in jacquard.toml)");
            std::process::exit(1);
        });
    let input_vcd = args.input_vcd
        .or_else(|| config.as_ref().and_then(|c| c.sim.input_vcd.clone()))
        .unwrap_or_else(|| {
            eprintln!("Error: input_vcd is required (positional arg or sim.input_vcd in jacquard.toml)");
            std::process::exit(1);
        });
    let output_vcd = args.output_vcd
        .or_else(|| config.as_ref().and_then(|c| c.sim.output_vcd.clone()))
        .unwrap_or_else(|| {
            eprintln!("Error: output_vcd is required (positional arg or sim.output_vcd in jacquard.toml)");
            std::process::exit(1);
        });
    let top_module = args.top_module
        .or_else(|| config.as_ref().and_then(|c| c.design.top_module.clone()));
    let level_split = if args.level_split.is_empty() {
        config.as_ref().map(|c| c.map.level_split.clone()).unwrap_or_default()
    } else {
        args.level_split
    };
    let input_vcd_scope = args.input_vcd_scope
        .or_else(|| config.as_ref().and_then(|c| c.sim.input_vcd_scope.clone()));
    let output_vcd_scope = args.output_vcd_scope
        .or_else(|| config.as_ref().and_then(|c| c.sim.output_vcd_scope.clone()));
    let check_with_cpu = args.check_with_cpu
        || config.as_ref().and_then(|c| c.sim.check_with_cpu).unwrap_or(false);
    let max_cycles = args.max_cycles
        .or_else(|| config.as_ref().and_then(|c| c.sim.max_cycles));
    let json_path = args.json_path
        .or_else(|| config.as_ref().and_then(|c| c.sim.json_path.clone()));
    let sdf = args.sdf
        .or_else(|| config.as_ref().and_then(|c| c.sim.sdf.as_ref().and_then(|s| s.file.clone())));
    let sdf_corner = if sdf.is_some() {
        args.sdf_corner
    } else {
        config.as_ref()
            .and_then(|c| c.sim.sdf.as_ref().and_then(|s| s.corner.clone()))
            .unwrap_or_else(|| "typ".to_string())
    };
    let sdf_debug = args.sdf_debug
        || config.as_ref().and_then(|c| c.sim.sdf.as_ref().and_then(|s| s.debug)).unwrap_or(false);
    let xprop = args.xprop
        || config.as_ref().and_then(|c| c.sim.xprop).unwrap_or(false);
    let enable_timing = args.enable_timing
        || config.as_ref().and_then(|c| c.sim.timing.as_ref().and_then(|t| t.enabled)).unwrap_or(false);
    let timing_clock_period = if args.timing_clock_period != 1000 {
        args.timing_clock_period
    } else {
        config.as_ref()
            .and_then(|c| c.sim.timing.as_ref().and_then(|t| t.clock_period))
            .unwrap_or(1000)
    };
    let timing_report_violations = args.timing_report_violations
        || config.as_ref().and_then(|c| c.sim.timing.as_ref().and_then(|t| t.report_violations)).unwrap_or(false);
    let liberty = args.liberty
        .or_else(|| config.as_ref().and_then(|c| c.design.liberty.clone()));

    // Auto-detect or default num_blocks if not specified
    let num_blocks_cli = args.num_blocks
        .or_else(|| config.as_ref().and_then(|c| c.sim.num_blocks));
    let num_blocks = num_blocks_cli.unwrap_or_else(|| {
        #[cfg(feature = "metal")]
        {
            clilog::info!("num_blocks not specified; using 1 for Metal GPU");
            1
        }
        #[cfg(all(not(feature = "metal"), any(feature = "cuda", feature = "hip")))]
        {
            clilog::info!("num_blocks not specified; defaulting to 64 (suitable for most GPUs)");
            clilog::info!("For optimal performance, set --num-blocks to 2x the number of SMs (CUDA) or CUs (HIP)");
            64
        }
        #[cfg(not(any(feature = "metal", feature = "cuda", feature = "hip")))]
        {
            eprintln!("Build with --features metal, --features cuda, or --features hip to enable GPU simulation");
            std::process::exit(1);
        }
    });

    let design_args = DesignArgs {
        netlist_verilog: netlist_verilog.clone(),
        top_module: top_module.clone(),
        level_split: level_split.clone(),
        gemparts: gemparts.clone(),
        num_blocks,
        json_path: json_path.clone(),
        sdf: sdf.clone(),
        sdf_corner: sdf_corner.clone(),
        sdf_debug,
        clock_period_ps: None,
        xprop,
    };

    #[allow(unused_mut)]
    let mut design = setup::load_design(&design_args);
    let timing_constraints = setup::build_timing_constraints(&design.script);

    // Parse input VCD
    let input_vcd_file = std::fs::File::open(&input_vcd).expect("Failed to open input VCD");
    let mut bufrd = std::io::BufReader::with_capacity(65536, input_vcd_file);
    let mut vcd_parser = vcd_ng::Parser::new(&mut bufrd);
    let header = vcd_parser.parse_header().unwrap();
    drop(vcd_parser);
    use std::io::{Seek, SeekFrom};
    let mut vcd_file = bufrd.into_inner();
    vcd_file.seek(SeekFrom::Start(0)).unwrap();
    let mut vcdflow = vcd_ng::FastFlow::new(vcd_file, 65536);

    // Resolve VCD scope
    let top_scope = vcd_io::resolve_vcd_scope(
        &header.items[..],
        input_vcd_scope.as_deref(),
        &design.netlistdb,
        top_module.as_deref(),
    );

    // Match VCD inputs to netlist ports
    let (vcd2inp, _) = vcd_io::match_vcd_inputs(top_scope, &design.netlistdb);

    // Parse input VCD into state vectors
    let parsed = vcd_io::parse_input_vcd(
        &mut vcdflow,
        &vcd2inp,
        &design.aig,
        &design.script,
        &design.netlistdb,
        max_cycles,
    );

    // Set up output VCD writer
    let write_buf = std::fs::File::create(&output_vcd).expect("Failed to create output VCD");
    let write_buf = std::io::BufWriter::new(write_buf);
    let mut writer = vcd_ng::Writer::new(write_buf);
    let output_mapping = vcd_io::setup_output_vcd(
        &mut writer,
        &header,
        output_vcd_scope.as_deref(),
        &design.netlistdb,
        &design.aig,
        &design.script,
    );

    // GPU dispatch
    let input_states = parsed.input_states;
    let offsets_timestamps = parsed.offsets_timestamps;
    let num_cycles = offsets_timestamps.len();

    #[cfg(any(feature = "metal", feature = "cuda", feature = "hip"))]
    let (gpu_states, sim_elapsed) = {
        #[cfg(feature = "metal")]
        {
            let sim_start = std::time::Instant::now();
            let states = sim_metal(
                &design,
                &input_states,
                &offsets_timestamps,
                &timing_constraints,
            );
            (states, sim_start.elapsed())
        }

        #[cfg(all(feature = "cuda", not(feature = "metal")))]
        {
            let sim_start = std::time::Instant::now();
            let states = sim_cuda(
                &design,
                &input_states,
                &offsets_timestamps,
                &timing_constraints,
            );
            (states, sim_start.elapsed())
        }

        #[cfg(all(feature = "hip", not(feature = "metal"), not(feature = "cuda")))]
        {
            let sim_start = std::time::Instant::now();
            let states = sim_hip(
                &design,
                &input_states,
                &offsets_timestamps,
                &timing_constraints,
            );
            (states, sim_start.elapsed())
        }
    };

    #[cfg(not(any(feature = "metal", feature = "cuda", feature = "hip")))]
    {
        eprintln!(
            "jacquard sim requires GPU support. Build with:\n\
             \n  cargo build -r --features metal --bin jacquard   (macOS)\n\
             \n  cargo build -r --features cuda --bin jacquard    (Linux/NVIDIA)\n\
             \n  cargo build -r --features hip --bin jacquard     (Linux/AMD)\n"
        );
        std::process::exit(1);
    }

    // CPU sanity check
    #[cfg(any(feature = "metal", feature = "cuda", feature = "hip"))]
    if check_with_cpu {
        if design.script.xprop_enabled {
            let rio = design.script.reg_io_state_size as usize;
            let (gpu_values, gpu_xmasks) = vcd_io::split_xprop_states(&gpu_states[..], rio);
            // Build input X-masks: same initial template as expand_states_for_xprop
            let num_input_snaps = input_states.len() / rio;
            let mut xmask_template = vec![0xFFFF_FFFFu32; rio];
            for &pos in design.script.input_map.values() {
                xmask_template[(pos >> 5) as usize] &= !(1u32 << (pos & 31));
            }
            let mut input_xmasks = Vec::with_capacity(num_input_snaps * rio);
            for _ in 0..num_input_snaps {
                input_xmasks.extend_from_slice(&xmask_template);
            }
            jacquard::sim::cpu_reference::sanity_check_cpu_xprop(
                &design.script,
                &input_states,
                &gpu_values,
                &input_xmasks,
                &gpu_xmasks,
                num_cycles,
            );
        } else {
            jacquard::sim::cpu_reference::sanity_check_cpu(
                &design.script,
                &input_states,
                &gpu_states[..],
                num_cycles,
            );
        }
    }

    // Post-simulation timing analysis
    #[cfg(any(feature = "metal", feature = "cuda", feature = "hip"))]
    if enable_timing {
        run_timing_analysis(
            &mut design.aig,
            liberty.as_ref(),
            timing_clock_period,
            timing_report_violations,
        );
    }

    // Write output VCD
    #[cfg(any(feature = "metal", feature = "cuda", feature = "hip"))]
    if design.script.xprop_enabled {
        let rio = design.script.reg_io_state_size as usize;
        let (values, xmasks) = vcd_io::split_xprop_states(&gpu_states[..], rio);
        vcd_io::write_output_vcd_xprop(
            &mut writer,
            &output_mapping,
            &offsets_timestamps,
            &values,
            &xmasks,
        );

        // X-propagation report: count X bits at primary outputs
        let eff = design.script.effective_state_size() as usize;
        let num_snapshots = gpu_states.len() / eff;
        let mut first_x_free_cycle: Option<usize> = None;
        for snap_i in 1..num_snapshots {
            let xmask_base = snap_i * eff + rio;
            let mut has_x = false;
            for &(_aigpin, pos, _vid) in &output_mapping.out2vcd {
                if pos == u32::MAX {
                    continue;
                }
                let x = gpu_states[xmask_base + (pos >> 5) as usize] >> (pos & 31) & 1;
                if x != 0 {
                    has_x = true;
                    break;
                }
            }
            if !has_x && first_x_free_cycle.is_none() {
                first_x_free_cycle = Some(snap_i - 1);
            }
        }
        if let Some(cycle) = first_x_free_cycle {
            clilog::info!("All primary outputs X-free at cycle {}", cycle);
        } else if num_snapshots > 1 {
            clilog::warn!(
                "Primary outputs still have X values at final cycle {}",
                num_snapshots - 2
            );
        }
    } else {
        vcd_io::write_output_vcd(
            &mut writer,
            &output_mapping,
            &offsets_timestamps,
            &gpu_states[..],
        );
    }

    // Print simulation summary stats
    #[cfg(any(feature = "metal", feature = "cuda", feature = "hip"))]
    {
        let elapsed_secs = sim_elapsed.as_secs_f64();
        let cycles_per_sec = if elapsed_secs > 0.0 {
            num_cycles as f64 / elapsed_secs
        } else {
            0.0
        };
        println!("\n=== Simulation Summary ===");
        println!("Total cycles simulated: {}", num_cycles);
        println!("Wall-clock time: {:.3}s", elapsed_secs);
        println!("Throughput: {:.2e} cycles/sec", cycles_per_sec);
        println!("Output VCD: {}", output_vcd.display());
    }
}

#[cfg(feature = "metal")]
fn sim_metal(
    design: &jacquard::sim::setup::LoadedDesign,
    input_states: &[u32],
    offsets_timestamps: &[(usize, u64)],
    timing_constraints: &Option<Vec<u32>>,
) -> Vec<u32> {
    use jacquard::aig::SimControlType;
    use jacquard::display::format_display_message;
    use jacquard::event_buffer::{
        process_events, AssertConfig, EventBuffer, EventType, SimControl, SimStats, MAX_EVENTS,
    };
    use metal::{Device as MTLDevice, MTLResourceOptions, MTLSize};
    use ulib::{AsUPtr, AsUPtrMut, Device, UVec};

    let script = &design.script;

    // Initialize Metal
    let mtl_device = MTLDevice::system_default().expect("No Metal device found");
    clilog::info!("Using Metal device: {}", mtl_device.name());

    let metallib_path = env!("METALLIB_PATH");
    let library = mtl_device
        .new_library_with_file(metallib_path)
        .expect("Failed to load metallib");
    let kernel_function = library
        .get_function("simulate_v1_stage", None)
        .expect("Failed to get kernel function");
    let pipeline_state = mtl_device
        .new_compute_pipeline_state_with_function(&kernel_function)
        .expect("Failed to create pipeline state");
    let command_queue = mtl_device.new_command_queue();

    let device = Device::Metal(0);
    let num_cycles = offsets_timestamps.len();

    // When xprop is enabled, expand the value-only state buffer to include X-mask
    let expanded_states = if script.xprop_enabled {
        jacquard::sim::vcd_io::expand_states_for_xprop(input_states, script)
    } else {
        input_states.to_vec()
    };
    let mut input_states_uvec: UVec<_> = expanded_states.into();
    input_states_uvec.as_mut_uptr(device);
    let mut sram_storage: UVec<u32> = UVec::new_zeroed(script.sram_storage_size as usize, device);
    // SRAM X-mask shadow: all 0xFFFFFFFF (unknown) initially when xprop enabled
    let sram_xmask_size = if script.xprop_enabled {
        script.sram_storage_size as usize
    } else {
        1 // Metal requires non-zero buffer; kernel checks is_x_capable before reading
    };
    let mut sram_xmask: UVec<u32> = if script.xprop_enabled {
        // Build on CPU then transfer to GPU — UVec::new_filled() doesn't support Metal/CUDA devices
        let v: UVec<u32> = vec![0xFFFF_FFFFu32; sram_xmask_size].into();
        v
    } else {
        UVec::new_zeroed(sram_xmask_size, device)
    };

    // Get Metal buffer pointers
    let blocks_start_ptr = script.blocks_start.as_uptr(device);
    let blocks_data_ptr = script.blocks_data.as_uptr(device);
    let sram_data_ptr = sram_storage.as_mut_uptr(device);
    let sram_xmask_ptr = sram_xmask.as_mut_uptr(device);
    let states_ptr = input_states_uvec.as_mut_uptr(device);

    let blocks_start_buffer = mtl_device.new_buffer_with_bytes_no_copy(
        blocks_start_ptr as *const _,
        (script.blocks_start.len() * std::mem::size_of::<usize>()) as u64,
        MTLResourceOptions::StorageModeShared,
        None,
    );
    let blocks_data_buffer = mtl_device.new_buffer_with_bytes_no_copy(
        blocks_data_ptr as *const _,
        (script.blocks_data.len() * std::mem::size_of::<u32>()) as u64,
        MTLResourceOptions::StorageModeShared,
        None,
    );
    let sram_data_buffer = mtl_device.new_buffer_with_bytes_no_copy(
        sram_data_ptr as *mut _ as *const _,
        (sram_storage.len() * std::mem::size_of::<u32>()) as u64,
        MTLResourceOptions::StorageModeShared,
        None,
    );
    let states_buffer = mtl_device.new_buffer_with_bytes_no_copy(
        states_ptr as *mut _ as *const _,
        (input_states_uvec.len() * std::mem::size_of::<u32>()) as u64,
        MTLResourceOptions::StorageModeShared,
        None,
    );
    let sram_xmask_buffer = mtl_device.new_buffer_with_bytes_no_copy(
        sram_xmask_ptr as *mut _ as *const _,
        (sram_xmask.len() * std::mem::size_of::<u32>()) as u64,
        MTLResourceOptions::StorageModeShared,
        None,
    );

    // Event buffer
    let event_buffer = Box::new(EventBuffer::new());
    let event_buffer_ptr = Box::into_raw(event_buffer);
    let event_buffer_metal = mtl_device.new_buffer_with_bytes_no_copy(
        event_buffer_ptr as *const _,
        std::mem::size_of::<EventBuffer>() as u64,
        MTLResourceOptions::StorageModeShared,
        None,
    );

    // Timing constraint buffer (THE GAP FIX)
    let timing_buffer = timing_constraints.as_ref().map(|buf| {
        mtl_device.new_buffer_with_data(
            buf.as_ptr() as *const _,
            (buf.len() * std::mem::size_of::<u32>()) as u64,
            MTLResourceOptions::StorageModeShared,
        )
    });

    #[repr(C)]
    struct SimParams {
        num_blocks: u64,
        num_major_stages: u64,
        num_cycles: u64,
        state_size: u64,
        current_cycle: u64,
        current_stage: u64,
    }

    let assert_config = AssertConfig::default();
    let mut sim_stats = SimStats::default();
    let mut cycles_completed = 0;
    let mut final_control = SimControl::Continue;

    let timer_sim = clilog::stimer!("simulation");

    for cycle_i in 0..num_cycles {
        unsafe {
            (*event_buffer_ptr).reset();
        }

        for stage_i in 0..script.num_major_stages {
            let params = SimParams {
                num_blocks: script.num_blocks as u64,
                num_major_stages: script.num_major_stages as u64,
                num_cycles: num_cycles as u64,
                state_size: script.effective_state_size() as u64,
                current_cycle: cycle_i as u64,
                current_stage: stage_i as u64,
            };

            let params_buffer = mtl_device.new_buffer_with_data(
                &params as *const SimParams as *const _,
                std::mem::size_of::<SimParams>() as u64,
                MTLResourceOptions::StorageModeShared,
            );

            let command_buffer = command_queue.new_command_buffer();
            let encoder = command_buffer.new_compute_command_encoder();

            encoder.set_compute_pipeline_state(&pipeline_state);
            encoder.set_buffer(0, Some(&blocks_start_buffer), 0);
            encoder.set_buffer(1, Some(&blocks_data_buffer), 0);
            encoder.set_buffer(2, Some(&sram_data_buffer), 0);
            encoder.set_buffer(3, Some(&states_buffer), 0);
            encoder.set_buffer(4, Some(&params_buffer), 0);
            encoder.set_buffer(5, Some(&event_buffer_metal), 0);
            // Buffer slot 6: timing constraints (THE GAP FIX)
            encoder.set_buffer(6, timing_buffer.as_ref().map(|v| &**v), 0);
            // Buffer slot 7: SRAM X-mask shadow
            encoder.set_buffer(7, Some(&sram_xmask_buffer), 0);

            let threads_per_threadgroup = MTLSize::new(256, 1, 1);
            let threadgroups = MTLSize::new(script.num_blocks as u64, 1, 1);

            encoder.dispatch_thread_groups(threadgroups, threads_per_threadgroup);
            encoder.end_encoding();

            command_buffer.commit();
            command_buffer.wait_until_completed();
        }

        // Check assertions
        if !script.assertion_positions.is_empty() {
            let states_slice =
                unsafe { std::slice::from_raw_parts(states_ptr, input_states_uvec.len()) };
            let cycle_output_offset = (cycle_i + 1) * script.effective_state_size() as usize;

            for &(cell_id, pos, message_id, control_type) in &script.assertion_positions {
                let word_idx = (pos >> 5) as usize;
                let bit_idx = pos & 31;
                let abs_word_idx = cycle_output_offset + word_idx;
                if abs_word_idx < states_slice.len() {
                    let condition = (states_slice[abs_word_idx] >> bit_idx) & 1;
                    if condition == 1 {
                        let event_type = match control_type {
                            None => EventType::AssertFail,
                            Some(SimControlType::Stop) => EventType::Stop,
                            Some(SimControlType::Finish) => EventType::Finish,
                        };

                        unsafe {
                            let count = (*event_buffer_ptr)
                                .count
                                .fetch_add(1, std::sync::atomic::Ordering::AcqRel)
                                as usize;
                            if count < MAX_EVENTS {
                                let event = &mut (*event_buffer_ptr).events[count];
                                event.event_type = event_type as u32;
                                event.message_id = message_id;
                                event.cycle = cycle_i as u32;
                            }
                        }

                        clilog::debug!(
                            "[cycle {}] Assertion condition fired: cell={}, pos={}, type={:?}",
                            cycle_i,
                            cell_id,
                            pos,
                            control_type
                        );
                    }
                }
            }
        }

        // Check display conditions
        if !script.display_positions.is_empty() {
            let states_slice =
                unsafe { std::slice::from_raw_parts(states_ptr, input_states_uvec.len()) };
            let cycle_output_offset = (cycle_i + 1) * script.effective_state_size() as usize;

            for (cell_id, enable_pos, format, arg_positions, arg_widths) in
                &script.display_positions
            {
                let word_idx = (*enable_pos >> 5) as usize;
                let bit_idx = *enable_pos & 31;
                let abs_word_idx = cycle_output_offset + word_idx;
                if abs_word_idx < states_slice.len() {
                    let enable = (states_slice[abs_word_idx] >> bit_idx) & 1;
                    if enable == 1 {
                        let mut display_args: Vec<u64> = Vec::new();
                        for &arg_pos in arg_positions {
                            let arg_word_idx = (arg_pos >> 5) as usize;
                            let arg_bit_idx = arg_pos & 31;
                            let abs_arg_idx = cycle_output_offset + arg_word_idx;
                            if abs_arg_idx < states_slice.len() {
                                let val = ((states_slice[abs_arg_idx] >> arg_bit_idx) & 1) as u64;
                                display_args.push(val);
                            }
                        }

                        let message = format_display_message(format, &display_args, arg_widths);
                        print!("{}", message);

                        unsafe {
                            let count = (*event_buffer_ptr)
                                .count
                                .fetch_add(1, std::sync::atomic::Ordering::AcqRel)
                                as usize;
                            if count < MAX_EVENTS {
                                let event = &mut (*event_buffer_ptr).events[count];
                                event.event_type = EventType::Display as u32;
                                event.message_id = *cell_id as u32;
                                event.cycle = cycle_i as u32;
                            }
                        }

                        clilog::debug!(
                            "[cycle {}] Display fired: cell={}, format='{}'",
                            cycle_i,
                            cell_id,
                            format
                        );
                    }
                }
            }
        }

        // Process events
        let control = unsafe {
            process_events(
                &*event_buffer_ptr,
                &assert_config,
                &mut sim_stats,
                |msg_id, cycle, _data| {
                    clilog::debug!("[cycle {}] Event processed: message id={}", cycle, msg_id);
                },
            )
        };

        cycles_completed = cycle_i + 1;

        match control {
            SimControl::Continue => {}
            SimControl::Pause => {
                final_control = SimControl::Pause;
            }
            SimControl::Terminate => {
                final_control = SimControl::Terminate;
                break;
            }
        }
    }

    clilog::finish!(timer_sim);

    // Report simulation result
    match final_control {
        SimControl::Continue => {
            clilog::info!("Simulation completed {} cycles", cycles_completed);
        }
        SimControl::Pause => {
            clilog::info!(
                "Simulation paused at cycle {} ($stop encountered)",
                cycles_completed
            );
        }
        SimControl::Terminate => {
            clilog::info!(
                "Simulation terminated at cycle {} ($finish encountered)",
                cycles_completed
            );
        }
    }

    if sim_stats.assertion_failures > 0 {
        clilog::warn!("Total assertion failures: {}", sim_stats.assertion_failures);
    }

    // Clean up event buffer
    unsafe {
        drop(Box::from_raw(event_buffer_ptr));
    }

    input_states_uvec[..].to_vec()
}

#[cfg(feature = "cuda")]
fn sim_cuda(
    design: &jacquard::sim::setup::LoadedDesign,
    input_states: &[u32],
    offsets_timestamps: &[(usize, u64)],
    timing_constraints: &Option<Vec<u32>>,
) -> Vec<u32> {
    use jacquard::aig::SimControlType;
    use jacquard::display::format_display_message;
    use jacquard::event_buffer::{AssertAction, AssertConfig, EventType, SimStats};
    use ulib::{AsUPtrMut, Device, UVec};

    mod ucci {
        include!(concat!(env!("OUT_DIR"), "/uccbind/kernel_v1.rs"));
    }

    let script = &design.script;
    let device = Device::CUDA(0);
    let num_cycles = offsets_timestamps.len();

    // When xprop is enabled, expand the value-only state buffer to include X-mask
    let expanded_states = if script.xprop_enabled {
        jacquard::sim::vcd_io::expand_states_for_xprop(input_states, script)
    } else {
        input_states.to_vec()
    };
    let mut input_states_uvec: UVec<_> = expanded_states.into();
    input_states_uvec.as_mut_uptr(device);
    let mut sram_storage = UVec::new_zeroed(script.sram_storage_size as usize, device);
    // SRAM X-mask shadow: all 0xFFFFFFFF (unknown) initially when xprop enabled
    let sram_xmask_size = if script.xprop_enabled {
        script.sram_storage_size as usize
    } else {
        1 // Kernel checks is_x_capable before reading
    };
    let mut sram_xmask: UVec<u32> = if script.xprop_enabled {
        // Build on CPU then transfer to GPU — UVec::new_filled() doesn't support Metal/CUDA devices
        let v: UVec<u32> = vec![0xFFFF_FFFFu32; sram_xmask_size].into();
        v
    } else {
        UVec::new_zeroed(sram_xmask_size, device)
    };

    // Launch GPU simulation
    device.synchronize();
    let timer_sim = clilog::stimer!("simulation");

    if timing_constraints.is_some() {
        // TODO: Wire timing constraints to CUDA timed kernel variant.
        // The EventBuffer struct contains AtomicU32, which doesn't impl Copy,
        // so UVec<EventBuffer> can't satisfy the UniversalCopy trait bound.
        // For now, the simple_scan variant passes nullptr for both timing_constraints
        // and event_buffer on the C side.
        clilog::warn!(
            "Timing constraints requested but CUDA timed kernel not yet wired; \
             running without GPU-side timing checks"
        );
    }
    ucci::simulate_v1_noninteractive_simple_scan(
        script.num_blocks,
        script.num_major_stages,
        &script.blocks_start,
        &script.blocks_data,
        &mut sram_storage,
        &mut sram_xmask,
        num_cycles,
        script.effective_state_size() as usize,
        &mut input_states_uvec,
        device,
    );

    device.synchronize();
    clilog::finish!(timer_sim);

    // Process display outputs (post-sim scan)
    if !script.display_positions.is_empty() {
        clilog::info!(
            "Processing {} display nodes",
            script.display_positions.len()
        );

        let eff_size = script.effective_state_size() as usize;
        let states_slice = &input_states_uvec[eff_size..];
        for cycle_i in 0..num_cycles {
            let cycle_offset = cycle_i * eff_size;
            for (_cell_id, enable_pos, format, arg_positions, arg_widths) in
                &script.display_positions
            {
                let word_idx = (*enable_pos >> 5) as usize;
                let bit_idx = *enable_pos & 31;
                let abs_word_idx = cycle_offset + word_idx;
                if abs_word_idx < states_slice.len() {
                    let enable = (states_slice[abs_word_idx] >> bit_idx) & 1;
                    if enable == 1 {
                        let mut args: Vec<u64> = Vec::new();
                        for &arg_pos in arg_positions {
                            let arg_word_idx = (arg_pos >> 5) as usize;
                            let arg_bit_idx = arg_pos & 31;
                            let abs_arg_idx = cycle_offset + arg_word_idx;
                            if abs_arg_idx < states_slice.len() {
                                let val = ((states_slice[abs_arg_idx] >> arg_bit_idx) & 1) as u64;
                                args.push(val);
                            }
                        }
                        let message = format_display_message(format, &args, arg_widths);
                        print!("{}", message);
                    }
                }
            }
        }
    }

    // Process assertion conditions (post-sim scan)
    if !script.assertion_positions.is_empty() {
        clilog::info!(
            "Processing {} assertion nodes",
            script.assertion_positions.len()
        );

        let assert_config = AssertConfig::default();
        let mut sim_stats = SimStats::default();

        let eff_size = script.effective_state_size() as usize;
        let states_slice = &input_states_uvec[eff_size..];
        for cycle_i in 0..num_cycles {
            let cycle_offset = cycle_i * eff_size;
            for &(_cell_id, pos, _message_id, control_type) in &script.assertion_positions {
                let word_idx = (pos >> 5) as usize;
                let bit_idx = pos & 31;
                let abs_word_idx = cycle_offset + word_idx;
                if abs_word_idx < states_slice.len() {
                    let condition = (states_slice[abs_word_idx] >> bit_idx) & 1;
                    if condition == 1 {
                        let event_type = match control_type {
                            None => EventType::AssertFail,
                            Some(SimControlType::Stop) => EventType::Stop,
                            Some(SimControlType::Finish) => EventType::Finish,
                        };
                        clilog::warn!(
                            "[cycle {}] Assertion condition fired: pos={}, type={:?}",
                            cycle_i,
                            pos,
                            control_type
                        );
                        match (event_type, assert_config.on_failure) {
                            (EventType::AssertFail, AssertAction::Log) => {
                                sim_stats.assertion_failures += 1;
                            }
                            (EventType::AssertFail, AssertAction::Terminate) => {
                                clilog::error!("Assertion failed - terminating simulation");
                                sim_stats.assertion_failures += 1;
                                std::process::exit(1);
                            }
                            (EventType::Stop, _) => {
                                clilog::info!("$stop encountered at cycle {}", cycle_i);
                                sim_stats.stop_count += 1;
                                break;
                            }
                            (EventType::Finish, _) => {
                                clilog::info!("$finish encountered at cycle {}", cycle_i);
                                break;
                            }
                            _ => {}
                        }
                    }
                }
            }
        }

        if sim_stats.assertion_failures > 0 {
            clilog::warn!(
                "Simulation completed with {} assertion failures",
                sim_stats.assertion_failures
            );
        }
    }

    input_states_uvec[..].to_vec()
}

#[cfg(feature = "hip")]
fn sim_hip(
    design: &jacquard::sim::setup::LoadedDesign,
    input_states: &[u32],
    offsets_timestamps: &[(usize, u64)],
    timing_constraints: &Option<Vec<u32>>,
) -> Vec<u32> {
    use jacquard::aig::SimControlType;
    use jacquard::display::format_display_message;
    use jacquard::event_buffer::{AssertAction, AssertConfig, EventType, SimStats};
    use ulib::{AsUPtrMut, Device, UVec};

    mod ucci_hip {
        include!(concat!(env!("OUT_DIR"), "/uccbind/kernel_v1_hip.rs"));
    }

    let script = &design.script;
    let device = Device::HIP(0);
    let num_cycles = offsets_timestamps.len();

    // When xprop is enabled, expand the value-only state buffer to include X-mask
    let expanded_states = if script.xprop_enabled {
        jacquard::sim::vcd_io::expand_states_for_xprop(input_states, script)
    } else {
        input_states.to_vec()
    };
    let mut input_states_uvec: UVec<_> = expanded_states.into();
    input_states_uvec.as_mut_uptr(device);
    let mut sram_storage = UVec::new_zeroed(script.sram_storage_size as usize, device);
    // SRAM X-mask shadow: all 0xFFFFFFFF (unknown) initially when xprop enabled
    let sram_xmask_size = if script.xprop_enabled {
        script.sram_storage_size as usize
    } else {
        1 // Kernel checks is_x_capable before reading
    };
    let mut sram_xmask: UVec<u32> = if script.xprop_enabled {
        let v: UVec<u32> = vec![0xFFFF_FFFFu32; sram_xmask_size].into();
        v
    } else {
        UVec::new_zeroed(sram_xmask_size, device)
    };

    // Launch GPU simulation
    device.synchronize();
    let timer_sim = clilog::stimer!("simulation");

    if timing_constraints.is_some() {
        clilog::warn!(
            "Timing constraints requested but HIP timed kernel not yet wired; \
             running without GPU-side timing checks"
        );
    }
    ucci_hip::simulate_v1_noninteractive_simple_scan(
        script.num_blocks,
        script.num_major_stages,
        &script.blocks_start,
        &script.blocks_data,
        &mut sram_storage,
        &mut sram_xmask,
        num_cycles,
        script.effective_state_size() as usize,
        &mut input_states_uvec,
        device,
    );

    device.synchronize();
    clilog::finish!(timer_sim);

    // Process display outputs (post-sim scan)
    if !script.display_positions.is_empty() {
        clilog::info!(
            "Processing {} display nodes",
            script.display_positions.len()
        );

        let eff_size = script.effective_state_size() as usize;
        let states_slice = &input_states_uvec[eff_size..];
        for cycle_i in 0..num_cycles {
            let cycle_offset = cycle_i * eff_size;
            for (_cell_id, enable_pos, format, arg_positions, arg_widths) in
                &script.display_positions
            {
                let word_idx = (*enable_pos >> 5) as usize;
                let bit_idx = *enable_pos & 31;
                let abs_word_idx = cycle_offset + word_idx;
                if abs_word_idx < states_slice.len() {
                    let enable = (states_slice[abs_word_idx] >> bit_idx) & 1;
                    if enable == 1 {
                        let mut args: Vec<u64> = Vec::new();
                        for &arg_pos in arg_positions {
                            let arg_word_idx = (arg_pos >> 5) as usize;
                            let arg_bit_idx = arg_pos & 31;
                            let abs_arg_idx = cycle_offset + arg_word_idx;
                            if abs_arg_idx < states_slice.len() {
                                let val = ((states_slice[abs_arg_idx] >> arg_bit_idx) & 1) as u64;
                                args.push(val);
                            }
                        }
                        let message = format_display_message(format, &args, arg_widths);
                        print!("{}", message);
                    }
                }
            }
        }
    }

    // Process assertion conditions (post-sim scan)
    if !script.assertion_positions.is_empty() {
        clilog::info!(
            "Processing {} assertion nodes",
            script.assertion_positions.len()
        );

        let assert_config = AssertConfig::default();
        let mut sim_stats = SimStats::default();

        let eff_size = script.effective_state_size() as usize;
        let states_slice = &input_states_uvec[eff_size..];
        for cycle_i in 0..num_cycles {
            let cycle_offset = cycle_i * eff_size;
            for &(_cell_id, pos, _message_id, control_type) in &script.assertion_positions {
                let word_idx = (pos >> 5) as usize;
                let bit_idx = pos & 31;
                let abs_word_idx = cycle_offset + word_idx;
                if abs_word_idx < states_slice.len() {
                    let condition = (states_slice[abs_word_idx] >> bit_idx) & 1;
                    if condition == 1 {
                        let event_type = match control_type {
                            None => EventType::AssertFail,
                            Some(SimControlType::Stop) => EventType::Stop,
                            Some(SimControlType::Finish) => EventType::Finish,
                        };
                        clilog::warn!(
                            "[cycle {}] Assertion condition fired: pos={}, type={:?}",
                            cycle_i,
                            pos,
                            control_type
                        );
                        match (event_type, assert_config.on_failure) {
                            (EventType::AssertFail, AssertAction::Log) => {
                                sim_stats.assertion_failures += 1;
                            }
                            (EventType::AssertFail, AssertAction::Terminate) => {
                                clilog::error!("Assertion failed - terminating simulation");
                                sim_stats.assertion_failures += 1;
                                std::process::exit(1);
                            }
                            (EventType::Stop, _) => {
                                clilog::info!("$stop encountered at cycle {}", cycle_i);
                                sim_stats.stop_count += 1;
                                break;
                            }
                            (EventType::Finish, _) => {
                                clilog::info!("$finish encountered at cycle {}", cycle_i);
                                break;
                            }
                            _ => {}
                        }
                    }
                }
            }
        }

        if sim_stats.assertion_failures > 0 {
            clilog::warn!(
                "Simulation completed with {} assertion failures",
                sim_stats.assertion_failures
            );
        }
    }

    input_states_uvec[..].to_vec()
}

#[cfg(any(feature = "metal", feature = "cuda", feature = "hip"))]
fn run_timing_analysis(
    aig: &mut AIG,
    liberty_path: Option<&PathBuf>,
    timing_clock_period: u64,
    timing_report_violations: bool,
) {
    use jacquard::liberty_parser::TimingLibrary;

    clilog::info!("Running timing analysis on GPU simulation results...");
    let timer_timing = clilog::stimer!("timing_analysis");

    let lib = if let Some(lib_path) = liberty_path {
        TimingLibrary::from_file(lib_path).expect("Failed to load Liberty library")
    } else {
        TimingLibrary::load_aigpdk().expect("Failed to load default AIGPDK library")
    };
    clilog::info!("Loaded Liberty library: {}", lib.name);

    aig.load_timing_library(&lib);
    aig.clock_period_ps = timing_clock_period;

    let report = aig.compute_timing();
    println!();
    println!("{}", report);
    println!(
        "Clock period: {} ps ({:.3} ns)",
        timing_clock_period,
        timing_clock_period as f64 / 1000.0
    );
    println!();

    println!("=== Critical Paths (Top 5) ===");
    let critical_paths = aig.get_critical_paths(5);
    for (i, (endpoint, arrival)) in critical_paths.iter().enumerate() {
        let slack = timing_clock_period as i64 - *arrival as i64;
        println!(
            "#{}: endpoint aigpin {} arrival={} ps, slack={} ps",
            i + 1,
            endpoint,
            arrival,
            slack
        );
    }
    println!();

    if timing_report_violations && report.has_violations() {
        println!("=== Timing Violations ===");
        for (i, ((_cell_id, dff), (&setup_slack, &hold_slack))) in aig
            .dffs
            .iter()
            .zip(aig.setup_slacks.iter().zip(aig.hold_slacks.iter()))
            .enumerate()
        {
            if setup_slack < 0 || hold_slack < 0 {
                println!("DFF #{}: D aigpin {}", i, dff.d_iv >> 1);
                if setup_slack < 0 {
                    println!("  SETUP VIOLATION: slack = {} ps", setup_slack);
                }
                if hold_slack < 0 {
                    println!("  HOLD VIOLATION: slack = {} ps", hold_slack);
                }
            }
        }
        println!();
    }

    if report.has_violations() {
        clilog::warn!(
            "TIMING ANALYSIS: FAILED ({} setup, {} hold violations)",
            report.setup_violations,
            report.hold_violations
        );
    } else {
        clilog::info!("TIMING ANALYSIS: PASSED");
    }

    clilog::finish!(timer_timing);
}

fn init_logging(verbose: u8, quiet: u8) {
    use simplelog::*;
    let level = match (verbose, quiet) {
        (0, 0) => LevelFilter::Info,
        (1, 0) => LevelFilter::Debug,
        (v, 0) if v >= 2 => LevelFilter::Trace,
        (0, 1) => LevelFilter::Warn,
        (0, q) if q >= 2 => LevelFilter::Error,
        // Both set: verbose takes precedence
        (v, _) if v > 0 => {
            match v {
                1 => LevelFilter::Debug,
                _ => LevelFilter::Trace,
            }
        }
        (_, q) => {
            match q {
                1 => LevelFilter::Warn,
                _ => LevelFilter::Error,
            }
        }
    };

    TermLogger::init(
        level,
        ConfigBuilder::new()
            .set_location_level(LevelFilter::Debug)
            .set_thread_level(LevelFilter::Trace)
            .add_filter_ignore_str("rustyline")
            .build(),
        TerminalMode::Stderr,
        ColorChoice::Auto,
    )
    .unwrap();

    clilog::set_max_print_count(clilog::Level::Warn, "NL_SV_LIT", 1);
}

/// Load config file: from explicit --config path, or auto-discover jacquard.toml.
fn load_config(config_path: Option<&PathBuf>) -> Option<JacquardConfig> {
    if let Some(path) = config_path {
        match JacquardConfig::load(path) {
            Ok(mut config) => {
                let config_dir = path.parent().unwrap_or(std::path::Path::new("."));
                config.resolve_paths(config_dir);
                clilog::info!("Loaded config from {}", path.display());
                Some(config)
            }
            Err(e) => {
                eprintln!("Error: {}", e);
                std::process::exit(1);
            }
        }
    } else {
        JacquardConfig::discover().map(|(config, path)| {
            clilog::info!("Auto-discovered config: {}", path.display());
            config
        })
    }
}

fn main() {
    let cli = Cli::parse();
    init_logging(cli.verbose, cli.quiet);

    let config = load_config(cli.config.as_ref());

    match cli.command {
        Commands::Map(args) => cmd_map(args, &config),
        Commands::Sim(args) => cmd_sim(args, &config),
        Commands::Cosim(args) => cmd_cosim(args, &config),
    }
}

#[allow(unused_variables)]
fn cmd_cosim(args: CosimArgs, project_config: &Option<JacquardConfig>) {
    #[cfg(not(feature = "metal"))]
    {
        eprintln!(
            "jacquard cosim requires Metal support (macOS only). Build with:\n\
             \n  cargo build -r --features metal --bin jacquard\n"
        );
        std::process::exit(1);
    }

    #[cfg(feature = "metal")]
    {
        use jacquard::sim::cosim_metal::CosimOpts;
        use jacquard::sim::setup;
        use jacquard::testbench::TestbenchConfig;

        // Merge CLI args with config file (CLI wins)
        let netlist_verilog = args.netlist_verilog
            .or_else(|| project_config.as_ref().and_then(|c| c.design.netlist.clone()))
            .unwrap_or_else(|| {
                eprintln!("Error: netlist_verilog is required (positional arg or design.netlist in jacquard.toml)");
                std::process::exit(1);
            });
        let gemparts = args.gemparts
            .or_else(|| project_config.as_ref().and_then(|c| c.effective_gemparts().cloned()))
            .unwrap_or_else(|| {
                eprintln!("Error: gemparts is required (positional arg or sim.gemparts/map.output in jacquard.toml)");
                std::process::exit(1);
            });
        let cosim_config_path = args.config
            .or_else(|| project_config.as_ref().and_then(|c| c.cosim.config.clone()))
            .unwrap_or_else(|| {
                eprintln!("Error: --config is required (CLI flag or cosim.config in jacquard.toml)");
                std::process::exit(1);
            });
        let top_module = args.top_module
            .or_else(|| project_config.as_ref().and_then(|c| c.design.top_module.clone()));
        let level_split = if args.level_split.is_empty() {
            project_config.as_ref().map(|c| c.map.level_split.clone()).unwrap_or_default()
        } else {
            args.level_split
        };
        let num_blocks = if args.num_blocks != 64 {
            args.num_blocks
        } else {
            project_config.as_ref().and_then(|c| c.cosim.num_blocks).unwrap_or(64)
        };
        let max_cycles = args.max_cycles
            .or_else(|| project_config.as_ref().and_then(|c| c.cosim.max_cycles));
        let clock_period = args.clock_period
            .or_else(|| project_config.as_ref().and_then(|c| c.cosim.clock_period));

        // Load testbench config
        let file = std::fs::File::open(&cosim_config_path).expect("Failed to open config file");
        let reader = std::io::BufReader::new(file);
        let config: TestbenchConfig =
            serde_json::from_reader(reader).expect("Failed to parse config JSON");
        clilog::info!("Loaded testbench config: {:?}", config);

        // Determine clock period for SDF loading
        let clock_period_ps = clock_period
            .or(config.clock_period_ps)
            .or(config.timing.as_ref().map(|t| t.clock_period_ps));

        // Determine SDF path: CLI --sdf takes priority, then config.timing.sdf_file
        let sdf = args.sdf.clone().or_else(|| {
            config
                .timing
                .as_ref()
                .map(|t| std::path::PathBuf::from(&t.sdf_file))
        });
        let sdf_corner = if args.sdf.is_some() {
            args.sdf_corner.clone()
        } else if let Some(ref t) = config.timing {
            t.sdf_corner.clone()
        } else {
            "typ".to_string()
        };
        let sdf_debug = args.sdf_debug;

        let design_args = DesignArgs {
            netlist_verilog: netlist_verilog.clone(),
            top_module: top_module.clone(),
            level_split: level_split.clone(),
            gemparts: gemparts.clone(),
            num_blocks,
            json_path: None,
            sdf,
            sdf_corner,
            sdf_debug,
            clock_period_ps,
            xprop: false, // cosim doesn't support xprop yet
        };

        let mut design = setup::load_design(&design_args);
        let timing_constraints = setup::build_timing_constraints(&design.script);

        let opts = CosimOpts {
            max_cycles,
            num_blocks,
            flash_verbose: args.flash_verbose,
            check_with_cpu: args.check_with_cpu,
            gpu_profile: args.gpu_profile,
            clock_period,
        };

        let result =
            jacquard::sim::cosim_metal::run_cosim(&mut design, &config, &opts, &timing_constraints);
        std::process::exit(if result.passed { 0 } else { 1 });
    }
}
