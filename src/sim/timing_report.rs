// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Post-simulation timing slack report generation.
//!
//! Produces a per-DFF utilization report using dynamic arrival times from GPU
//! simulation (when `--timing-report` is given). The text summary goes to
//! stdout; a JSON file with full combinational-chain detail is written to the
//! user-specified path.

use crate::aig::{DriverType, AIG};
use crate::flatten::FlattenedScriptV1;
use crate::sim::vcd_io;
use netlistdb::NetlistDB;
use serde::Serialize;
use std::io::Write;
use std::path::Path;

// ── Data Structures ──────────────────────────────────────────────────────────

/// One gate in the combinational chain leading to a DFF D-input.
#[derive(Debug, Serialize)]
pub struct ChainGate {
    pub cell_type: String,
    pub cell_name: String,
    pub pin_name: String,
    pub delay_ps: u64,
    pub arrival_ps: u64,
}

/// Timing entry for a single DFF.
#[derive(Debug, Serialize)]
pub struct DFFTimingEntry {
    pub dff_name: String,
    pub cell_id: u32,
    /// Dynamic arrival time from the GPU (last simulated cycle), in ps.
    pub arrival_ps: u64,
    pub setup_ps: u16,
    pub hold_ps: u16,
    /// Slack = clock_period - setup - arrival. Negative means violation.
    pub slack_ps: i64,
    /// Utilization = arrival / (clock_period - setup).
    pub utilization: f64,
    /// Static combinational chain (from AIG arrival times) for JSON detail.
    pub chain: Vec<ChainGate>,
}

/// Top-level timing slack report.
#[derive(Debug, Serialize)]
pub struct TimingSlackReport {
    pub clock_period_ps: u64,
    pub threshold: f64,
    pub total_dffs: usize,
    pub reported_dffs: usize,
    pub setup_violations: usize,
    pub entries: Vec<DFFTimingEntry>,
}

// ── Report Generation ────────────────────────────────────────────────────────

/// Generate a timing slack report from GPU simulation results.
///
/// `gpu_states` is the raw GPU output buffer (including arrival data when
/// `timing_arrivals_enabled`). The AIG must have `arrival_times` populated
/// (via `compute_timing()`) for chain tracing.
pub fn generate_timing_report(
    script: &FlattenedScriptV1,
    gpu_states: &[u32],
    aig: &AIG,
    netlistdb: &NetlistDB,
    clock_period_ps: u64,
    threshold: f64,
) -> TimingSlackReport {
    assert!(
        script.timing_arrivals_enabled,
        "timing report requires timing arrivals to be enabled"
    );

    // Extract per-word arrival data from GPU output buffer
    let arrivals = vcd_io::split_arrival_states(gpu_states, script);
    let rio = script.reg_io_state_size as usize;
    let num_snapshots = arrivals.len() / rio;
    assert!(num_snapshots > 0, "no simulation snapshots available");

    // Use the last cycle's arrivals as representative values
    let last_arrivals = &arrivals[(num_snapshots - 1) * rio..];

    let total_dffs = script.dff_constraints.len();
    let mut entries = Vec::new();
    let mut setup_violations = 0usize;

    // Iterate DFF constraints (same ordering as aig.dffs)
    let dff_iter: Vec<_> = aig.dffs.iter().collect();
    assert_eq!(
        dff_iter.len(),
        script.dff_constraints.len(),
        "DFF count mismatch between AIG ({}) and script ({})",
        dff_iter.len(),
        script.dff_constraints.len()
    );

    for (i, constraint) in script.dff_constraints.iter().enumerate() {
        let data_state_pos = constraint.data_state_pos;
        if data_state_pos == u32::MAX {
            // DFF D-input not in output map (shouldn't happen normally)
            continue;
        }

        // Read arrival time from GPU: stored as u32 at the D-input's
        // bit position (word granularity).
        let word_idx = (data_state_pos / 32) as usize;
        let arrival: u64 = if word_idx < last_arrivals.len() {
            last_arrivals[word_idx] as u64
        } else {
            0
        };

        let setup = constraint.setup_ps;
        let deadline = clock_period_ps.saturating_sub(setup as u64);
        let slack = deadline as i64 - arrival as i64;
        let utilization = if deadline > 0 {
            arrival as f64 / deadline as f64
        } else {
            f64::INFINITY
        };

        if slack < 0 {
            setup_violations += 1;
        }

        if utilization >= threshold {
            // Resolve DFF instance name
            let cell_id = constraint.cell_id as usize;
            let dff_name = if cell_id < netlistdb.cellnames.len() {
                netlistdb.cellnames[cell_id].to_string()
            } else {
                format!("cell_{}", cell_id)
            };

            // Trace static critical chain through AIG
            let (&_cell_id_key, dff) = &dff_iter[i];
            let chain = trace_dff_chain(aig, netlistdb, dff.d_iv);

            entries.push(DFFTimingEntry {
                dff_name,
                cell_id: constraint.cell_id,
                arrival_ps: arrival,
                setup_ps: setup,
                hold_ps: constraint.hold_ps,
                slack_ps: slack,
                utilization,
                chain,
            });
        }
    }

    // Sort by slack ascending (worst first)
    entries.sort_by(|a, b| a.slack_ps.cmp(&b.slack_ps));

    TimingSlackReport {
        clock_period_ps,
        threshold,
        total_dffs,
        reported_dffs: entries.len(),
        setup_violations,
        entries,
    }
}

// ── Chain Tracing ────────────────────────────────────────────────────────────

/// Trace the combinational chain backwards from a DFF D-input through the AIG.
///
/// Follows the pattern of `AIG::trace_critical_path`: at each AND gate, follow
/// the input with the larger static arrival time. Stops at DFF Q, InputPort,
/// SRAM, or Tie0.
fn trace_dff_chain(aig: &AIG, netlistdb: &NetlistDB, d_iv: usize) -> Vec<ChainGate> {
    let mut chain = Vec::new();
    let mut current = d_iv >> 1; // strip inversion bit

    if current == 0 || aig.arrival_times.is_empty() {
        return chain;
    }

    loop {
        if current == 0 || current > aig.num_aigpins {
            break;
        }

        let (_, arrival) = aig.arrival_times[current];
        let delay = if current < aig.gate_delays.len() {
            let (rise, fall) = aig.gate_delays[current];
            rise.max(fall)
        } else {
            0
        };

        // Resolve cell origin for this AIG pin
        let (cell_type, cell_name, pin_name) =
            resolve_cell_origin(aig, netlistdb, current);

        chain.push(ChainGate {
            cell_type,
            cell_name,
            pin_name,
            delay_ps: delay,
            arrival_ps: arrival,
        });

        match &aig.drivers[current] {
            DriverType::AndGate(a, b) => {
                let a_idx = a >> 1;
                let b_idx = b >> 1;

                // Follow the input with larger arrival time
                let a_arr = if a_idx > 0 && a_idx < aig.arrival_times.len() {
                    aig.arrival_times[a_idx].1
                } else {
                    0
                };
                let b_arr = if b_idx > 0 && b_idx < aig.arrival_times.len() {
                    aig.arrival_times[b_idx].1
                } else {
                    0
                };

                current = if a_arr >= b_arr { a_idx } else { b_idx };
            }
            // Stop at sequential elements or primary inputs
            _ => break,
        }
    }

    chain
}

/// Resolve the cell origin (type, name, pin) for an AIG pin.
fn resolve_cell_origin(
    aig: &AIG,
    netlistdb: &NetlistDB,
    aigpin: usize,
) -> (String, String, String) {
    if aigpin < aig.aigpin_cell_origins.len() {
        let origins = &aig.aigpin_cell_origins[aigpin];
        if let Some((cell_id, cell_type, pin_name)) = origins.first() {
            let cell_name = if *cell_id < netlistdb.cellnames.len() {
                netlistdb.cellnames[*cell_id].to_string()
            } else {
                format!("cell_{}", cell_id)
            };
            return (cell_type.clone(), cell_name, pin_name.clone());
        }
    }

    // Fallback: describe by driver type
    let type_name = match &aig.drivers[aigpin] {
        DriverType::AndGate(..) => "and_gate",
        DriverType::InputPort(_) => "input_port",
        DriverType::DFF(_) => "dff",
        DriverType::SRAM(_) => "sram",
        DriverType::Tie0 => "tie0",
        DriverType::InputClockFlag(..) => "clock_flag",
    };
    (type_name.to_string(), format!("aig_{}", aigpin), "Y".to_string())
}

// ── Word Arrival Spread Analysis ──────────────────────────────────────────────

/// Per-word arrival spread statistics.
/// Shows the min/max static arrival times across the DFF D-input bits
/// sharing each 32-bit state word.
pub struct WordSpreadAnalysis {
    /// (word_index, num_bits, min_arrival_ps, max_arrival_ps)
    pub words: Vec<(usize, usize, u64, u64)>,
}

/// Analyze per-word arrival time spread using static STA data.
///
/// Groups DFF D-input arrival times by their 32-bit state word index
/// (from `script.output_map`) and computes min/max within each word.
/// This measures how much information is lost by the GPU's per-word
/// arrival approximation.
///
/// Requires `aig.arrival_times` to be populated (via `compute_timing()`).
pub fn analyze_word_arrival_spread(
    script: &FlattenedScriptV1,
    aig: &AIG,
) -> WordSpreadAnalysis {
    use std::collections::BTreeMap;

    assert!(
        !aig.arrival_times.is_empty(),
        "arrival_times not computed — call compute_timing() first"
    );

    // Group DFF D-input arrivals by state word index
    let mut word_arrivals: BTreeMap<usize, Vec<u64>> = BTreeMap::new();

    for (_cell_id, dff) in &aig.dffs {
        let d_pin = dff.d_iv >> 1;
        if d_pin == 0 || d_pin >= aig.arrival_times.len() {
            continue;
        }
        if let Some(&pos) = script.output_map.get(&dff.d_iv) {
            if pos == u32::MAX {
                continue;
            }
            let word_idx = (pos / 32) as usize;
            let (_, max_arrival) = aig.arrival_times[d_pin];
            word_arrivals.entry(word_idx).or_default().push(max_arrival);
        }
    }

    let words: Vec<_> = word_arrivals
        .into_iter()
        .map(|(word_idx, arrivals)| {
            let min = *arrivals.iter().min().unwrap();
            let max = *arrivals.iter().max().unwrap();
            (word_idx, arrivals.len(), min, max)
        })
        .collect();

    WordSpreadAnalysis { words }
}

impl WordSpreadAnalysis {
    /// Compare static per-word max arrivals against GPU dynamic arrivals.
    ///
    /// For each word, the static max is the largest arrival from `compute_timing()`
    /// across all DFF D-inputs in that word. The GPU arrival is the u32 from
    /// `split_arrival_states()`. Differences reveal the effect of the boomerang's
    /// delay approximation (max-per-thread-position packing).
    pub fn compare_with_gpu(
        &self,
        script: &FlattenedScriptV1,
        gpu_states: &[u32],
        w: &mut impl Write,
    ) -> std::io::Result<()> {
        let arrivals = vcd_io::split_arrival_states(gpu_states, script);
        let rio = script.reg_io_state_size as usize;
        let num_snapshots = arrivals.len() / rio;
        if num_snapshots == 0 {
            writeln!(w, "No GPU snapshots available for comparison.")?;
            return Ok(());
        }
        let last_arrivals = &arrivals[(num_snapshots - 1) * rio..];

        writeln!(w, "\n=== Static vs GPU Arrival Comparison ===")?;
        writeln!(w, "Static: per-pin max from compute_timing(), grouped by word")?;
        writeln!(w, "GPU:    per-word u32 from boomerang arrival tracking\n")?;

        let mut diffs: Vec<(usize, u64, u64, i64)> = Vec::new();
        for &(word_idx, _bits, _min, static_max) in &self.words {
            let gpu_arrival = if word_idx < last_arrivals.len() {
                last_arrivals[word_idx] as u64
            } else {
                0
            };
            let diff = gpu_arrival as i64 - static_max as i64;
            diffs.push((word_idx, static_max, gpu_arrival, diff));
        }

        // Summary statistics
        let positive_diffs: Vec<i64> = diffs.iter().map(|d| d.3).filter(|&d| d > 0).collect();
        let negative_diffs: Vec<i64> = diffs.iter().map(|d| d.3).filter(|&d| d < 0).collect();
        let exact_matches = diffs.iter().filter(|d| d.3 == 0).count();
        let mean_abs_diff = diffs.iter().map(|d| d.3.unsigned_abs()).sum::<u64>() as f64
            / diffs.len().max(1) as f64;

        writeln!(w, "Words compared:   {}", diffs.len())?;
        writeln!(w, "Exact matches:    {} ({:.1}%)",
            exact_matches, 100.0 * exact_matches as f64 / diffs.len() as f64)?;
        writeln!(w, "GPU > static:     {} (boomerang over-estimates)",
            positive_diffs.len())?;
        writeln!(w, "GPU < static:     {} (GPU under-reports — data-dependent)",
            negative_diffs.len())?;
        writeln!(w, "Mean |diff|:      {:.0}ps", mean_abs_diff)?;
        if let Some(&max_over) = positive_diffs.iter().max() {
            writeln!(w, "Max over-est:     {}ps", max_over)?;
        }
        if let Some(&max_under) = negative_diffs.iter().min() {
            writeln!(w, "Max under-report: {}ps", max_under)?;
        }

        // Show worst 10 by absolute difference
        let mut sorted = diffs.clone();
        sorted.sort_by(|a, b| b.3.unsigned_abs().cmp(&a.3.unsigned_abs()));
        sorted.truncate(10);

        if !sorted.is_empty() {
            writeln!(w, "\nTop 10 words by |static - GPU| difference:")?;
            writeln!(w, "{:>6}  {:>10}  {:>10}  {:>10}",
                "Word", "Static(ps)", "GPU(ps)", "Diff(ps)")?;
            writeln!(w, "{}", "-".repeat(42))?;
            for (word_idx, static_max, gpu_arr, diff) in &sorted {
                writeln!(w, "{:>6}  {:>10}  {:>10}  {:>+10}",
                    word_idx, static_max, gpu_arr, diff)?;
            }
        }

        writeln!(w)?;
        Ok(())
    }

    /// Print human-readable summary with histogram of spreads.
    pub fn write_text(&self, w: &mut impl Write) -> std::io::Result<()> {
        writeln!(w, "\n=== Per-Word Arrival Spread Analysis ===")?;
        writeln!(
            w,
            "Measures how much arrival time varies within each 32-bit state word."
        )?;
        writeln!(
            w,
            "The GPU tracks one arrival per word, so large spreads indicate lost precision.\n"
        )?;

        let total_words = self.words.len();
        if total_words == 0 {
            writeln!(w, "No DFF words found.")?;
            return Ok(());
        }

        // Compute spread for each word
        let spreads: Vec<u64> = self.words.iter().map(|&(_, _, min, max)| max - min).collect();

        // Summary statistics
        let max_spread = *spreads.iter().max().unwrap();
        let mean_spread = spreads.iter().sum::<u64>() as f64 / spreads.len() as f64;
        let zero_spread = spreads.iter().filter(|&&s| s == 0).count();

        // Histogram buckets (ps)
        let buckets = [0, 10, 50, 100, 200, 500, 1000, 2000, 5000, u64::MAX];
        let bucket_labels = [
            "     0ps",
            "  1-10ps",
            " 11-50ps",
            "51-100ps",
            "101-200ps",
            "201-500ps",
            "0.5-1ns",
            "  1-2ns",
            "  2-5ns",
            "   >5ns",
        ];
        let mut bucket_counts = vec![0usize; buckets.len()];
        for &s in &spreads {
            for i in 0..buckets.len() {
                if i == buckets.len() - 1 || s < buckets[i + 1] {
                    // Special case: bucket[0] is exactly 0
                    if i == 0 && s == 0 {
                        bucket_counts[0] += 1;
                    } else if i == 0 && s > 0 && s <= buckets[1] {
                        bucket_counts[1] += 1;
                    } else if i > 0 {
                        bucket_counts[i] += 1;
                    }
                    break;
                }
            }
        }

        writeln!(w, "Words with DFF bits: {}", total_words)?;
        writeln!(w, "Zero spread (exact):  {} ({:.1}%)",
            zero_spread, 100.0 * zero_spread as f64 / total_words as f64)?;
        writeln!(w, "Mean spread:          {:.0}ps", mean_spread)?;
        writeln!(w, "Max spread:           {}ps\n", max_spread)?;

        writeln!(w, "Spread distribution:")?;
        let bar_max = 50;
        let count_max = *bucket_counts.iter().max().unwrap_or(&1).max(&1);
        for i in 0..bucket_labels.len() {
            let count = bucket_counts[i];
            let bar_len = (count * bar_max) / count_max;
            let pct = 100.0 * count as f64 / total_words as f64;
            writeln!(
                w,
                "  {} | {:<width$} {:>4} ({:>5.1}%)",
                bucket_labels[i],
                "#".repeat(bar_len),
                count,
                pct,
                width = bar_max
            )?;
        }

        // Show worst 10 words
        let mut worst: Vec<_> = self.words.iter()
            .map(|&(idx, bits, min, max)| (max - min, idx, bits, min, max))
            .collect();
        worst.sort_by(|a, b| b.0.cmp(&a.0));
        worst.truncate(10);

        if !worst.is_empty() && worst[0].0 > 0 {
            writeln!(w, "\nTop 10 words by spread:")?;
            writeln!(w, "{:>6}  {:>5}  {:>8}  {:>8}  {:>8}",
                "Word", "Bits", "Min(ps)", "Max(ps)", "Spread")?;
            writeln!(w, "{}", "-".repeat(43))?;
            for (spread, idx, bits, min, max) in &worst {
                writeln!(w, "{:>6}  {:>5}  {:>8}  {:>8}  {:>8}",
                    idx, bits, min, max, spread)?;
            }
        }

        writeln!(w)?;
        Ok(())
    }
}

// ── Output Formatting ────────────────────────────────────────────────────────

impl TimingSlackReport {
    /// Write human-readable summary table to a writer (typically stdout).
    pub fn write_text(&self, w: &mut impl Write) -> std::io::Result<()> {
        writeln!(w, "\n=== Timing Slack Report ===")?;
        writeln!(
            w,
            "Clock period: {}ps, Threshold: {:.1}%",
            self.clock_period_ps,
            self.threshold * 100.0
        )?;
        writeln!(
            w,
            "Total DFFs: {}, Reported: {}, Setup violations: {}\n",
            self.total_dffs, self.reported_dffs, self.setup_violations
        )?;

        if self.entries.is_empty() {
            writeln!(w, "No DFFs exceed the utilization threshold.")?;
            return Ok(());
        }

        writeln!(
            w,
            "{:<50} {:>8} {:>6} {:>8} {:>7}",
            "DFF Instance", "Arrival", "Setup", "Slack", "Util%"
        )?;
        writeln!(w, "{}", "-".repeat(83))?;

        for entry in &self.entries {
            // Truncate long names with ellipsis
            let name = if entry.dff_name.len() > 50 {
                format!("...{}", &entry.dff_name[entry.dff_name.len() - 47..])
            } else {
                entry.dff_name.clone()
            };

            writeln!(
                w,
                "{:<50} {:>6}ps {:>4}ps {:>6}ps {:>6.1}%",
                name,
                entry.arrival_ps,
                entry.setup_ps,
                entry.slack_ps,
                entry.utilization * 100.0
            )?;
        }

        writeln!(w)?;
        Ok(())
    }

    /// Write full JSON report (with chain detail) to a file.
    pub fn write_json(&self, path: &Path) -> std::io::Result<()> {
        let file = std::fs::File::create(path)?;
        let writer = std::io::BufWriter::new(file);
        serde_json::to_writer_pretty(writer, self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))
    }
}
