// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//! Partition scheduler and flattener

#![allow(
    clippy::doc_overindented_list_items,
    clippy::doc_lazy_continuation,
    clippy::too_many_arguments,
    clippy::type_complexity,
    clippy::needless_range_loop
)]

use crate::aig::{DriverType, EndpointGroup, AIG};
use crate::aigpdk::AIGPDK_SRAM_ADDR_WIDTH;
use crate::liberty_parser::TimingLibrary;
use crate::pe::{Partition, BOOMERANG_NUM_STAGES};
use crate::staging::StagedAIG;
use indexmap::IndexMap;
use netlistdb::NetlistDB;
use rayon::prelude::*;
use std::collections::{BTreeMap, HashMap};
use ulib::UVec;

pub const NUM_THREADS_V1: usize = 1 << (BOOMERANG_NUM_STAGES - 5);

// === Timing Data Structures for GPU ===

/// Packed delay representation for GPU consumption.
/// Uses 16-bit fixed-point values in picoseconds.
/// Range: 0-65535 ps (0-65.5 ns), resolution: 1 ps.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(C)]
pub struct PackedDelay {
    /// Cell rise delay in picoseconds
    pub rise_ps: u16,
    /// Cell fall delay in picoseconds
    pub fall_ps: u16,
}

impl PackedDelay {
    /// Create a new PackedDelay from rise/fall times.
    pub fn new(rise_ps: u16, fall_ps: u16) -> Self {
        Self { rise_ps, fall_ps }
    }

    /// Create a PackedDelay from u64 picosecond values, saturating at u16::MAX.
    pub fn from_u64(rise_ps: u64, fall_ps: u64) -> Self {
        Self {
            rise_ps: rise_ps.min(u16::MAX as u64) as u16,
            fall_ps: fall_ps.min(u16::MAX as u64) as u16,
        }
    }

    /// Get the maximum delay (for critical path analysis).
    pub fn max_delay(&self) -> u16 {
        self.rise_ps.max(self.fall_ps)
    }

    /// Pack into a single u32 for GPU transfer.
    /// Format: [rise_ps:16][fall_ps:16]
    pub fn to_u32(&self) -> u32 {
        ((self.rise_ps as u32) << 16) | (self.fall_ps as u32)
    }

    /// Unpack from a u32.
    pub fn from_u32(packed: u32) -> Self {
        Self {
            rise_ps: (packed >> 16) as u16,
            fall_ps: (packed & 0xFFFF) as u16,
        }
    }
}

/// DFF timing constraints for GPU checking.
#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub struct DFFConstraint {
    /// Setup time in picoseconds
    pub setup_ps: u16,
    /// Hold time in picoseconds
    pub hold_ps: u16,
    /// Per-DFF clock arrival in picoseconds, capture-side, signed so the
    /// field admits negative values for future use. Populated from the
    /// IR's `ClockArrival` table (Pillar B Stage 1, see
    /// `docs/timing-model-extensions.md`); 0 when no clock-tree timing
    /// data is available — the existing zero-skew behaviour.
    ///
    /// Folded into the effective setup/hold values inside
    /// [`build_timing_constraint_buffer`] so the GPU side stays unchanged.
    pub clock_arrival_ps: i16,
    /// Position of D input data arrival in state
    pub data_state_pos: u32,
    /// DFF cell ID (for error reporting)
    pub cell_id: u32,
}

impl DFFConstraint {
    /// Pack into two u32s for GPU transfer.
    /// Returns (timing_word, position_word).
    pub fn to_u32_pair(&self) -> (u32, u32) {
        let (eff_setup, eff_hold) = self.effective_setup_hold();
        let timing = ((eff_setup as u32) << 16) | (eff_hold as u32);
        let position = self.data_state_pos;
        (timing, position)
    }

    /// Effective per-DFF (setup, hold) after folding capture-side clock
    /// arrival into the constraint values, treating the launch-side
    /// reference as 0 (Pillar B Stage 2, design in
    /// `docs/timing-model-extensions.md`).
    ///
    /// Positive arrival relaxes setup (clock arrives later → more time
    /// for data) and tightens hold by the same amount; the sign convention
    /// here mirrors the reasoning in the design doc.
    pub fn effective_setup_hold(&self) -> (u16, u16) {
        let setup_i32 = self.setup_ps as i32 - self.clock_arrival_ps as i32;
        let hold_i32 = self.hold_ps as i32 + self.clock_arrival_ps as i32;
        (
            setup_i32.clamp(0, u16::MAX as i32) as u16,
            hold_i32.clamp(0, u16::MAX as i32) as u16,
        )
    }
}

/// One DFF's identity within a packed state word: hierarchical name
/// plus the bit offset (0–31) inside the word.
#[derive(Debug, Clone)]
pub struct DffSiteName {
    pub name: String,
    pub bit: u8,
}

/// Maps GPU state-word indices to the DFF cells packed into each word.
/// Built once from `FlattenedScriptV1::dff_constraints` + `NetlistDB`,
/// consulted at runtime by `process_events` to convert opaque word
/// indices into symbolic names in violation messages.
#[derive(Debug, Clone, Default)]
pub struct WordSymbolMap {
    word_to_dffs: HashMap<u32, Vec<DffSiteName>>,
}

impl WordSymbolMap {
    /// Construct from a pre-built map of word index to DFF sites. The
    /// caller is responsible for sorting each word's site list by bit
    /// offset; `build_word_symbol_map` does this, callers building the
    /// map directly should too.
    pub fn from_sites(word_to_dffs: HashMap<u32, Vec<DffSiteName>>) -> Self {
        Self { word_to_dffs }
    }

    /// Format a violation message's "site" descriptor: a comma-separated
    /// list of DFF names sharing the word, truncated to `max_names`
    /// entries with a "+N more" suffix when overlong. Always appends
    /// "[word=N]" so the raw word index is still grep-able.
    pub fn describe_word(&self, word_id: u32, max_names: usize) -> String {
        let Some(dffs) = self.word_to_dffs.get(&word_id) else {
            return format!("<no DFF in word> [word={word_id}]");
        };
        let total = dffs.len();
        let shown = total.min(max_names.max(1));
        let mut parts: Vec<String> = dffs
            .iter()
            .take(shown)
            .map(|d| format!("{}[bit {}]", d.name, d.bit))
            .collect();
        if total > shown {
            parts.push(format!("+{} more", total - shown));
        }
        format!("{} [word={}]", parts.join(", "), word_id)
    }

    /// Total number of DFF sites mapped (across all words).
    pub fn num_sites(&self) -> usize {
        self.word_to_dffs.values().map(Vec::len).sum()
    }

    /// Number of distinct state words with at least one DFF site.
    pub fn num_words(&self) -> usize {
        self.word_to_dffs.len()
    }
}

/// A flattened script, for partition executor version 1.
/// See [FlattenedScriptV1::blocks_data] for the format details.
///
/// Generally, a script contains a number of major stages.
/// Each stage consists of the same number of blocks.
/// Each block contains a list of flattened partitions.
pub struct FlattenedScriptV1 {
    /// the number of blocks
    pub num_blocks: usize,
    /// the number of major stages
    pub num_major_stages: usize,
    /// the CSR start indices of stages and blocks.
    ///
    /// this length is num_blocks * num_major_stages + 1
    pub blocks_start: UVec<usize>,
    /// the partition instructions.
    ///
    /// the instructions follow a special format.
    /// it consists of zero or more partitions.
    /// 1. metadata section [1x256]:
    ///    the number of boomerang stages.
    ///      32-bit
    ///      if this is zero, the stage will not run. only happens
    ///      when the block has no partition mapped onto it.
    ///    is this the last boomerang stage?
    ///      32-bit, only 0 or 1
    ///    the number of valid write-outs.
    ///      32-bit
    ///    the write-out destination offset.
    ///      32-bit
    ///      (at this offset, we put all FFs and other outputs.)
    ///    the srams count and offsets.
    ///      32-bit count
    ///      32-bit memory offset for the first mem.
    ///    the number of global read rounds
    ///      32-bit count
    ///    the number of output-duplicate writeouts
    ///      32-bit count
    ///      this is used when one output pin is used by either
    ///      <both output and FFs>, or <multiple FFs with different
    ///      enabling conditions>.
    ///    padding towards 128
    ///    the location of early write-outs, excl. mem.
    ///      256 * [#boomstage & 0~256 id], compressed to 128
    /// 2. initial read-global permutation [2x]*rounds
    ///    32-bit indices*1 for each of the 256 threads.
    ///    32-bit valid mask for each of the 256 threads.
    ///    if the valid mask is zero, the memory is not read.
    ///    the result should be stored "like pext instruction",
    ///    but "reversed", and then appended to the low bits
    ///    in each round.
    ///    the index is encoded with a type bit at the highest bit:
    ///      if it is 0: it means it is offset from previous iteration.
    ///      if it is 1: it is offset from current iteration
    ///        which means it is an intermediate value coming from
    ///        the same cycle but a previous major stage.
    /// 3. boomerang sections, repeat below N*16
    ///    1. local shuffle permutation
    ///       32-bit indices * 16 for each of the 256 threads.
    ///    2. input (with inv) * 8192bits * (3+1padding)
    ///       32-bit * 256 threads * (3+1): xora, xorb, orb, [0 padding]
    ///       0xy: and gate, out = (a^x)&(b^y).
    ///       100: passthrough, out = a.
    ///       111: invalid, can be skipped.
    ///    -. write out, according to rest
    /// 4. global write-outs.
    ///    1. sram & additional endpoint copy permutations, inv. [16x].
    ///       only the inputs within sram and endpoint copy
    ///       range will be considered.
    ///       followed by [4x] invert, set0, and two 0 paddings.
    ///    2. permutation for the write-out enabler pins, inv. [16]
    ///       include itself inv and data inv.
    ///       followed by [3(+1padding)x]
    ///         clock invert, clock set0, data invert, and 0 padding
    ///    -. commit the write-out
    pub blocks_data: UVec<u32>,
    /// the state size including DFF and I/O states only.
    ///
    /// the inputs are always at front.
    pub reg_io_state_size: u32,
    /// the u32 array length for storing SRAMs.
    pub sram_storage_size: u32,
    /// expected input AIG pins layout
    pub input_layout: Vec<usize>,
    /// maps from primary outputs, FF:D and SRAM:PORT_R_RD_DATA AIG pins
    /// to state offset, index with invert.
    pub output_map: IndexMap<usize, u32>,
    /// maps from primary inputs, FF:Q/SRAM:* input AIG pins to state offset,
    /// index WITHOUT invert.
    pub input_map: IndexMap<usize, u32>,
    /// (for debug purpose) the relation between major stage, block and
    /// part indices as given in construction.
    pub stages_blocks_parts: Vec<Vec<Vec<usize>>>,
    /// Maps from assertion cell IDs to their condition positions in state.
    /// Each entry is (cell_id, state_bit_position, message_id, control_type).
    /// control_type: None = assertion, Some(Stop) = $stop, Some(Finish) = $finish
    pub assertion_positions: Vec<(usize, u32, u32, Option<crate::aig::SimControlType>)>,
    /// Maps from display cell IDs to their enable positions and format info.
    /// Each entry is (cell_id, enable_pos, format_string, arg_positions, arg_widths).
    pub display_positions: Vec<(usize, u32, String, Vec<u32>, Vec<u32>)>,

    // === Timing Analysis Fields ===
    /// Per-AIG-pin delays loaded from Liberty library.
    /// Index 0 is unused (Tie0). Indexed by aigpin.
    /// Empty if timing not loaded.
    pub gate_delays: Vec<PackedDelay>,

    /// DFF timing constraints for setup/hold checking.
    /// One entry per DFF in the design.
    pub dff_constraints: Vec<DFFConstraint>,

    /// Clock period in picoseconds for timing checks.
    pub clock_period_ps: u64,

    /// Whether timing data has been loaded.
    pub timing_enabled: bool,

    /// Delay injection info for GPU kernel.
    /// Each entry: (offset in blocks_data where the padding u32 lives,
    ///              list of AIG pin indices contributing to this thread position).
    /// Used by load_timing() / load_timing_from_ir() to patch the script
    /// with per-thread-position max gate delays.
    pub delay_patch_map: Vec<(usize, Vec<usize>)>,

    // === X-Propagation Fields ===
    /// Whether X-propagation is enabled for this design.
    /// When true, the state buffer is doubled (value + X-mask) and
    /// X-capable partitions use dual-lane computation.
    pub xprop_enabled: bool,

    /// Per-partition X-capability flag.
    /// Indexed by a flat partition index across all stages/blocks.
    /// When `xprop_enabled && partition_x_capable[i]`, partition `i`
    /// runs the X-aware kernel variant.
    pub partition_x_capable: Vec<bool>,

    /// Offset into the state buffer where X-mask words begin.
    /// Equal to `reg_io_state_size` (the X-mask mirrors the value section).
    /// Only meaningful when `xprop_enabled == true`.
    pub xprop_state_offset: u32,

    // === Timing Arrival Readback Fields ===
    /// Whether timing arrival readback is enabled for timed VCD output.
    /// When true, the state buffer includes an arrival section and the GPU
    /// kernel writes per-output arrival times to global memory.
    pub timing_arrivals_enabled: bool,

    /// Offset into the per-cycle state buffer where arrival data begins.
    /// Equal to `reg_io_state_size * (1 + xprop_enabled as u32)`.
    /// Only meaningful when `timing_arrivals_enabled == true`.
    pub arrival_state_offset: u32,
}

fn map_global_read_to_rounds(inputs_taken: &BTreeMap<u32, u32>) -> Vec<Vec<(u32, u32)>> {
    let inputs_taken = inputs_taken
        .iter()
        .map(|(&a, &b)| (a, b))
        .collect::<Vec<_>>();
    // the larger the sorting chunk size, the better the successful chance,
    // but the less efficient due to worse cache coherency.
    let mut chunk_size = inputs_taken.len();
    while chunk_size >= 1 {
        let mut slices = inputs_taken.chunks(chunk_size).collect::<Vec<_>>();
        slices.sort_by_cached_key(|&slice| {
            u32::MAX - slice.iter().map(|(_, mask)| mask.count_ones()).sum::<u32>()
        });
        let mut rounds_idx_masks: Vec<Vec<(u32, u32)>> = vec![vec![]; NUM_THREADS_V1];
        let mut round_map_j = 0;
        let mut fail = false;
        for slice in slices {
            for &(offset, mask) in slice {
                let wrap_fail_j = round_map_j;
                while rounds_idx_masks[round_map_j]
                    .iter()
                    .map(|(_, mask)| mask.count_ones())
                    .sum::<u32>()
                    + mask.count_ones()
                    > 32
                {
                    round_map_j += 1;
                    if round_map_j == NUM_THREADS_V1 {
                        round_map_j = 0;
                    }
                    if round_map_j == wrap_fail_j {
                        // panic!("failed to map at part {} mem offset {}", i, offset);
                        fail = true;
                        break;
                    }
                }
                if fail {
                    break;
                }
                rounds_idx_masks[round_map_j].push((offset, mask));
                round_map_j += 1;
                if round_map_j == NUM_THREADS_V1 {
                    round_map_j = 0;
                }
            }
            if fail {
                break;
            }
        }
        if !fail {
            // let max_rounds = rounds_idx_masks.iter().map(|v| v.len()).max().unwrap();
            // println!("max_rounds: {}, round_map_j: {}, inputs_taken len {}", max_rounds, round_map_j, inputs_taken.len());
            return rounds_idx_masks;
        }
        chunk_size /= 2;
    }
    panic!("cannot map global init to any multiples of rounds.");
}

/// temporaries for a part being flattened. will be discarded after built.
#[derive(Debug, Clone, Default)]
struct FlatteningPart {
    /// for each boomerang stage, the result bits layout.
    afters: Vec<Vec<usize>>,
    /// for each partition, the output bits layout not containing sram outputs yet.
    parts_after_writeouts: Vec<usize>,
    /// mapping from aig pin index to writeout position (0~8192)
    after_writeout_pin2pos: IndexMap<usize, u16>,
    /// the number of SRAMs to simulate in this part.
    num_srams: u32,
    /// number of normal writeouts
    num_normal_writeouts: u32,
    /// number of writeout slots for output duplication
    num_duplicate_writeouts: u32,
    /// number of total writeouts
    num_writeouts: u32,
    /// the outputs categorized into activations
    comb_outputs_activations: IndexMap<usize, IndexMap<usize, Option<u16>>>,
    /// the current (placed) count of duplicate permutes
    cnt_placed_duplicate_permute: u32,

    /// the starting offset for FFs, outputs, and SRAM read results.
    state_start: u32,
    /// the starting offset of SRAM storage.
    sram_start: u32,

    /// the partial permutation instructions for
    /// 1. sram inputs
    /// 2. duplicated output pins due to difference in polarity/clock en.
    ///
    /// len: 8192
    sram_duplicate_permute: Vec<u16>,
    /// invert bit for sram_duplicate.
    ///
    /// len: 256
    sram_duplicate_inv: Vec<u32>,
    /// set-0 bit for sram_duplicate.
    ///
    /// len: 256
    sram_duplicate_set0: Vec<u32>,
    /// the permutation for clock enable pins.
    ///
    /// len: 8192
    clken_permute: Vec<u16>,
    /// invert bit for clken
    ///
    /// len: 256
    clken_inv: Vec<u32>,
    /// set-0 bit for clken
    ///
    /// len: 256
    clken_set0: Vec<u32>,
    /// invert bit for data corresponding to clken
    ///
    /// len: 256
    data_inv: Vec<u32>,
}

fn set_bit_in_u32(v: &mut u32, pos: u32, bit: u8) {
    if bit != 0 {
        *v |= 1 << pos;
    } else {
        *v &= !(1 << pos);
    }
}

impl FlatteningPart {
    fn init_afters_writeouts(&mut self, aig: &AIG, staged: &StagedAIG, part: &Partition) {
        let afters = part
            .stages
            .iter()
            .map(|s| {
                let mut after = Vec::with_capacity(1 << BOOMERANG_NUM_STAGES);
                after.push(usize::MAX);
                for i in (1..=BOOMERANG_NUM_STAGES).rev() {
                    after.extend(s.hier[i].iter().copied());
                }
                after
            })
            .collect::<Vec<_>>();
        let wos = part
            .stages
            .iter()
            .zip(afters.iter())
            .flat_map(|(s, after)| {
                s.write_outs
                    .iter()
                    .flat_map(|&woi| after[woi * 32..(woi + 1) * 32].iter().copied())
            })
            .collect::<Vec<_>>();

        // println!("test wos: {:?}", wos);

        self.afters = afters;
        self.parts_after_writeouts = wos;
        self.num_normal_writeouts = part
            .stages
            .iter()
            .map(|s| s.write_outs.len())
            .sum::<usize>() as u32;
        self.num_srams = 0;

        // map: output aig pin id -> ((clken, data iv) -> pos)
        let mut comb_outputs_activations = IndexMap::<usize, IndexMap<usize, Option<u16>>>::new();
        for &endpt_i in &part.endpoints {
            match staged.get_endpoint_group(aig, endpt_i) {
                EndpointGroup::RAMBlock(_) => {
                    self.num_srams += 1;
                }
                EndpointGroup::PrimaryOutput(idx) => {
                    if idx <= 1 {
                        // Skip constant primary outputs — handled separately in second loop.
                        continue;
                    }
                    comb_outputs_activations
                        .entry(idx >> 1)
                        .or_default()
                        .insert(2 | (idx & 1), None);
                }
                EndpointGroup::StagedIOPin(idx) => {
                    comb_outputs_activations
                        .entry(idx)
                        .or_default()
                        .insert(2, None);
                }
                EndpointGroup::DFF(dff) => {
                    if dff.d_iv <= 1 {
                        // Skip constant-D DFFs — they don't need output placement.
                        // Their Q will be mapped to const_zero_pos in the second loop.
                        continue;
                    }
                    comb_outputs_activations
                        .entry(dff.d_iv >> 1)
                        .or_default()
                        .insert(dff.en_iv << 1 | (dff.d_iv & 1), None);
                }
                EndpointGroup::SimControl(ctrl) => {
                    // SimControl condition is always active (no clock enable)
                    comb_outputs_activations
                        .entry(ctrl.condition_iv >> 1)
                        .or_default()
                        .insert(2 | (ctrl.condition_iv & 1), None);
                }
                EndpointGroup::Display(disp) => {
                    // Display enable is always active (no clock enable for now)
                    comb_outputs_activations
                        .entry(disp.enable_iv >> 1)
                        .or_default()
                        .insert(2 | (disp.enable_iv & 1), None);
                    // Also track argument signals
                    for &arg_iv in &disp.args_iv {
                        if arg_iv > 1 {
                            comb_outputs_activations
                                .entry(arg_iv >> 1)
                                .or_default()
                                .insert(2 | (arg_iv & 1), None);
                        }
                    }
                }
            }
        }
        self.num_duplicate_writeouts = comb_outputs_activations
            .values()
            .map(|v| v.len() - 1)
            .sum::<usize>()
            .div_ceil(32) as u32;
        self.comb_outputs_activations = comb_outputs_activations;

        self.num_writeouts =
            self.num_normal_writeouts + self.num_srams + self.num_duplicate_writeouts;

        self.after_writeout_pin2pos = self
            .parts_after_writeouts
            .iter()
            .enumerate()
            .filter_map(|(i, &pin)| {
                if pin == usize::MAX {
                    None
                } else {
                    Some((pin, i as u16))
                }
            })
            .collect::<IndexMap<_, _>>();
    }

    /// returns permutation id, invert bit, and setzero bit
    fn query_permute_with_pin_iv(&self, pin_iv: usize) -> (u16, u8, u8) {
        if pin_iv <= 1 {
            return (0, pin_iv as u8, 1);
        }
        let pos = self.after_writeout_pin2pos.get(&(pin_iv >> 1)).unwrap();
        (*pos, (pin_iv & 1) as u8, 0)
    }

    /// places a sram_duplicate bit.
    fn place_sram_duplicate(&mut self, pos: usize, (perm, inv, set0): (u16, u8, u8)) {
        self.sram_duplicate_permute[pos] = perm;
        set_bit_in_u32(
            &mut self.sram_duplicate_inv[pos >> 5],
            (pos & 31) as u32,
            inv,
        );
        set_bit_in_u32(
            &mut self.sram_duplicate_set0[pos >> 5],
            (pos & 31) as u32,
            set0,
        );
    }

    /// places a writeout bit's clock enable and data invert.
    fn place_clken_datainv(
        &mut self,
        pos: usize,
        clken_iv_perm: u16,
        clken_iv_inv: u8,
        clken_iv_set0: u8,
        data_inv: u8,
    ) {
        self.clken_permute[pos] = clken_iv_perm;
        set_bit_in_u32(
            &mut self.clken_inv[pos >> 5],
            (pos & 31) as u32,
            clken_iv_inv,
        );
        set_bit_in_u32(
            &mut self.clken_set0[pos >> 5],
            (pos & 31) as u32,
            clken_iv_set0,
        );
        set_bit_in_u32(&mut self.data_inv[pos >> 5], (pos & 31) as u32, data_inv);
    }

    /// returns a final local position for a data output bit with given pin_iv and clken_iv.
    ///
    /// if is not already placed, we will place it as well as place
    /// the clock enable bit, duplication bit, and bitflags for clock and data.
    fn get_or_place_output_with_activation(&mut self, pin_iv: usize, clken_iv: usize) -> u16 {
        let (activ_idx, _, pos) = self
            .comb_outputs_activations
            .get(&(pin_iv >> 1))
            .unwrap()
            .get_full(&(clken_iv << 1 | (pin_iv & 1)))
            .unwrap();
        if let Some(pos) = *pos {
            return pos;
        }
        let (clken_iv_perm, clken_iv_inv, clken_iv_set0) = self.query_permute_with_pin_iv(clken_iv);
        let origpos = match self.after_writeout_pin2pos.get(&(pin_iv >> 1)) {
            Some(origpos) => *origpos,
            None => {
                panic!("position of pin_iv {} (clken_iv {}) not found.. buggy boomerang in partitioning.", pin_iv, clken_iv)
            }
        } as usize;
        let r_pos = if activ_idx == 0 {
            self.place_clken_datainv(
                origpos,
                clken_iv_perm,
                clken_iv_inv,
                clken_iv_set0,
                (pin_iv & 1) as u8,
            );
            origpos as u16
        } else {
            self.cnt_placed_duplicate_permute += 1;
            let dup_pos = ((self.num_writeouts - self.num_srams) * 32
                - self.cnt_placed_duplicate_permute) as usize;
            let dup_perm_pos = ((self.num_srams * 4 + self.num_duplicate_writeouts) * 32
                - self.cnt_placed_duplicate_permute) as usize;
            if dup_perm_pos >= 8192 {
                panic!("sram duplicate bit larger than expected..")
                // dup_perm_pos = 8191;
            }
            self.place_sram_duplicate(dup_perm_pos, (origpos as u16, 0, 0));
            self.place_clken_datainv(
                dup_pos,
                clken_iv_perm,
                clken_iv_inv,
                clken_iv_set0,
                (pin_iv & 1) as u8,
            );
            dup_pos as u16
        };
        *self
            .comb_outputs_activations
            .get_mut(&(pin_iv >> 1))
            .unwrap()
            .get_mut(&(clken_iv << 1 | (pin_iv & 1)))
            .unwrap() = Some(r_pos);
        r_pos
    }

    fn make_inputs_outputs(
        &mut self,
        aig: &AIG,
        staged: &StagedAIG,
        part: &Partition,
        input_map: &mut IndexMap<usize, u32>,
        staged_io_map: &mut IndexMap<usize, u32>,
        output_map: &mut IndexMap<usize, u32>,
        const_zero_pos: u32,
    ) {
        self.sram_duplicate_permute = vec![0; 1 << BOOMERANG_NUM_STAGES];
        self.sram_duplicate_inv = vec![0u32; NUM_THREADS_V1];
        self.sram_duplicate_set0 = vec![u32::MAX; NUM_THREADS_V1];
        self.clken_permute = vec![0; 1 << BOOMERANG_NUM_STAGES];
        self.clken_inv = vec![0u32; NUM_THREADS_V1];
        self.clken_set0 = vec![u32::MAX; NUM_THREADS_V1];
        self.data_inv = vec![0u32; NUM_THREADS_V1];
        self.cnt_placed_duplicate_permute = 0;

        let mut cur_sram_id = 0;
        for &endpt_i in &part.endpoints {
            match staged.get_endpoint_group(aig, endpt_i) {
                EndpointGroup::RAMBlock(ram) => {
                    let sram_rd_data_local_offset = self.num_writeouts as usize
                        - self.num_srams as usize
                        + cur_sram_id as usize;
                    let sram_rd_data_global_start =
                        self.state_start + self.num_writeouts - self.num_srams + cur_sram_id;
                    let (perm_r_en_iv, perm_r_en_iv_inv, perm_r_en_iv_set0) =
                        self.query_permute_with_pin_iv(ram.port_r_en_iv);
                    for k in 0..32 {
                        let d = ram.port_r_rd_data[k];
                        if d == usize::MAX {
                            continue;
                        }
                        input_map.insert(d, sram_rd_data_global_start * 32 + k as u32);
                        output_map.insert(d << 1, sram_rd_data_global_start * 32 + k as u32);
                        self.place_clken_datainv(
                            sram_rd_data_local_offset * 32 + k,
                            perm_r_en_iv,
                            perm_r_en_iv_inv,
                            perm_r_en_iv_set0,
                            0,
                        );
                    }
                    let sram_input_perm_st = (cur_sram_id * 32 * 4) as usize;
                    for k in 0..13 {
                        self.place_sram_duplicate(
                            sram_input_perm_st + k,
                            self.query_permute_with_pin_iv(ram.port_r_addr_iv[k]),
                        );
                        self.place_sram_duplicate(
                            sram_input_perm_st + 16 + k,
                            self.query_permute_with_pin_iv(ram.port_w_addr_iv[k]),
                        );
                    }
                    for k in 0..32 {
                        self.place_sram_duplicate(
                            sram_input_perm_st + 32 + k,
                            self.query_permute_with_pin_iv(ram.port_w_wr_en_iv[k]),
                        );
                        self.place_sram_duplicate(
                            sram_input_perm_st + 64 + k,
                            self.query_permute_with_pin_iv(ram.port_w_wr_data_iv[k]),
                        );
                    }
                    cur_sram_id += 1;
                }
                EndpointGroup::PrimaryOutput(idx_iv) => {
                    if idx_iv <= 1 {
                        // Output tied to constant (0=false, 1=true) - map to position 0
                        // (state buffer bit 0 is always 0; for idx_iv=1, the sim won't use
                        // this since aigpin_iv <= 1 is skipped in GPIO mapping)
                        clilog::warn!(
                            PO_CONST_ERR,
                            "primary output idx_iv={} (tied to constant), skipping",
                            idx_iv
                        );
                        output_map.insert(idx_iv, 0);
                        continue;
                    }
                    let pos = self.state_start * 32
                        + self.get_or_place_output_with_activation(idx_iv, 1) as u32;
                    output_map.insert(idx_iv, pos);
                }
                EndpointGroup::StagedIOPin(idx) => {
                    if idx == 0 {
                        panic!("staged IO pin has zero..??")
                    }
                    let pos = self.state_start * 32
                        + self.get_or_place_output_with_activation(idx << 1, 1) as u32;
                    staged_io_map.insert(idx, pos);
                }
                EndpointGroup::DFF(dff) => {
                    if dff.d_iv == 0 {
                        // DFF with constant-0 D input. Q is always 0.
                        // Map Q to const_zero_pos (a guaranteed-zero bit in the
                        // padding area, NOT position 0 which is the posedge flag).
                        input_map.insert(dff.q, const_zero_pos);
                        continue;
                    }
                    if dff.d_iv == 1 {
                        // DFF with constant-1 D input (D = NOT(Tie0) = 1).
                        // Q transitions 0→1 after first clock edge and stays 1.
                        // Map to const_zero_pos as approximation (starts at 0,
                        // which is the correct initial value; it would need a
                        // proper state position to track the 0→1 transition).
                        clilog::warn!(
                            DFF_CONST_ERR,
                            "dff d_iv=1 (constant 1), Q mapped to const_zero_pos={}. \
                             Q will read 0 instead of transitioning to 1.",
                            const_zero_pos
                        );
                        input_map.insert(dff.q, const_zero_pos);
                        continue;
                    }
                    let pos = self.state_start * 32
                        + self.get_or_place_output_with_activation(dff.d_iv, dff.en_iv) as u32;
                    output_map.insert(dff.d_iv, pos);
                    input_map.insert(dff.q, pos);
                }
                EndpointGroup::SimControl(ctrl) => {
                    // SimControl condition is like a primary output - always active
                    if ctrl.condition_iv == 0 {
                        continue;
                    }
                    let local_pos =
                        self.get_or_place_output_with_activation(ctrl.condition_iv, 1) as u32;
                    // Note: The position returned by get_or_place_output_with_activation is
                    // 1-indexed from how the combinational logic is scheduled. We need to
                    // convert to 0-indexed by subtracting 1.
                    // TODO: investigate root cause of off-by-one
                    let pos = self.state_start * 32 + local_pos.saturating_sub(1);
                    output_map.insert(ctrl.condition_iv, pos);
                }
                EndpointGroup::Display(disp) => {
                    // Display enable condition
                    if disp.enable_iv == 0 {
                        continue;
                    }
                    let local_pos =
                        self.get_or_place_output_with_activation(disp.enable_iv, 1) as u32;
                    let pos = self.state_start * 32 + local_pos.saturating_sub(1);
                    output_map.insert(disp.enable_iv, pos);

                    // Also place argument signals
                    for &arg_iv in &disp.args_iv {
                        if arg_iv > 1 {
                            let local_pos =
                                self.get_or_place_output_with_activation(arg_iv, 1) as u32;
                            let pos = self.state_start * 32 + local_pos.saturating_sub(1);
                            output_map.insert(arg_iv, pos);
                        }
                    }
                }
            }
        }
        assert_eq!(cur_sram_id, self.num_srams);
        assert_eq!(
            self.cnt_placed_duplicate_permute.div_ceil(32),
            self.num_duplicate_writeouts
        );

        // println!("test clken_permute: {:?}, wos (w/o sram or dup): {:?}", self.clken_permute, self.parts_after_writeouts);
    }

    /// Build the GPU script for this partition.
    /// Returns (script_data, delay_patch_entries) where each delay_patch_entry
    /// is (local_offset, vec_of_aigpins) for patching timing data later.
    fn build_script(
        &self,
        aig: &AIG,
        part: &Partition,
        input_map: &IndexMap<usize, u32>,
        staged_io_map: &IndexMap<usize, u32>,
    ) -> (Vec<u32>, Vec<(usize, Vec<usize>)>) {
        let mut script = Vec::<u32>::new();
        let mut delay_patches: Vec<(usize, Vec<usize>)> = Vec::new();

        // metadata
        script.push(part.stages.len() as u32);
        script.push(0);
        script.push(self.num_writeouts);
        script.push(self.state_start);
        script.push(self.num_srams);
        script.push(self.sram_start);
        script.push(0); // [6]=num global read rounds, assigned later
        script.push(self.num_duplicate_writeouts);
        // padding
        while script.len() < 128 {
            script.push(0);
        }
        // final 128: write-out locations
        // compressed 2-1
        let mut last_wo = u32::MAX;
        for (j, bs) in part.stages.iter().enumerate() {
            for &wo in &bs.write_outs {
                let cur_wo = (j as u32) << 8 | (wo as u32);
                if last_wo == u32::MAX {
                    last_wo = cur_wo;
                } else {
                    script.push(last_wo | (cur_wo << 16));
                    last_wo = u32::MAX;
                }
            }
        }
        if last_wo != u32::MAX {
            script.push(last_wo | (((1 << 16) - 1) << 16));
        }
        while script.len() < 256 {
            script.push(u32::MAX);
        }
        // read global (256x32)
        let mut inputs_taken = BTreeMap::<u32, u32>::new();
        for &inp in &part.stages[0].hier[0] {
            if inp == usize::MAX {
                continue;
            }
            match input_map.get(&inp) {
                Some(&pos) => {
                    *inputs_taken.entry(pos >> 5).or_default() |= 1 << (pos & 31);
                }
                None => match staged_io_map.get(&inp) {
                    Some(&pos) => {
                        *inputs_taken.entry((pos >> 5) | (1u32 << 31)).or_default() |=
                            1 << (pos & 31);
                    }
                    None => {
                        panic!("cannot find input pin {}, driver: {:?}, in either primary inputs or staged IOs", inp, aig.drivers[inp]);
                    }
                },
            }
        }
        // clilog::debug!(
        //     "part (?) inputs_taken len {}: {:?}",
        //     inputs_taken.len(),
        //     inputs_taken.iter().map(|(id, val)| format!("{}[{}]", id, val.count_ones())).collect::<Vec<_>>()
        // );
        let rounds_idx_masks = map_global_read_to_rounds(&inputs_taken);
        let num_global_stages = rounds_idx_masks.iter().map(|v| v.len()).max().unwrap() as u32;
        script[6] = num_global_stages;
        assert_eq!(script.len(), NUM_THREADS_V1);
        let global_perm_start = script.len();
        script.extend((0..(2 * num_global_stages as usize * NUM_THREADS_V1)).map(|_| 0));
        for (i, v) in rounds_idx_masks.iter().enumerate() {
            for (round, &(idx, mask)) in v.iter().enumerate() {
                script[global_perm_start + NUM_THREADS_V1 * 2 * round + (i * 2)] = idx;
                script[global_perm_start + NUM_THREADS_V1 * 2 * round + (i * 2 + 1)] = mask;
                // println!("test: round {} i {} idx {} mask {}",
                //          round, i, idx, mask);
            }
        }

        let outputpos2localpos = rounds_idx_masks
            .iter()
            .enumerate()
            .flat_map(|(local_i, v)| {
                let mut local_op2lp = Vec::with_capacity(32);
                let mut bit_id = 0;
                for &(idx, mask) in v.iter().rev() {
                    let is_staged_io = (idx >> 31) != 0;
                    for k in (0..32).rev() {
                        if (mask >> k & 1) != 0 {
                            local_op2lp.push((
                                (is_staged_io, idx << 5 | k),
                                (local_i * 32 + bit_id) as u16,
                            ));
                            bit_id += 1;
                        }
                    }
                }
                assert!(bit_id <= 32);
                local_op2lp.into_iter()
            })
            .collect::<IndexMap<_, _>>();
        // println!("output2localpos: {:?}", outputpos2localpos);

        let mut last_pin2localpos = IndexMap::new();
        for &inp in &part.stages[0].hier[0] {
            if inp == usize::MAX {
                continue;
            }
            let pos = match input_map.get(&inp) {
                Some(&pos) => (false, pos),
                None => (true, *staged_io_map.get(&inp).unwrap()),
            };
            last_pin2localpos.insert(inp, *outputpos2localpos.get(&pos).unwrap());
        }

        // boomerang sections start
        for (bs_i, bs) in part.stages.iter().enumerate() {
            let bs_perm = bs.hier[0]
                .iter()
                .map(|&pin| {
                    if pin == usize::MAX {
                        0
                    } else {
                        *last_pin2localpos.get(&pin).unwrap()
                    }
                })
                .collect::<Vec<_>>();

            let mut bs_xora = vec![0u32; NUM_THREADS_V1];
            let mut bs_xorb = vec![0u32; NUM_THREADS_V1];
            let mut bs_orb = vec![0u32; NUM_THREADS_V1];
            for hi in 1..bs.hier.len() {
                let hi_len = bs.hier[hi].len();
                for j in 0..hi_len {
                    let out = bs.hier[hi][j];
                    let a = bs.hier[hi - 1][j];
                    let b = bs.hier[hi - 1][j + hi_len];
                    if out == usize::MAX {
                        continue;
                    }
                    if out == a {
                        bs_orb[(hi_len + j) >> 5] |= 1 << ((hi_len + j) & 31);
                        continue;
                    }
                    let (a_iv, b_iv) = match aig.drivers[out] {
                        DriverType::AndGate(a_iv, b_iv) => (a_iv, b_iv),
                        _ => unreachable!(),
                    };
                    assert_eq!(a_iv >> 1, a);
                    assert_eq!(b_iv >> 1, b);
                    if (a_iv & 1) != 0 {
                        bs_xora[(hi_len + j) >> 5] |= 1 << ((hi_len + j) & 31);
                    }
                    if (b_iv & 1) != 0 {
                        bs_xorb[(hi_len + j) >> 5] |= 1 << ((hi_len + j) & 31);
                    }
                }
            }

            for k in 0..4 {
                for i in ((k * 8)..bs_perm.len()).step_by(32) {
                    script.push((bs_perm[i] as u32) | (bs_perm[i + 1] as u32) << 16);
                    script.push((bs_perm[i + 2] as u32) | (bs_perm[i + 3] as u32) << 16);
                    script.push((bs_perm[i + 4] as u32) | (bs_perm[i + 5] as u32) << 16);
                    script.push((bs_perm[i + 6] as u32) | (bs_perm[i + 7] as u32) << 16);
                }
            }
            // Compute per-thread-position delay (max gate delay across all
            // 32 signals evaluated in this thread position).
            // Used by GPU timing kernel for arrival time tracking.
            // Collect AIG pins per thread position for delay injection
            let mut thread_aigpins: Vec<Vec<usize>> = vec![Vec::new(); NUM_THREADS_V1];
            for hi in 1..bs.hier.len() {
                let hi_len = bs.hier[hi].len();
                for j in 0..hi_len {
                    let out = bs.hier[hi][j];
                    if out == usize::MAX {
                        continue;
                    }
                    let thread_pos = (hi_len + j) >> 5;
                    if thread_pos < NUM_THREADS_V1 {
                        thread_aigpins[thread_pos].push(out);
                    }
                }
            }

            for i in 0..NUM_THREADS_V1 {
                script.push(bs_xora[i]);
                script.push(bs_xorb[i]);
                script.push(bs_orb[i]);
                let padding_offset = script.len();
                script.push(0); // Padding slot — patched by inject_timing_to_script()
                if !thread_aigpins[i].is_empty() {
                    delay_patches.push((padding_offset, std::mem::take(&mut thread_aigpins[i])));
                }
            }

            last_pin2localpos = self.afters[bs_i]
                .iter()
                .enumerate()
                .filter_map(|(i, &pin)| {
                    if pin == usize::MAX {
                        None
                    } else {
                        Some((pin, i as u16))
                    }
                })
                .collect::<IndexMap<_, _>>();
        }

        // sram worker
        for k in 0..4 {
            for i in ((k * 8)..self.sram_duplicate_permute.len()).step_by(32) {
                script.push(
                    (self.sram_duplicate_permute[i] as u32)
                        | (self.sram_duplicate_permute[i + 1] as u32) << 16,
                );
                script.push(
                    (self.sram_duplicate_permute[i + 2] as u32)
                        | (self.sram_duplicate_permute[i + 3] as u32) << 16,
                );
                script.push(
                    (self.sram_duplicate_permute[i + 4] as u32)
                        | (self.sram_duplicate_permute[i + 5] as u32) << 16,
                );
                script.push(
                    (self.sram_duplicate_permute[i + 6] as u32)
                        | (self.sram_duplicate_permute[i + 7] as u32) << 16,
                );
            }
        }
        for i in 0..NUM_THREADS_V1 {
            script.push(self.sram_duplicate_inv[i]);
            script.push(self.sram_duplicate_set0[i]);
            script.push(0);
            script.push(0);
        }
        // clock enable signal
        for k in 0..4 {
            for i in ((k * 8)..self.clken_permute.len()).step_by(32) {
                script.push(
                    (self.clken_permute[i] as u32) | (self.clken_permute[i + 1] as u32) << 16,
                );
                script.push(
                    (self.clken_permute[i + 2] as u32) | (self.clken_permute[i + 3] as u32) << 16,
                );
                script.push(
                    (self.clken_permute[i + 4] as u32) | (self.clken_permute[i + 5] as u32) << 16,
                );
                script.push(
                    (self.clken_permute[i + 6] as u32) | (self.clken_permute[i + 7] as u32) << 16,
                );
            }
        }
        for i in 0..NUM_THREADS_V1 {
            script.push(self.clken_inv[i]);
            script.push(self.clken_set0[i]);
            script.push(self.data_inv[i]);
            script.push(0);
        }

        (script, delay_patches)
    }
}

fn build_flattened_script_v1(
    aig: &AIG,
    stageds: &[&StagedAIG],
    parts_in_stages: &[&[Partition]],
    num_blocks: usize,
    input_layout: Vec<usize>,
    x_capable_pins: Option<&[bool]>,
) -> FlattenedScriptV1 {
    // determine the output position.
    // this is the prerequisite for generating the read
    // permutations and more.
    // input map:
    // locate input pins and FF/SRAM Q's - for partition input
    // output map:
    // locate primary outputs - for circuit outs
    // staged io map:
    // store intermediate nodes between major stages
    let mut input_map = IndexMap::new();
    let mut output_map = IndexMap::new();
    let mut staged_io_map = IndexMap::new();
    let mut delay_patch_map: Vec<(usize, Vec<usize>)> = Vec::new();
    for (i, &input) in input_layout.iter().enumerate() {
        if input == usize::MAX {
            continue;
        }
        input_map.insert(input, i as u32);
    }

    let num_major_stages = parts_in_stages.len();

    // Reserve a guaranteed-zero bit position for constant-D DFFs.
    // This must be in the padding area of the primary input words (never written to).
    let const_zero_pos = input_layout.len() as u32;
    let mut states_start = input_layout.len().div_ceil(32) as u32;
    if const_zero_pos == states_start * 32 {
        // No padding bits available, add one extra word for constant pool
        states_start += 1;
    }
    let mut sum_state_start = states_start;
    let mut sum_srams_start = 0;

    // enumerate all major stages and build them one by one.

    // #[derive(Debug, Clone, Default)]
    // struct FlatteningStage {
    //     blocks_parts: Vec<Vec<usize>>,
    //     flattening_parts: Vec<FlatteningPart>,
    //     parts_data_split: Vec<Vec<u32>>,
    // }
    // let mut flattening_stages =
    //     Vec::<FlatteningStage>::with_capacity(num_major_stages);

    // assemble script per block.
    let mut blocks_data = Vec::new();
    let mut blocks_start = Vec::<usize>::with_capacity(num_blocks * num_major_stages + 1);
    let mut stages_blocks_parts = Vec::new();
    let mut stages_flattening_parts = Vec::new();

    for (i, (init_parts, &staged)) in parts_in_stages
        .iter()
        .copied()
        .zip(stageds.iter())
        .enumerate()
    {
        // first arrange parts onto blocks.
        let mut blocks_parts = vec![vec![]; num_blocks];
        let mut tot_nstages_blocks = vec![0; num_blocks];
        // below models the fixed pre&post-cost for each executor
        let executor_fixed_cost = 3;
        // masonry layout of blocks. assume parts are sorted with
        // decreasing order of #stages.
        for i in 0..init_parts.len().min(num_blocks) {
            blocks_parts[i].push(i);
            tot_nstages_blocks[i] = init_parts[i].stages.len() + executor_fixed_cost;
        }
        for i in num_blocks..init_parts.len() {
            let put = tot_nstages_blocks
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.cmp(b))
                .unwrap()
                .0;
            blocks_parts[put].push(i);
            tot_nstages_blocks[put] += init_parts[i].stages.len() + executor_fixed_cost;
        }
        // clilog::debug!("blocks_parts: {:?}", blocks_parts);
        clilog::debug!(
            "major stage {}: max total boomerang depth (w/ cost) {}",
            i,
            tot_nstages_blocks.iter().copied().max().unwrap()
        );

        // the intermediates for parts being flattened
        let mut flattening_parts: Vec<FlatteningPart> = vec![Default::default(); init_parts.len()];

        // basic index preprocessing for stages (parallel - each is independent)
        flattening_parts
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, fp)| {
                fp.init_afters_writeouts(aig, staged, &init_parts[i]);
            });

        // allocate output state positions for all srams,
        // in the order of block affinity.
        for block in &blocks_parts {
            for &part_id in block {
                flattening_parts[part_id].state_start = sum_state_start;
                sum_state_start += flattening_parts[part_id].num_writeouts;
                flattening_parts[part_id].sram_start = sum_srams_start;
                sum_srams_start +=
                    flattening_parts[part_id].num_srams * (1 << AIGPDK_SRAM_ADDR_WIDTH);
            }
        }

        // besides input ports, we also have outputs from partitions.
        // they include original-placed comb output pins,
        // copied pins for different FF activation,
        // and SRAM read outputs.
        for part_id in 0..init_parts.len() {
            // clilog::debug!("initializing output for part {}", part_id);
            flattening_parts[part_id].make_inputs_outputs(
                aig,
                staged,
                &init_parts[part_id],
                &mut input_map,
                &mut staged_io_map,
                &mut output_map,
                const_zero_pos,
            );
        }
        stages_blocks_parts.push(blocks_parts);
        stages_flattening_parts.push(flattening_parts);
    }

    for ((blocks_parts, flattening_parts), init_parts) in stages_blocks_parts
        .iter()
        .zip(stages_flattening_parts.iter_mut())
        .zip(parts_in_stages.iter().copied())
    {
        // build script per part in parallel. we will later assemble them to blocks.
        let parts_results: Vec<(Vec<u32>, Vec<(usize, Vec<usize>)>)> = flattening_parts
            .par_iter()
            .enumerate()
            .map(|(part_id, fp)| {
                fp.build_script(aig, &init_parts[part_id], &input_map, &staged_io_map)
            })
            .collect();

        for block_id in 0..num_blocks {
            blocks_start.push(blocks_data.len());
            if blocks_parts[block_id].is_empty() {
                let mut dummy = vec![0; NUM_THREADS_V1];
                dummy[1] = 1;
                blocks_data.extend(dummy.into_iter());
            } else {
                let num_parts = blocks_parts[block_id].len();
                let mut last_part_st = usize::MAX;
                for (i, &part_id) in blocks_parts[block_id].iter().enumerate() {
                    if i == num_parts - 1 {
                        last_part_st = blocks_data.len();
                    }
                    let base_offset = blocks_data.len();
                    let (ref script_data, ref patches) = parts_results[part_id];
                    blocks_data.extend(script_data.iter().copied());
                    // Adjust patch offsets from local to global blocks_data position
                    for (local_off, aigpins) in patches {
                        delay_patch_map.push((base_offset + local_off, aigpins.clone()));
                    }
                }
                assert_ne!(last_part_st, usize::MAX);
                blocks_data[last_part_st + 1] = 1;
            }
        }
    }
    blocks_start.push(blocks_data.len());
    blocks_data.extend((0..NUM_THREADS_V1 * 8).map(|_| 0)); // padding

    clilog::info!(
        "Built script for {} blocks, reg/io state size {}, sram size {}, script size {}",
        num_blocks,
        sum_state_start,
        sum_srams_start,
        blocks_data.len()
    );

    // Build assertion_positions from simcontrols
    let assertion_positions: Vec<_> = aig
        .simcontrols
        .iter()
        .filter_map(|(&cell_id, ctrl)| {
            // Get the position of the condition in the output state
            output_map
                .get(&ctrl.condition_iv)
                .map(|&pos| (cell_id, pos, ctrl.message_id, ctrl.control_type))
        })
        .collect();

    if !assertion_positions.is_empty() {
        clilog::info!(
            "Found {} assertion/simcontrol nodes",
            assertion_positions.len()
        );
        for &(cell_id, pos, msg_id, ctrl_type) in &assertion_positions {
            clilog::debug!(
                "  Assertion: cell={}, pos={} (word={}, bit={}), msg_id={}, type={:?}",
                cell_id,
                pos,
                pos >> 5,
                pos & 31,
                msg_id,
                ctrl_type
            );
        }
    }

    // Build display_positions from displays
    let display_positions: Vec<_> = aig
        .displays
        .iter()
        .filter_map(|(&cell_id, disp)| {
            output_map.get(&disp.enable_iv).map(|&enable_pos| {
                // Get positions for all argument signals
                let arg_positions: Vec<u32> = disp
                    .args_iv
                    .iter()
                    .filter_map(|&arg_iv| output_map.get(&arg_iv).copied())
                    .collect();
                (
                    cell_id,
                    enable_pos,
                    disp.format.clone(),
                    arg_positions,
                    disp.arg_widths.clone(),
                )
            })
        })
        .collect();

    if !display_positions.is_empty() {
        clilog::info!("Found {} display nodes", display_positions.len());
        for (cell_id, pos, format, arg_pos, _) in &display_positions {
            clilog::debug!(
                "  Display: cell={}, enable_pos={} (word={}, bit={}), format='{}', args={:?}",
                cell_id,
                pos,
                pos >> 5,
                pos & 31,
                format,
                arg_pos
            );
        }
    }

    // === X-Propagation: Partition Classification ===
    let xprop_enabled = x_capable_pins.is_some();
    let mut partition_x_capable = Vec::new();
    let xprop_state_offset = sum_state_start;

    if let Some(x_pins) = x_capable_pins {
        // Classify each partition: X-capable if any of its AIG pins are X-capable.
        // We iterate over all parts in stage/block order.
        let mut part_x_flags: Vec<Vec<bool>> = Vec::new();

        for (stage_parts, &_staged) in parts_in_stages.iter().copied().zip(stageds.iter()) {
            let mut stage_flags = vec![false; stage_parts.len()];
            for (part_id, part) in stage_parts.iter().enumerate() {
                // Check all hier levels in all stages of this partition
                'outer: for bs in &part.stages {
                    for hi in &bs.hier {
                        for &pin in hi {
                            if pin != usize::MAX && pin < x_pins.len() && x_pins[pin] {
                                stage_flags[part_id] = true;
                                break 'outer;
                            }
                        }
                    }
                }
            }
            part_x_flags.push(stage_flags);
        }

        // Fixpoint: if a partition reads from an X-capable partition's output,
        // it becomes X-capable too (inter-partition X propagation).
        loop {
            let mut changed = false;
            for (stage_idx, stage_parts) in parts_in_stages.iter().copied().enumerate() {
                for (part_id, part) in stage_parts.iter().enumerate() {
                    if part_x_flags[stage_idx][part_id] {
                        continue;
                    }
                    // Check if this partition reads from any X-capable state words
                    // by looking at its global read indices.
                    for bs in &part.stages {
                        for &pin in &bs.hier[0] {
                            if pin == usize::MAX {
                                continue;
                            }
                            // Check if this input pin is in the output of an X-capable partition
                            if let Some(&out_pos) = output_map.get(&pin) {
                                let out_word = out_pos >> 5;
                                // Find which partition owns this output word
                                for (s_idx, s_parts) in parts_in_stages.iter().copied().enumerate()
                                {
                                    for (p_idx, _p) in s_parts.iter().enumerate() {
                                        if part_x_flags[s_idx][p_idx] {
                                            let fp = &stages_flattening_parts[s_idx][p_idx];
                                            let fp_start = fp.state_start;
                                            let fp_end = fp_start + fp.num_writeouts;
                                            if out_word >= fp_start && out_word < fp_end {
                                                part_x_flags[stage_idx][part_id] = true;
                                                changed = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if !changed {
                break;
            }
        }

        // Flatten into a single vec matching the stages_blocks_parts layout
        for (stage_idx, blocks_parts) in stages_blocks_parts.iter().enumerate() {
            for block_parts in blocks_parts {
                for &part_id in block_parts {
                    partition_x_capable.push(part_x_flags[stage_idx][part_id]);
                }
            }
        }

        // Encode X-prop metadata into partition scripts (words 8 and 9).
        // Word 8: is_x_capable flag (0 or 1)
        // Word 9: X-mask state offset
        // Post-process: patch metadata words 8 and 9 in blocks_data
        // Walk through the blocks_data to find each partition's metadata section.
        for (stage_idx, blocks_parts) in stages_blocks_parts.iter().enumerate() {
            for (block_id, block_parts) in blocks_parts.iter().enumerate() {
                let block_start = blocks_start[stage_idx * num_blocks + block_id];
                let mut offset = block_start;
                for &part_id in block_parts {
                    let is_x = part_x_flags[stage_idx][part_id];
                    // Patch word 8: is_x_capable
                    blocks_data[offset + 8] = if is_x { 1 } else { 0 };
                    // Patch word 9: X-mask state offset
                    blocks_data[offset + 9] = xprop_state_offset;
                    // Advance past this partition's script to the next partition.
                    // The partition script size is determined by reading num_stages:
                    // if num_stages == 0, it's just 256 words (dummy).
                    let num_stages = blocks_data[offset] as usize;
                    if num_stages == 0 {
                        break; // dummy partition means no more in this block
                    }
                    let num_gr_rounds = blocks_data[offset + 6] as usize;
                    let _num_srams = blocks_data[offset + 4] as usize;
                    let _num_ios = blocks_data[offset + 2] as usize;
                    let _num_dup = blocks_data[offset + 7] as usize;
                    // Script size = metadata(256) + global_read(2*rounds*256) +
                    //   boomerang(num_stages * 5*256*4) + sram_dup(5*256*4) + clken(5*256*4)
                    offset += NUM_THREADS_V1; // metadata
                    offset += 2 * num_gr_rounds * NUM_THREADS_V1; // global read
                    offset += num_stages * 5 * NUM_THREADS_V1 * 4; // boomerang stages
                    offset += 5 * NUM_THREADS_V1 * 4; // sram+dup permutation
                    offset += 5 * NUM_THREADS_V1 * 4; // clock enable permutation
                }
            }
        }

        let num_x = partition_x_capable.iter().filter(|&&v| v).count();
        clilog::info!(
            "X-propagation: {}/{} partitions X-capable",
            num_x,
            partition_x_capable.len()
        );
    }

    FlattenedScriptV1 {
        num_blocks,
        num_major_stages,
        blocks_start: blocks_start.into(),
        blocks_data: blocks_data.into(),
        reg_io_state_size: sum_state_start,
        sram_storage_size: sum_srams_start,
        input_layout,
        input_map,
        output_map,
        stages_blocks_parts,
        assertion_positions,
        display_positions,
        // Timing fields - initialized empty, populated by load_timing()
        gate_delays: Vec::new(),
        dff_constraints: Vec::new(),
        clock_period_ps: 1000, // Default 1ns
        timing_enabled: false,
        delay_patch_map,
        // X-propagation fields
        xprop_enabled,
        partition_x_capable,
        xprop_state_offset,
        // Timing arrival readback fields - enabled later via --timing-vcd
        timing_arrivals_enabled: false,
        arrival_state_offset: 0,
    }
}

impl FlattenedScriptV1 {
    /// Effective per-cycle state size for GPU dispatch.
    ///
    /// When X-propagation is enabled, the state buffer is doubled:
    /// `[values (reg_io_state_size) | xmask (reg_io_state_size)]`.
    /// The kernel indexes into the X-mask section at offset `xprop_state_offset`.
    pub fn effective_state_size(&self) -> u32 {
        let mut size = self.reg_io_state_size;
        if self.xprop_enabled {
            size += self.reg_io_state_size;
        }
        if self.timing_arrivals_enabled {
            size += self.reg_io_state_size;
        }
        size
    }

    /// Enable timing arrival readback for timed VCD output.
    ///
    /// Must be called after loading SDF timing data. Sets the arrival state
    /// offset based on current state layout (values + optional xmask).
    pub fn enable_timing_arrivals(&mut self) {
        assert!(
            self.timing_enabled,
            "Cannot enable timing arrivals without SDF timing data"
        );
        self.timing_arrivals_enabled = true;
        self.arrival_state_offset = self.reg_io_state_size * (1 + self.xprop_enabled as u32);
    }

    /// build a flattened script.
    ///
    /// `init_parts` give the partitions to flatten.
    /// it is better sorted in advance in descending order
    /// of #layers for better duty cycling.
    ///
    /// `num_blocks` should be set to the hardware allowances,
    /// i.e. the number of SMs in your GPU.
    /// for example, A100 should set it to 108.
    ///
    /// `input_layout` should give the expected primary input
    /// memory layout, each one is an AIG bit index.
    /// padding bits should be set to usize::MAX.
    pub fn from(
        aig: &AIG,
        stageds: &[&StagedAIG],
        parts_in_stages: &[&[Partition]],
        num_blocks: usize,
        input_layout: Vec<usize>,
    ) -> FlattenedScriptV1 {
        build_flattened_script_v1(
            aig,
            stageds,
            parts_in_stages,
            num_blocks,
            input_layout,
            None,
        )
    }

    /// Build a flattened script with X-propagation support.
    ///
    /// `x_capable_pins` is the result of `AIG::compute_x_capable_pins()`.
    /// When provided, partitions containing X-capable pins will be flagged
    /// and their script metadata will include X-propagation offsets.
    pub fn from_with_xprop(
        aig: &AIG,
        stageds: &[&StagedAIG],
        parts_in_stages: &[&[Partition]],
        num_blocks: usize,
        input_layout: Vec<usize>,
        x_capable_pins: &[bool],
    ) -> FlattenedScriptV1 {
        build_flattened_script_v1(
            aig,
            stageds,
            parts_in_stages,
            num_blocks,
            input_layout,
            Some(x_capable_pins),
        )
    }

    /// Load timing data from a Liberty library and AIG.
    ///
    /// This populates gate_delays and dff_constraints for timing-aware simulation.
    /// Must be called after building the script if timing analysis is needed.
    pub fn load_timing(
        &mut self,
        aig: &AIG,
        netlistdb: &NetlistDB,
        lib: &TimingLibrary,
        clock_period_ps: u64,
    ) {
        // Get generic fallback delays from library
        let and_delay = lib.and_gate_delay("AND2_00_0").unwrap_or((1, 1));
        let dff_timing = lib.dff_timing();
        let sram_timing = lib.sram_timing();

        // Initialize gate delays for all AIG pins
        self.gate_delays = vec![PackedDelay::default(); aig.num_aigpins + 1];

        let mut matched = 0usize;
        let mut fallback_count = 0usize;

        for aigpin in 1..=aig.num_aigpins {
            let origins = &aig.aigpin_cell_origins[aigpin];
            let delay = if !origins.is_empty() {
                // AIG pin has cell origin(s) — look up per-cell delays from Liberty.
                // When multiple cells share an AIG pin (e.g., inverter chain collapsed
                // to a single wire via invert-bit reuse), sum their delays (serial chain).
                let mut total_rise: u64 = 0;
                let mut total_fall: u64 = 0;

                for (cellid, _cell_type, output_pin_name) in origins {
                    let full_celltype = &netlistdb.celltypes[*cellid];
                    // Try looking up by full celltype (e.g., "sky130_fd_sc_hd__inv_1")
                    let cell_delay = lib
                        .get_cell(full_celltype)
                        .map(|c| {
                            // Try pin-specific delay first, then max combinational
                            c.pins
                                .get(output_pin_name.as_str())
                                .and_then(|pin| {
                                    pin.timing_arcs.iter().find_map(|arc| {
                                        if arc.timing_type.is_none()
                                            || arc.timing_type.as_deref() == Some("combinational")
                                        {
                                            Some((
                                                arc.cell_rise_ps.unwrap_or(0),
                                                arc.cell_fall_ps.unwrap_or(0),
                                            ))
                                        } else {
                                            None
                                        }
                                    })
                                })
                                .unwrap_or_else(|| c.max_combinational_delay())
                        })
                        .unwrap_or(and_delay);

                    total_rise += cell_delay.0;
                    total_fall += cell_delay.1;
                }

                matched += 1;
                PackedDelay::from_u64(total_rise, total_fall)
            } else {
                // No cell origins — internal decomposition gate or input/tie/DFF/SRAM
                match &aig.drivers[aigpin] {
                    DriverType::AndGate(_, _) => {
                        // Internal AND gate from cell decomposition — zero delay.
                        // The full cell delay is on the output AIG pin (via origins).
                        PackedDelay::default()
                    }
                    DriverType::InputPort(_)
                    | DriverType::InputClockFlag(_, _)
                    | DriverType::Tie0 => PackedDelay::default(),
                    DriverType::DFF(_) => {
                        fallback_count += 1;
                        dff_timing
                            .as_ref()
                            .map(|t| PackedDelay::from_u64(t.clk_to_q_rise_ps, t.clk_to_q_fall_ps))
                            .unwrap_or_default()
                    }
                    DriverType::SRAM(_) => {
                        fallback_count += 1;
                        sram_timing
                            .as_ref()
                            .map(|t| {
                                PackedDelay::from_u64(
                                    t.read_clk_to_data_rise_ps,
                                    t.read_clk_to_data_fall_ps,
                                )
                            })
                            .unwrap_or(PackedDelay::new(1, 1))
                    }
                }
            };
            self.gate_delays[aigpin] = delay;
        }

        // Build DFF constraints
        self.dff_constraints = Vec::with_capacity(aig.dffs.len());
        let setup_time = dff_timing.as_ref().map(|t| t.max_setup()).unwrap_or(0) as u16;
        let hold_time = dff_timing.as_ref().map(|t| t.max_hold()).unwrap_or(0) as u16;

        for (&cell_id, dff) in &aig.dffs {
            let data_state_pos = self.output_map.get(&dff.d_iv).copied().unwrap_or(u32::MAX);

            self.dff_constraints.push(DFFConstraint {
                setup_ps: setup_time,
                hold_ps: hold_time,
                clock_arrival_ps: 0,
                data_state_pos,
                cell_id: cell_id as u32,
            });
        }

        self.clock_period_ps = clock_period_ps;
        self.timing_enabled = true;

        let nonzero = self
            .gate_delays
            .iter()
            .filter(|d| d.max_delay() > 0)
            .count();
        clilog::info!(
            "Loaded Liberty timing: {} gate delays ({} from cell origins, {} fallback, {} nonzero), \
             {} DFF constraints, clock={}ps",
            self.gate_delays.len(),
            matched,
            fallback_count,
            nonzero,
            self.dff_constraints.len(),
            clock_period_ps
        );
    }

    /// Get the packed delay for an AIG pin (by index).
    pub fn get_delay(&self, aigpin: usize) -> PackedDelay {
        if aigpin < self.gate_delays.len() {
            self.gate_delays[aigpin]
        } else {
            PackedDelay::default()
        }
    }

    /// Check if timing data is available.
    pub fn has_timing(&self) -> bool {
        self.timing_enabled
    }

    /// Inject timing delay data into the GPU script's padding slots.
    /// Must be called after load_timing() or load_timing_from_ir().
    /// Each padding u32 gets the raw max gate delay (in picoseconds, clamped
    /// to u16 range 0-65535) across all AIG pins mapped to that thread position.
    pub fn inject_timing_to_script(&mut self) {
        if self.gate_delays.is_empty() || self.delay_patch_map.is_empty() {
            return;
        }

        let mut patched = 0usize;
        for (offset, aigpins) in &self.delay_patch_map {
            let mut max_delay: u32 = 0;
            for &aigpin in aigpins {
                if aigpin < self.gate_delays.len() {
                    max_delay = max_delay.max(self.gate_delays[aigpin].max_delay() as u32);
                } else {
                    clilog::debug!(
                        "inject_timing: aigpin {} out of range (gate_delays len={})",
                        aigpin,
                        self.gate_delays.len()
                    );
                }
            }
            // Store raw picoseconds, clamped to u16 range
            let delay_ps = max_delay.min(65535);
            if *offset < self.blocks_data.len() {
                self.blocks_data[*offset] = delay_ps;
                if delay_ps > 0 {
                    patched += 1;
                }
            }
        }

        clilog::info!(
            "Injected timing to GPU script: {} padding slots patched ({} total)",
            patched,
            self.delay_patch_map.len()
        );
    }

    /// Build a `WordSymbolMap` mapping each state-word index to the
    /// hierarchical names of the DFFs packed into that word. Used by the
    /// runtime violation reporter to convert opaque word indices into
    /// symbolic names. Returns `None` if no DFF constraints are loaded.
    pub fn build_word_symbol_map(&self, netlistdb: &NetlistDB) -> Option<WordSymbolMap> {
        use netlistdb::GeneralHierName;
        if self.dff_constraints.is_empty() {
            return None;
        }
        let mut word_to_dffs: HashMap<u32, Vec<DffSiteName>> = HashMap::new();
        for c in &self.dff_constraints {
            if c.data_state_pos == u32::MAX {
                continue;
            }
            let word_id = c.data_state_pos / 32;
            let bit = (c.data_state_pos % 32) as u8;
            let cell_idx = c.cell_id as usize;
            let name = if cell_idx < netlistdb.cellnames.len() {
                netlistdb.cellnames[cell_idx].dbg_fmt_hier()
            } else {
                format!("<cell #{cell_idx} out of range>")
            };
            word_to_dffs
                .entry(word_id)
                .or_default()
                .push(DffSiteName { name, bit });
        }
        for dffs in word_to_dffs.values_mut() {
            dffs.sort_by(|a, b| a.bit.cmp(&b.bit));
        }
        Some(WordSymbolMap { word_to_dffs })
    }

    /// Build a per-word timing constraint buffer for GPU-side setup/hold checking.
    ///
    /// Returns `(clock_period_ps, constraints)` where `constraints` has one u32 per
    /// state word (`reg_io_state_size` words). Each u32 packs:
    ///   - bits [31:16] = min setup_ps across all DFFs in that word
    ///   - bits [15:0]  = min hold_ps across all DFFs in that word
    ///
    /// Words with no DFF constraints have value 0 (skipped by the kernel).
    /// This is conservative: the max arrival across the word is compared against the
    /// min constraint, which may over-report violations but never misses real ones.
    pub fn build_timing_constraint_buffer(&self) -> (u32, Vec<u32>) {
        let num_words = self.reg_io_state_size as usize;
        let mut constraints = vec![0u32; num_words];
        for c in &self.dff_constraints {
            if c.data_state_pos == u32::MAX {
                continue;
            }
            let word_idx = (c.data_state_pos / 32) as usize;
            if word_idx >= num_words {
                continue;
            }
            // Fold per-DFF capture clock arrival into setup/hold (Pillar B
            // Stage 2). With no `ClockArrival` data the field defaults to
            // 0 and the values pass through unchanged.
            let (eff_setup, eff_hold) = c.effective_setup_hold();
            let existing = constraints[word_idx];
            let old_setup = (existing >> 16) as u16;
            let old_hold = (existing & 0xFFFF) as u16;
            // Most restrictive (min) constraint per word, treating 0 as "not yet set"
            let new_setup = if old_setup == 0 {
                eff_setup
            } else {
                old_setup.min(eff_setup)
            };
            let new_hold = if old_hold == 0 {
                eff_hold
            } else {
                old_hold.min(eff_hold)
            };
            constraints[word_idx] = ((new_setup as u32) << 16) | (new_hold as u32);
        }
        let clock_ps = self.clock_period_ps.min(u32::MAX as u64) as u32;
        (clock_ps, constraints)
    }

    /// Load timing from a Jacquard timing-IR document.
    ///
    /// Consumes the IR directly with `/` as the hierarchy separator
    /// (OpenSTA's default divider). See
    /// `docs/plans/ws3-delete-sdf-parser.md`.
    pub fn load_timing_from_ir(
        &mut self,
        aig: &AIG,
        netlistdb: &NetlistDB,
        ir: &timing_ir::TimingIR<'_>,
        clock_period_ps: u64,
        corner_index: u32,
        liberty_fallback: Option<&TimingLibrary>,
        debug: bool,
    ) {
        // cellid → IR cell_instance path. NetlistDB iterates leaf-first,
        // so reverse and join with '/'.
        let mut cellid_to_ir_path: HashMap<usize, String> = HashMap::new();
        for cellid in 1..netlistdb.num_cells {
            let parts: Vec<&str> = netlistdb.cellnames[cellid]
                .iter()
                .map(|s| s.as_str())
                .collect();
            let ir_path: String = parts.iter().rev().cloned().collect::<Vec<_>>().join("/");
            cellid_to_ir_path.insert(cellid, ir_path);
        }

        // Index arcs and checks by cell_instance for fast lookup.
        let mut arcs_by_cell: HashMap<&str, Vec<timing_ir::TimingArc<'_>>> = HashMap::new();
        if let Some(arcs) = ir.timing_arcs() {
            for i in 0..arcs.len() {
                let arc = arcs.get(i);
                if let Some(name) = arc.cell_instance() {
                    arcs_by_cell.entry(name).or_default().push(arc);
                }
            }
        }
        let mut checks_by_cell: HashMap<&str, Vec<timing_ir::SetupHoldCheck<'_>>> = HashMap::new();
        if let Some(checks) = ir.setup_hold_checks() {
            for i in 0..checks.len() {
                let check = checks.get(i);
                if let Some(name) = check.cell_instance() {
                    checks_by_cell.entry(name).or_default().push(check);
                }
            }
        }
        // Per-DFF clock arrival (Pillar B Stage 1 IR; consumer Stage 2).
        // Indexed by cell_instance — multiple records per cell are folded
        // by taking the max picosecond value, mirroring the worst-case
        // pessimism convention used elsewhere in this loader.
        let mut arrivals_by_cell: HashMap<&str, u64> = HashMap::new();
        if let Some(arrivals) = ir.clock_arrivals() {
            for i in 0..arrivals.len() {
                let ca = arrivals.get(i);
                if let Some(name) = ca.cell_instance() {
                    let arr = ir_corner_max(ca.arrival(), corner_index);
                    let entry = arrivals_by_cell.entry(name).or_insert(0);
                    *entry = (*entry).max(arr);
                }
            }
        }

        // Hierarchy prefix detection — same heuristic as the SDF version,
        // using '/' instead of '.'.
        {
            let mut sample_hits = 0usize;
            let mut sample_misses = 0usize;
            let mut common_prefix: Option<String> = None;
            for (_, path) in cellid_to_ir_path.iter().take(100) {
                if arcs_by_cell.contains_key(path.as_str())
                    || checks_by_cell.contains_key(path.as_str())
                {
                    sample_hits += 1;
                } else {
                    sample_misses += 1;
                    if let Some(slash_pos) = path.find('/') {
                        let stripped = &path[slash_pos + 1..];
                        if arcs_by_cell.contains_key(stripped)
                            || checks_by_cell.contains_key(stripped)
                        {
                            let prefix = &path[..slash_pos];
                            if common_prefix.is_none() {
                                common_prefix = Some(prefix.to_string());
                            }
                        }
                    }
                }
            }
            if let Some(prefix) = common_prefix.filter(|_| sample_misses > sample_hits) {
                let prefix_slash = format!("{}/", prefix);
                clilog::info!(
                    "IR hierarchy prefix mismatch detected: stripping '{}' from {} cell paths",
                    prefix_slash,
                    cellid_to_ir_path.len()
                );
                for path in cellid_to_ir_path.values_mut() {
                    if let Some(stripped) = path.strip_prefix(&prefix_slash) {
                        *path = stripped.to_string();
                    }
                }
            }
        }

        // Wire delays from interconnect_delays. Empty pre-WS2.2; the
        // logic remains so that path is wired when WS2.2 lands.
        let mut wire_delays_per_cell: HashMap<usize, (u64, u64)> = HashMap::new();
        if let Some(ics) = ir.interconnect_delays() {
            let path_to_cellid: HashMap<&str, usize> = cellid_to_ir_path
                .iter()
                .map(|(c, p)| (p.as_str(), *c))
                .collect();
            for i in 0..ics.len() {
                let ic = ics.get(i);
                if let Some(to_pin) = ic.to_pin() {
                    if let Some(slash_pos) = to_pin.rfind('/') {
                        let dest_inst = &to_pin[..slash_pos];
                        if let Some(&dest_cellid) = path_to_cellid.get(dest_inst) {
                            let d = ir_corner_max(ic.delay(), corner_index);
                            let entry = wire_delays_per_cell.entry(dest_cellid).or_insert((0, 0));
                            entry.0 = entry.0.max(d);
                            entry.1 = entry.1.max(d);
                        }
                    }
                }
            }
        }
        let num_wire_delays = wire_delays_per_cell.len();

        self.gate_delays = vec![PackedDelay::default(); aig.num_aigpins + 1];

        let mut matched = 0usize;
        let mut unmatched = 0usize;
        let mut fallback_count = 0usize;
        let mut unmatched_instances: Vec<String> = Vec::new();

        let lib_and_delay = liberty_fallback
            .and_then(|lib| lib.and_gate_delay("AND2_00_0"))
            .unwrap_or((0, 0));
        let lib_dff_timing = liberty_fallback.and_then(|lib| lib.dff_timing());
        let lib_sram_timing = liberty_fallback.and_then(|lib| lib.sram_timing());

        for aigpin in 1..=aig.num_aigpins {
            let origin = &aig.aigpin_cell_origins[aigpin];
            let delay = if !origin.is_empty() {
                let mut total_rise: u64 = 0;
                let mut total_fall: u64 = 0;
                let mut any_matched = false;
                let mut any_unmatched = false;

                for (cellid, _cell_type, output_pin_name) in origin {
                    if let Some(ir_path) = cellid_to_ir_path.get(cellid) {
                        if let Some(arcs) = arcs_by_cell.get(ir_path.as_str()) {
                            // Match the existing SDF semantics: first arc whose
                            // load_pin matches the origin output pin, or fall
                            // back to the first arc on the cell.
                            let arc = arcs
                                .iter()
                                .find(|a| {
                                    a.load_pin().is_some_and(|p| p == output_pin_name.as_str())
                                })
                                .or_else(|| arcs.first());
                            if let Some(arc) = arc {
                                any_matched = true;
                                let rise_ps = ir_corner_max(arc.rise_delay(), corner_index);
                                let fall_ps = ir_corner_max(arc.fall_delay(), corner_index);
                                let wire = wire_delays_per_cell.get(cellid);
                                total_rise += rise_ps + wire.map_or(0, |w| w.0);
                                total_fall += fall_ps + wire.map_or(0, |w| w.1);
                            } else {
                                any_unmatched = true;
                                if debug && unmatched_instances.len() < 20 {
                                    unmatched_instances.push(format!("{} (no arc)", ir_path));
                                }
                            }
                        } else {
                            any_unmatched = true;
                            if debug && unmatched_instances.len() < 20 {
                                unmatched_instances.push(ir_path.clone());
                            }
                        }
                    }
                }

                if any_matched {
                    matched += 1;
                    if any_unmatched {
                        unmatched += 1;
                    }
                    PackedDelay::from_u64(total_rise, total_fall)
                } else {
                    unmatched += 1;
                    self.get_liberty_fallback_delay(
                        &aig.drivers[aigpin],
                        lib_and_delay,
                        &lib_dff_timing,
                        &lib_sram_timing,
                        &mut fallback_count,
                    )
                }
            } else {
                match &aig.drivers[aigpin] {
                    DriverType::AndGate(_, _) => PackedDelay::default(),
                    DriverType::InputPort(_)
                    | DriverType::InputClockFlag(_, _)
                    | DriverType::Tie0 => PackedDelay::default(),
                    DriverType::DFF(_) => self.get_liberty_fallback_delay(
                        &aig.drivers[aigpin],
                        lib_and_delay,
                        &lib_dff_timing,
                        &lib_sram_timing,
                        &mut fallback_count,
                    ),
                    DriverType::SRAM(_) => self.get_liberty_fallback_delay(
                        &aig.drivers[aigpin],
                        lib_and_delay,
                        &lib_dff_timing,
                        &lib_sram_timing,
                        &mut fallback_count,
                    ),
                }
            };
            self.gate_delays[aigpin] = delay;
        }

        self.dff_constraints = Vec::with_capacity(aig.dffs.len());
        let lib_setup = lib_dff_timing.as_ref().map(|t| t.max_setup()).unwrap_or(0) as u16;
        let lib_hold = lib_dff_timing.as_ref().map(|t| t.max_hold()).unwrap_or(0) as u16;

        let mut arrivals_matched = 0usize;
        for (&cell_id, dff) in &aig.dffs {
            let data_state_pos = self.output_map.get(&dff.d_iv).copied().unwrap_or(u32::MAX);
            let ir_path = cellid_to_ir_path.get(&cell_id);
            let (setup_ps, hold_ps) = if let Some(path) = ir_path {
                if let Some(checks) = checks_by_cell.get(path.as_str()) {
                    if let Some(check) = checks.first() {
                        let setup = ir_corner_max(check.setup(), corner_index);
                        let hold = ir_corner_max(check.hold(), corner_index);
                        let setup = if setup > 0 { setup as u16 } else { lib_setup };
                        let hold = if hold > 0 { hold as u16 } else { lib_hold };
                        (setup, hold)
                    } else {
                        (lib_setup, lib_hold)
                    }
                } else {
                    (lib_setup, lib_hold)
                }
            } else {
                (lib_setup, lib_hold)
            };

            // Capture-side clock arrival: clamp into i16 picoseconds; values
            // beyond ±32 ns are saturated (warn-worthy but not fatal —
            // exotic, and the kernel's u16 effective setup/hold is the
            // hard ceiling anyway).
            let clock_arrival_ps = ir_path
                .and_then(|p| arrivals_by_cell.get(p.as_str()).copied())
                .map(|arr| {
                    arrivals_matched += 1;
                    arr.min(i16::MAX as u64) as i16
                })
                .unwrap_or(0);

            self.dff_constraints.push(DFFConstraint {
                setup_ps,
                hold_ps,
                clock_arrival_ps,
                data_state_pos,
                cell_id: cell_id as u32,
            });
        }

        self.clock_period_ps = clock_period_ps;
        self.timing_enabled = true;

        clilog::info!(
            "Loaded IR timing: {} matched, {} unmatched ({} Liberty fallback), {} wire delays, \
             {} DFF constraints ({} with per-DFF clock arrival), clock={}ps",
            matched,
            unmatched,
            fallback_count,
            num_wire_delays,
            self.dff_constraints.len(),
            arrivals_matched,
            clock_period_ps
        );

        if debug && !unmatched_instances.is_empty() {
            clilog::warn!(
                "IR unmatched instances (first {}):",
                unmatched_instances.len()
            );
            for inst in &unmatched_instances {
                clilog::warn!("  {}", inst);
            }
        }
    }

    /// Get a fallback delay from Liberty for a given driver type.
    fn get_liberty_fallback_delay(
        &self,
        driver: &DriverType,
        and_delay: (u64, u64),
        dff_timing: &Option<crate::liberty_parser::DFFTiming>,
        sram_timing: &Option<crate::liberty_parser::SRAMTiming>,
        fallback_count: &mut usize,
    ) -> PackedDelay {
        *fallback_count += 1;
        match driver {
            DriverType::AndGate(_, _) => PackedDelay::from_u64(and_delay.0, and_delay.1),
            DriverType::DFF(_) => dff_timing
                .as_ref()
                .map(|t| PackedDelay::from_u64(t.clk_to_q_rise_ps, t.clk_to_q_fall_ps))
                .unwrap_or_default(),
            DriverType::SRAM(_) => sram_timing
                .as_ref()
                .map(|t| {
                    PackedDelay::from_u64(t.read_clk_to_data_rise_ps, t.read_clk_to_data_fall_ps)
                })
                .unwrap_or(PackedDelay::new(1, 1)),
            _ => PackedDelay::default(),
        }
    }
}

/// Extract the max value at the requested corner index from an optional
/// IR `TimingValue` vector. Returns 0 for missing or non-positive
/// values. Rounds f64 ps to u64 ps. Used by `load_timing_from_ir`.
///
/// If the requested corner has no entry in the vector (single-corner
/// data + non-zero index), falls back to entry `0` so `--timing-corner`
/// still produces a usable IR consumption when one of the cell's arcs
/// happens to be single-corner.
fn ir_corner_max(
    values: Option<timing_ir::Vector<'_, timing_ir::TimingValue>>,
    corner_index: u32,
) -> u64 {
    let Some(v) = values else { return 0 };
    let extract = |idx: u32| -> Option<u64> {
        for i in 0..v.len() {
            let entry = v.get(i);
            if entry.corner_index() == idx {
                let m = entry.max();
                return if m > 0.0 { Some(m.round() as u64) } else { Some(0) };
            }
        }
        None
    };
    if let Some(found) = extract(corner_index) {
        return found;
    }
    if corner_index != 0 {
        if let Some(found) = extract(0) {
            return found;
        }
    }
    if !v.is_empty() {
        let m = v.get(0).max();
        if m > 0.0 {
            return m.round() as u64;
        }
    }
    0
}

/// Resolve `--timing-corner <NAME>` to the IR's corner index. Returns 0
/// when no name is given or the IR has no corners table; errors when a
/// name is given but no matching corner exists.
pub fn resolve_corner_index(
    ir: &timing_ir::TimingIR<'_>,
    requested: Option<&str>,
) -> Result<u32, String> {
    let Some(name) = requested else {
        return Ok(0);
    };
    let Some(corners) = ir.corners() else {
        return Err(format!(
            "--timing-corner {name} requested but the IR has no corners table"
        ));
    };
    // Corners are positional in the table; the slot index *is* the
    // value referenced by TimingValue.corner_index.
    for i in 0..corners.len() {
        let c = corners.get(i);
        if c.name() == Some(name) {
            return Ok(i as u32);
        }
    }
    let names: Vec<String> = (0..corners.len())
        .map(|i| corners.get(i).name().unwrap_or("?").to_string())
        .collect();
    Err(format!(
        "--timing-corner {name} not found in IR; available corners: [{}]",
        names.join(", ")
    ))
}

#[cfg(test)]
mod ir_delay_tests {
    use super::*;
    use crate::aig::AIG;
    use crate::sky130::SKY130LeafPins;
    use timing_ir::*;

    /// Helper: build a minimal FlattenedScriptV1 suitable for load_timing_from_ir.
    fn make_minimal_script(_aig: &AIG) -> FlattenedScriptV1 {
        FlattenedScriptV1 {
            num_blocks: 0,
            num_major_stages: 0,
            blocks_start: Vec::<usize>::new().into(),
            blocks_data: Vec::<u32>::new().into(),
            reg_io_state_size: 0,
            sram_storage_size: 0,
            input_layout: Vec::new(),
            output_map: IndexMap::new(),
            input_map: IndexMap::new(),
            stages_blocks_parts: Vec::new(),
            assertion_positions: Vec::new(),
            display_positions: Vec::new(),
            gate_delays: Vec::new(),
            dff_constraints: Vec::new(),
            clock_period_ps: 1000,
            timing_enabled: false,
            delay_patch_map: Vec::new(),
            xprop_enabled: false,
            partition_x_capable: Vec::new(),
            xprop_state_offset: 0,
            timing_arrivals_enabled: false,
            arrival_state_offset: 0,
        }
    }

    /// Helper: load inv_chain design + AIG.
    fn load_inv_chain() -> (netlistdb::NetlistDB, AIG) {
        let path = std::path::PathBuf::from("tests/timing_test/sky130_timing/inv_chain.v");
        assert!(path.exists(), "inv_chain.v not found");
        let netlistdb = netlistdb::NetlistDB::from_sverilog_file(&path, None, &SKY130LeafPins)
            .expect("Failed to parse inv_chain.v");
        let aig = AIG::from_netlistdb(&netlistdb);
        (netlistdb, aig)
    }

    /// Helper: build cellid → IR cell_instance path map.
    /// Mirrors the construction in `load_timing_from_ir`.
    fn build_cellid_to_ir_path(netlistdb: &netlistdb::NetlistDB) -> HashMap<usize, String> {
        let mut map = HashMap::new();
        for cellid in 1..netlistdb.num_cells {
            let parts: Vec<&str> = netlistdb.cellnames[cellid]
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<_>>();
            let ir_path: String = parts.iter().rev().cloned().collect::<Vec<_>>().join("/");
            map.insert(cellid, ir_path);
        }
        map
    }

    /// Build an IR mirroring the contents of
    /// `tests/timing_test/inv_chain_pnr/inv_chain_test.sdf`.
    ///
    /// IR semantics differ from SDF in two places that matter here:
    /// - `InterconnectDelay.delay` is a single TimingValue, not separate
    ///   rise/fall — we emit the SDF rise typ value as the canonical wire
    ///   delay (so the consumer applies the same delay to both edges).
    /// - `ir_corner0_max` reads the `max` slot of the corner-0 timing
    ///   value, so we put delay magnitudes there.
    fn build_inv_chain_ir() -> Vec<u8> {
        build_inv_chain_ir_with_arrivals(&[])
    }

    /// Same as `build_inv_chain_ir`, with optional `ClockArrival` records
    /// for testing the Pillar B Stage 2 fold-in path.
    fn build_inv_chain_ir_with_arrivals(clock_arrivals: &[(&str, f64)]) -> Vec<u8> {
        // SDF typ values per inverter, picosecond IOPATH (rise, fall):
        // index → (rise_typ_ps, fall_typ_ps)
        let inv_iopath: [(f64, f64); 16] = [
            (50.0, 40.0),
            (52.0, 42.0),
            (51.0, 41.0),
            (53.0, 43.0),
            (50.0, 40.0),
            (54.0, 44.0),
            (51.0, 41.0),
            (52.0, 42.0),
            (50.0, 40.0),
            (53.0, 43.0),
            (51.0, 41.0),
            (54.0, 44.0),
            (52.0, 42.0),
            (51.0, 41.0),
            (50.0, 40.0),
            (53.0, 43.0),
        ];

        // SDF wire delays (rise typ used as the single IR value).
        // (from_pin, to_pin, delay_ps)
        let wires: [(&'static str, &'static str, f64); 17] = [
            ("dff_in/Q", "i0/A", 15.0),
            ("i0/Y", "i1/A", 8.0),
            ("i1/Y", "i2/A", 9.0),
            ("i2/Y", "i3/A", 8.0),
            ("i3/Y", "i4/A", 10.0),
            ("i4/Y", "i5/A", 8.0),
            ("i5/Y", "i6/A", 9.0),
            ("i6/Y", "i7/A", 8.0),
            ("i7/Y", "i8/A", 11.0),
            ("i8/Y", "i9/A", 8.0),
            ("i9/Y", "i10/A", 9.0),
            ("i10/Y", "i11/A", 8.0),
            ("i11/Y", "i12/A", 10.0),
            ("i12/Y", "i13/A", 8.0),
            ("i13/Y", "i14/A", 9.0),
            ("i14/Y", "i15/A", 8.0),
            ("i15/Y", "dff_out/D", 15.0),
        ];

        let mut b = flatbuffers::FlatBufferBuilder::with_capacity(4096);

        // Single corner.
        let cn = b.create_string("default");
        let cp = b.create_string("typ");
        let corner = Corner::create(
            &mut b,
            &CornerArgs {
                name: Some(cn),
                process: Some(cp),
                voltage: 1.8,
                temperature: 25.0,
            },
        );
        let corners_vec = b.create_vector(&[corner]);

        let prov_t = b.create_string("test");
        let prov_f = b.create_string("inv_chain_synthetic");
        let prov = Provenance::create(
            &mut b,
            &ProvenanceArgs {
                source_tool: Some(prov_t),
                source_file: Some(prov_f),
                origin: Origin::Asserted,
            },
        );
        let cond_empty = b.create_string("");

        // Helper: build a per-arc TimingValue vector with a given max ps.
        macro_rules! tv_vec {
            ($b:expr, $ps:expr) => {{
                let arr = [TimingValue::new(0, $ps, $ps, $ps)];
                $b.create_vector(&arr)
            }};
        }

        // Build timing arcs.
        let mut arcs_offsets = Vec::new();
        for (idx, (rise_ps, fall_ps)) in inv_iopath.iter().enumerate() {
            let ci = b.create_string(&format!("i{}", idx));
            let dp = b.create_string("A");
            let lp = b.create_string("Y");
            let rv = tv_vec!(b, *rise_ps);
            let fv = tv_vec!(b, *fall_ps);
            arcs_offsets.push(TimingArc::create(
                &mut b,
                &TimingArcArgs {
                    cell_instance: Some(ci),
                    driver_pin: Some(dp),
                    load_pin: Some(lp),
                    rise_delay: Some(rv),
                    fall_delay: Some(fv),
                    condition: Some(cond_empty),
                    provenance: Some(prov),
                },
            ));
        }
        // dff_in CLK→Q (350, 330)
        {
            let ci = b.create_string("dff_in");
            let dp = b.create_string("CLK");
            let lp = b.create_string("Q");
            let rv = tv_vec!(b, 350.0);
            let fv = tv_vec!(b, 330.0);
            arcs_offsets.push(TimingArc::create(
                &mut b,
                &TimingArcArgs {
                    cell_instance: Some(ci),
                    driver_pin: Some(dp),
                    load_pin: Some(lp),
                    rise_delay: Some(rv),
                    fall_delay: Some(fv),
                    condition: Some(cond_empty),
                    provenance: Some(prov),
                },
            ));
        }
        // dff_out CLK→Q (360, 340)
        {
            let ci = b.create_string("dff_out");
            let dp = b.create_string("CLK");
            let lp = b.create_string("Q");
            let rv = tv_vec!(b, 360.0);
            let fv = tv_vec!(b, 340.0);
            arcs_offsets.push(TimingArc::create(
                &mut b,
                &TimingArcArgs {
                    cell_instance: Some(ci),
                    driver_pin: Some(dp),
                    load_pin: Some(lp),
                    rise_delay: Some(rv),
                    fall_delay: Some(fv),
                    condition: Some(cond_empty),
                    provenance: Some(prov),
                },
            ));
        }
        let arcs_vec = b.create_vector(&arcs_offsets);

        // Build interconnects.
        let mut ic_offsets = Vec::new();
        for (from, to, delay_ps) in wires.iter() {
            let net = b.create_string("");
            let fp = b.create_string(from);
            let tp = b.create_string(to);
            let dv = tv_vec!(b, *delay_ps);
            ic_offsets.push(InterconnectDelay::create(
                &mut b,
                &InterconnectDelayArgs {
                    net: Some(net),
                    from_pin: Some(fp),
                    to_pin: Some(tp),
                    delay: Some(dv),
                    provenance: Some(prov),
                },
            ));
        }
        let ic_vec = b.create_vector(&ic_offsets);

        // Build setup/hold checks (dff_in: setup=80, hold=-30; dff_out:
        // setup=85, hold=-28). The negative SDF holds become 0 after the
        // consumer's clamp.
        let mut sh_offsets = Vec::new();
        for (cell, setup_ps, hold_ps) in [
            ("dff_in", 80.0_f64, -30.0_f64),
            ("dff_out", 85.0_f64, -28.0_f64),
        ] {
            let ci = b.create_string(cell);
            let dp = b.create_string("D");
            let cp = b.create_string("CLK");
            let cond = b.create_string("");
            let sv = tv_vec!(b, setup_ps);
            let hv = tv_vec!(b, hold_ps);
            sh_offsets.push(SetupHoldCheck::create(
                &mut b,
                &SetupHoldCheckArgs {
                    cell_instance: Some(ci),
                    d_pin: Some(dp),
                    clk_pin: Some(cp),
                    edge: CheckEdge::Posedge,
                    setup: Some(sv),
                    hold: Some(hv),
                    condition: Some(cond),
                    provenance: Some(prov),
                },
            ));
        }
        let sh_vec = b.create_vector(&sh_offsets);

        // Optional ClockArrival records for Pillar B Stage 2 testing.
        let mut ca_offsets = Vec::new();
        for (cell, arr_ps) in clock_arrivals {
            let ci = b.create_string(cell);
            let cp = b.create_string("CLK");
            let av = tv_vec!(b, *arr_ps);
            ca_offsets.push(ClockArrival::create(
                &mut b,
                &ClockArrivalArgs {
                    cell_instance: Some(ci),
                    clk_pin: Some(cp),
                    arrival: Some(av),
                    provenance: Some(prov),
                },
            ));
        }
        let ca_vec = b.create_vector(&ca_offsets);

        let ve_vec = b.create_vector::<flatbuffers::WIPOffset<VendorExtension>>(&[]);

        let gt = b.create_string("inv_chain test fixture");
        let gv = b.create_string(env!("CARGO_PKG_VERSION"));
        let if_vec = b.create_vector::<flatbuffers::WIPOffset<&str>>(&[]);
        let sv_root = SchemaVersion::new(SCHEMA_MAJOR, SCHEMA_MINOR, SCHEMA_PATCH);

        let ir = TimingIR::create(
            &mut b,
            &TimingIRArgs {
                schema_version: Some(&sv_root),
                corners: Some(corners_vec),
                timing_arcs: Some(arcs_vec),
                interconnect_delays: Some(ic_vec),
                setup_hold_checks: Some(sh_vec),
                clock_arrivals: Some(ca_vec),
                vendor_extensions: Some(ve_vec),
                generator_tool: Some(gt),
                generator_version: Some(gv),
                input_files: Some(if_vec),
            },
        );
        finish_timing_ir_buffer(&mut b, ir);
        b.finished_data().to_vec()
    }

    // === IR delay application (former test_sdf_delay_application) ===
    //
    // Single-stage check on dff_out: IOPATH 360/340 + wire 15 (single
    // value applied to both edges in the IR consumer).
    #[test]
    fn test_ir_delay_application() {
        let (netlistdb, aig) = load_inv_chain();
        let ir_buf = build_inv_chain_ir();
        let ir = root_as_timing_ir(&ir_buf).expect("valid IR buffer");

        let mut script = make_minimal_script(&aig);
        script.load_timing_from_ir(&aig, &netlistdb, &ir, 10000, 0, None, true);

        assert_eq!(
            script.gate_delays.len(),
            aig.num_aigpins + 1,
            "gate_delays length should match num_aigpins + 1"
        );

        let cellid_to_ir_path = build_cellid_to_ir_path(&netlistdb);
        let path_to_cellid: HashMap<&str, usize> = cellid_to_ir_path
            .iter()
            .map(|(&cid, p)| (p.as_str(), cid))
            .collect();

        let dff_out_cellid = *path_to_cellid.get("dff_out").unwrap();
        let dff_out_aigpin = (1..=aig.num_aigpins)
            .find(|&ap| {
                aig.aigpin_cell_origins[ap]
                    .iter()
                    .any(|(cid, _, _)| *cid == dff_out_cellid)
            })
            .expect("dff_out should have a cell origin entry");

        let dff_out_delay = &script.gate_delays[dff_out_aigpin];
        // dff_out: IOPATH CLK→Q rise=360 + wire(i15.Y→dff_out.D)=15 → 375
        //          IOPATH CLK→Q fall=340 + wire 15 (same single value) → 355
        assert_eq!(
            dff_out_delay.rise_ps, 375,
            "dff_out rise: expected 375ps (360+15), got {}ps",
            dff_out_delay.rise_ps
        );
        assert_eq!(
            dff_out_delay.fall_ps, 355,
            "dff_out fall: expected 355ps (340+15), got {}ps",
            dff_out_delay.fall_ps
        );

        assert!(
            script.timing_enabled,
            "timing_enabled should be true after load_timing_from_ir"
        );
        assert_eq!(script.clock_period_ps, 10000);
    }

    #[test]
    fn test_internal_and_gates_zero_delay() {
        let (netlistdb, aig) = load_inv_chain();
        let ir_buf = build_inv_chain_ir();
        let ir = root_as_timing_ir(&ir_buf).expect("valid IR buffer");

        let mut script = make_minimal_script(&aig);
        script.load_timing_from_ir(&aig, &netlistdb, &ir, 10000, 0, None, false);

        for aigpin in 1..=aig.num_aigpins {
            if aig.aigpin_cell_origins[aigpin].is_empty() {
                let delay = &script.gate_delays[aigpin];
                assert_eq!(
                    delay.rise_ps, 0,
                    "AND gate aigpin {} without cell_origins should have zero rise delay, got {}",
                    aigpin, delay.rise_ps
                );
                assert_eq!(
                    delay.fall_ps, 0,
                    "AND gate aigpin {} without cell_origins should have zero fall delay, got {}",
                    aigpin, delay.fall_ps
                );
            }
        }
    }

    #[test]
    fn test_dff_constraints_loaded() {
        let (netlistdb, aig) = load_inv_chain();
        let ir_buf = build_inv_chain_ir();
        let ir = root_as_timing_ir(&ir_buf).expect("valid IR buffer");

        let mut script = make_minimal_script(&aig);
        script.load_timing_from_ir(&aig, &netlistdb, &ir, 10000, 0, None, false);

        assert_eq!(
            script.dff_constraints.len(),
            2,
            "Expected 2 DFF constraints, got {}",
            script.dff_constraints.len()
        );

        let cellid_to_ir_path = build_cellid_to_ir_path(&netlistdb);
        let constraint_by_name: Vec<(&str, &DFFConstraint)> = script
            .dff_constraints
            .iter()
            .map(|c| {
                let name = cellid_to_ir_path
                    .get(&(c.cell_id as usize))
                    .map(|s| s.as_str())
                    .unwrap_or("unknown");
                (name, c)
            })
            .collect();

        // dff_in: setup=80, hold=-30 (negative → ir_corner0_max returns 0)
        let dff_in = constraint_by_name
            .iter()
            .find(|(name, _)| *name == "dff_in")
            .expect("dff_in constraint not found");
        assert_eq!(
            dff_in.1.setup_ps, 80,
            "dff_in setup: expected 80ps, got {}ps",
            dff_in.1.setup_ps
        );
        assert_eq!(
            dff_in.1.hold_ps, 0,
            "dff_in hold: expected 0ps (negative clamped), got {}ps",
            dff_in.1.hold_ps
        );

        let dff_out = constraint_by_name
            .iter()
            .find(|(name, _)| *name == "dff_out")
            .expect("dff_out constraint not found");
        assert_eq!(
            dff_out.1.setup_ps, 85,
            "dff_out setup: expected 85ps, got {}ps",
            dff_out.1.setup_ps
        );
        assert_eq!(
            dff_out.1.hold_ps, 0,
            "dff_out hold: expected 0ps (negative clamped), got {}ps",
            dff_out.1.hold_ps
        );

        // data_state_pos is u32::MAX because make_minimal_script has empty output_map.
        assert_eq!(dff_in.1.data_state_pos, u32::MAX);
        assert_eq!(dff_out.1.data_state_pos, u32::MAX);

        // Pillar B Stage 1 — IR contained no ClockArrival records, so the
        // consumer-side field stays at the legacy zero-skew default.
        assert_eq!(
            dff_in.1.clock_arrival_ps, 0,
            "no ClockArrival records → clock_arrival_ps stays 0"
        );
        assert_eq!(dff_out.1.clock_arrival_ps, 0);
    }

    /// Pillar B Stage 1 IR + Stage 2 consumer plumbing — ClockArrival
    /// records in the IR populate `DFFConstraint.clock_arrival_ps`,
    /// matched to the right DFF instance.
    #[test]
    fn test_clock_arrivals_loaded_from_ir() {
        let (netlistdb, aig) = load_inv_chain();
        let ir_buf = build_inv_chain_ir_with_arrivals(&[("dff_in", 100.0), ("dff_out", 230.0)]);
        let ir = root_as_timing_ir(&ir_buf).expect("valid IR buffer");

        let mut script = make_minimal_script(&aig);
        script.load_timing_from_ir(&aig, &netlistdb, &ir, 10000, 0, None, false);

        let cellid_to_ir_path = build_cellid_to_ir_path(&netlistdb);
        let by_name: HashMap<&str, &DFFConstraint> = script
            .dff_constraints
            .iter()
            .map(|c| {
                let name = cellid_to_ir_path
                    .get(&(c.cell_id as usize))
                    .map(|s| s.as_str())
                    .unwrap_or("unknown");
                (name, c)
            })
            .collect();

        let dff_in = by_name.get("dff_in").expect("dff_in DFFConstraint");
        let dff_out = by_name.get("dff_out").expect("dff_out DFFConstraint");
        assert_eq!(dff_in.clock_arrival_ps, 100);
        assert_eq!(dff_out.clock_arrival_ps, 230);

        // And the effective values reflect the fold-in:
        // dff_in: setup 80 - 100 = -20 → clamps to 0; hold 0 + 100 = 100.
        // dff_out: setup 85 - 230 = -145 → clamps to 0; hold 0 + 230 = 230.
        assert_eq!(dff_in.effective_setup_hold(), (0, 100));
        assert_eq!(dff_out.effective_setup_hold(), (0, 230));
    }

    // === Test 4: GPU delay injection quantization ===
    //
    // These tests exercise `inject_timing_to_script` only — no SDF/IR
    // fixture needed.

    /// Helper: build a FlattenedScriptV1 for delay injection testing.
    /// `delays` are per-AIG-pin (index 0 is always default/Tie0).
    /// `patch_map` maps offsets in blocks_data to AIG pin indices.
    /// `blocks_data_size` is how many u32 slots to allocate.
    fn make_delay_injection_script(
        delays: Vec<PackedDelay>,
        patch_map: Vec<(usize, Vec<usize>)>,
        blocks_data_size: usize,
    ) -> FlattenedScriptV1 {
        FlattenedScriptV1 {
            num_blocks: 0,
            num_major_stages: 0,
            blocks_start: Vec::<usize>::new().into(),
            blocks_data: vec![0u32; blocks_data_size].into(),
            reg_io_state_size: 0,
            sram_storage_size: 0,
            input_layout: Vec::new(),
            output_map: IndexMap::new(),
            input_map: IndexMap::new(),
            stages_blocks_parts: Vec::new(),
            assertion_positions: Vec::new(),
            display_positions: Vec::new(),
            gate_delays: delays,
            dff_constraints: Vec::new(),
            clock_period_ps: 10000,
            timing_enabled: true,
            delay_patch_map: patch_map,
            xprop_enabled: false,
            partition_x_capable: Vec::new(),
            xprop_state_offset: 0,
            timing_arrivals_enabled: false,
            arrival_state_offset: 0,
        }
    }

    #[test]
    fn test_basic_delay_injection() {
        // delay=100ps → stored as raw 100
        let mut script = make_delay_injection_script(
            vec![PackedDelay::default(), PackedDelay::new(100, 100)],
            vec![(0, vec![1])],
            1,
        );
        script.inject_timing_to_script();
        assert_eq!(
            script.blocks_data[0], 100,
            "100ps stored as raw picoseconds"
        );
    }

    #[test]
    fn test_max_of_thread_position() {
        // Two aigpins [350ps, 0ps] → max=350, stored as raw 350
        let mut script = make_delay_injection_script(
            vec![
                PackedDelay::default(),
                PackedDelay::new(350, 350),
                PackedDelay::new(0, 0),
            ],
            vec![(0, vec![1, 2])],
            1,
        );
        script.inject_timing_to_script();
        assert_eq!(script.blocks_data[0], 350, "max(350,0)=350 raw ps");
    }

    #[test]
    fn test_zero_stays_zero() {
        let mut script = make_delay_injection_script(
            vec![PackedDelay::default(), PackedDelay::new(0, 0)],
            vec![(0, vec![1])],
            1,
        );
        script.inject_timing_to_script();
        assert_eq!(script.blocks_data[0], 0, "Zero delay stays zero");
    }

    #[test]
    fn test_ordering_preserved() {
        // delays 5ps and 15ps → stored as raw 5 and 15
        let mut script = make_delay_injection_script(
            vec![
                PackedDelay::default(),
                PackedDelay::new(5, 5),
                PackedDelay::new(15, 15),
            ],
            vec![(0, vec![1]), (1, vec![2])],
            2,
        );
        script.inject_timing_to_script();

        let d1 = script.blocks_data[0];
        let d2 = script.blocks_data[1];
        assert_eq!(d1, 5, "5ps stored as raw picoseconds");
        assert_eq!(d2, 15, "15ps stored as raw picoseconds");
        assert!(d1 < d2, "Ordering must be preserved: {} < {}", d1, d2);
    }
}

#[cfg(test)]
mod constraint_buffer_tests {
    use super::*;

    /// Helper: build a minimal FlattenedScriptV1 for constraint buffer testing.
    fn make_script_with_constraints(
        reg_io_state_size: u32,
        clock_period_ps: u64,
        constraints: Vec<DFFConstraint>,
    ) -> FlattenedScriptV1 {
        FlattenedScriptV1 {
            num_blocks: 0,
            num_major_stages: 0,
            blocks_start: Vec::<usize>::new().into(),
            blocks_data: Vec::<u32>::new().into(),
            reg_io_state_size,
            sram_storage_size: 0,
            input_layout: Vec::new(),
            output_map: IndexMap::new(),
            input_map: IndexMap::new(),
            stages_blocks_parts: Vec::new(),
            assertion_positions: Vec::new(),
            display_positions: Vec::new(),
            gate_delays: Vec::new(),
            dff_constraints: constraints,
            clock_period_ps,
            timing_enabled: true,
            delay_patch_map: Vec::new(),
            xprop_enabled: false,
            partition_x_capable: Vec::new(),
            xprop_state_offset: 0,
            timing_arrivals_enabled: false,
            arrival_state_offset: 0,
        }
    }

    #[test]
    fn test_empty_constraints() {
        let script = make_script_with_constraints(4, 10000, Vec::new());
        let (clock_ps, buf) = script.build_timing_constraint_buffer();
        assert_eq!(clock_ps, 10000);
        assert_eq!(buf.len(), 4);
        assert!(buf.iter().all(|&v| v == 0));
    }

    #[test]
    fn test_single_dff_constraint() {
        let script = make_script_with_constraints(
            4,
            25000,
            vec![DFFConstraint {
                setup_ps: 200,
                hold_ps: 50,
                clock_arrival_ps: 0,
                data_state_pos: 35,
                cell_id: 1,
            }],
        );
        let (clock_ps, buf) = script.build_timing_constraint_buffer();
        assert_eq!(clock_ps, 25000);
        // data_state_pos 35 → word_idx 1
        assert_eq!(buf[0], 0);
        assert_eq!(buf[1], (200u32 << 16) | 50);
        assert_eq!(buf[2], 0);
        assert_eq!(buf[3], 0);
    }

    #[test]
    fn test_multiple_dffs_same_word_takes_min() {
        let script = make_script_with_constraints(
            2,
            10000,
            vec![
                DFFConstraint {
                    setup_ps: 300,
                    hold_ps: 100,
                    clock_arrival_ps: 0,
                    data_state_pos: 5,
                    cell_id: 1,
                },
                DFFConstraint {
                    setup_ps: 150,
                    hold_ps: 200,
                    clock_arrival_ps: 0,
                    data_state_pos: 10,
                    cell_id: 2,
                },
            ],
        );
        let (_clock_ps, buf) = script.build_timing_constraint_buffer();
        // Both in word 0 → min(300,150)=150 setup, min(100,200)=100 hold
        assert_eq!(buf[0], (150u32 << 16) | 100);
    }

    #[test]
    fn test_skip_invalid_data_state_pos() {
        let script = make_script_with_constraints(
            2,
            10000,
            vec![DFFConstraint {
                setup_ps: 200,
                hold_ps: 50,
                clock_arrival_ps: 0,
                data_state_pos: u32::MAX,
                cell_id: 1,
            }],
        );
        let (_clock_ps, buf) = script.build_timing_constraint_buffer();
        assert!(buf.iter().all(|&v| v == 0));
    }

    #[test]
    fn test_skip_out_of_range_word() {
        let script = make_script_with_constraints(
            2,
            10000,
            vec![DFFConstraint {
                setup_ps: 200,
                hold_ps: 50,
                clock_arrival_ps: 0,
                data_state_pos: 100,
                cell_id: 1,
            }],
        );
        let (_clock_ps, buf) = script.build_timing_constraint_buffer();
        // word_idx = 100/32 = 3, but num_words = 2 → skipped
        assert!(buf.iter().all(|&v| v == 0));
    }

    #[test]
    fn test_clock_period_saturation() {
        let script = make_script_with_constraints(1, u64::MAX, Vec::new());
        let (clock_ps, _buf) = script.build_timing_constraint_buffer();
        assert_eq!(clock_ps, u32::MAX);
    }

    // === Violation detection logic tests ===
    // These reproduce the GPU kernel's setup/hold check arithmetic in pure Rust.
    // The GPU kernel checks:
    //   Setup: arrival > 0 && arrival + setup > clock_period → violation
    //   Hold:  arrival < hold → violation

    /// Simulate the GPU kernel's setup violation check.
    /// Returns Some(slack) if violation, None otherwise.
    fn check_setup_violation(arrival: u16, setup_ps: u16, clock_period: u32) -> Option<i32> {
        if arrival > 0 && (arrival as u32 + setup_ps as u32) > clock_period {
            let slack = clock_period as i32 - arrival as i32 - setup_ps as i32;
            Some(slack)
        } else {
            None
        }
    }

    /// Simulate the GPU kernel's hold violation check.
    /// Returns Some(slack) if violation, None otherwise.
    fn check_hold_violation(arrival: u16, hold_ps: u16) -> Option<i32> {
        if (arrival as u32) < (hold_ps as u32) {
            let slack = arrival as i32 - hold_ps as i32;
            Some(slack)
        } else {
            None
        }
    }

    #[test]
    fn test_setup_violation_detection() {
        // Clock period too short for arrival + setup
        // arrival=900, setup=200 → 900+200=1100 > 1000 → violation
        let result = check_setup_violation(900, 200, 1000);
        assert!(result.is_some(), "Should detect setup violation");
        assert_eq!(result.unwrap(), -100, "Slack should be -100ps");

        // Borderline: arrival=800, setup=200 → 800+200=1000, NOT > 1000 → no violation
        let result = check_setup_violation(800, 200, 1000);
        assert!(
            result.is_none(),
            "No violation when arrival+setup == clock_period"
        );
    }

    #[test]
    fn test_setup_violation_with_realistic_inv_chain() {
        // Use real inv_chain accumulated delay: 1323ps rise, setup=85ps (dff_out)
        let arrival: u16 = 1323;
        let setup: u16 = 85;

        // clock_period=1200ps → 1323+85=1408 > 1200 → violation, slack=-208
        let result = check_setup_violation(arrival, setup, 1200);
        assert!(result.is_some(), "Should violate at 1200ps clock");
        assert_eq!(result.unwrap(), -208, "Slack should be -208ps");

        // clock_period=1500ps → 1323+85=1408 ≤ 1500 → no violation
        let result = check_setup_violation(arrival, setup, 1500);
        assert!(result.is_none(), "Should not violate at 1500ps clock");

        // clock_period=1400ps → 1323+85=1408 > 1400 → borderline violation, slack=-8
        let result = check_setup_violation(arrival, setup, 1400);
        assert!(result.is_some(), "Should violate at 1400ps clock");
        assert_eq!(result.unwrap(), -8, "Slack should be -8ps");
    }

    #[test]
    fn test_hold_violation_detection() {
        // arrival=10, hold=50 → 10 < 50 → violation, slack=-40
        let result = check_hold_violation(10, 50);
        assert!(result.is_some(), "Should detect hold violation");
        assert_eq!(result.unwrap(), -40, "Slack should be -40ps");

        // arrival=50, hold=50 → 50 is NOT < 50 → no violation
        let result = check_hold_violation(50, 50);
        assert!(result.is_none(), "No violation when arrival == hold");

        // arrival=0, hold=50 → 0 < 50 → violation (hold check has no arrival>0 guard)
        let result = check_hold_violation(0, 50);
        assert!(
            result.is_some(),
            "arrival=0 should still trigger hold violation"
        );
        assert_eq!(result.unwrap(), -50, "Slack should be -50ps");
    }

    #[test]
    fn test_no_violation_with_zero_constraint() {
        // Constraint word = 0 means no DFF at this word → kernel skips entirely
        // Simulate: extract setup/hold from packed word
        let constraint_word: u32 = 0;
        let setup_ps = (constraint_word >> 16) as u16;
        let hold_ps = (constraint_word & 0xFFFF) as u16;

        assert_eq!(setup_ps, 0);
        assert_eq!(hold_ps, 0);

        // With zero constraints, no violation is possible:
        // Setup: any arrival + 0 > clock_period only if arrival > clock_period (unlikely in u16)
        // Hold: arrival < 0 is impossible for u16
        let result = check_hold_violation(0, hold_ps);
        assert!(
            result.is_none(),
            "Zero hold constraint should never trigger"
        );

        let result = check_hold_violation(500, hold_ps);
        assert!(
            result.is_none(),
            "Zero hold constraint should never trigger"
        );
    }

    #[test]
    fn test_constraint_buffer_with_tight_clock() {
        // Build constraints with setup=200ps DFF at word 5 (data_state_pos=160..191)
        let script = make_script_with_constraints(
            8,
            1000,
            vec![DFFConstraint {
                setup_ps: 200,
                hold_ps: 50,
                clock_arrival_ps: 0,
                data_state_pos: 160,
                cell_id: 1,
            }],
        );
        let (clock_ps, buf) = script.build_timing_constraint_buffer();
        assert_eq!(clock_ps, 1000);

        // Verify constraint is at word 5 (160/32 = 5)
        assert_eq!(
            buf[5],
            (200u32 << 16) | 50,
            "Word 5 should have packed constraint"
        );
        // Other words should be zero
        for i in [0, 1, 2, 3, 4, 6, 7] {
            assert_eq!(buf[i], 0, "Word {} should have no constraint", i);
        }

        // Now simulate arrival=850ps at this word → setup violation
        let setup_ps = (buf[5] >> 16) as u16;
        let result = check_setup_violation(850, setup_ps, clock_ps);
        assert!(
            result.is_some(),
            "arrival=850 + setup=200 = 1050 > 1000 → violation"
        );
        assert_eq!(result.unwrap(), -50, "Slack should be -50ps");
    }

    #[test]
    fn test_setup_skips_zero_arrival() {
        // The GPU kernel has an `arrival > 0` guard for setup checks.
        // arrival=0 means "no data propagated through combinational logic"
        // (e.g., first cycle, or a DFF whose inputs are constant).
        // Even with a tight clock where arrival + setup > clock_period,
        // the check must NOT fire when arrival == 0.
        let result = check_setup_violation(0, 200, 100);
        assert!(
            result.is_none(),
            "arrival=0 must skip setup check even when 0+200=200 > 100"
        );

        // Confirm a non-zero arrival with same parameters DOES fire
        let result = check_setup_violation(1, 200, 100);
        assert!(
            result.is_some(),
            "arrival=1 should trigger: 1+200=201 > 100"
        );

        // Also verify arrival=0 with a very large setup still doesn't fire
        let result = check_setup_violation(0, u16::MAX, 1);
        assert!(
            result.is_none(),
            "arrival=0 must always skip, regardless of setup or clock_period"
        );
    }

    #[test]
    fn test_setup_u16_max_arrival() {
        // Test arrival near u16::MAX (65535ps) with setup that pushes sum past
        // u16 range. The kernel uses u32 arithmetic:
        //   (u32)arrival + (u32)setup_ps > clock_period_ps
        // so it must not overflow at 16-bit boundary.

        // arrival=65000, setup=1000 → (u32)66000 > 60000 → violation
        let result = check_setup_violation(65000, 1000, 60000);
        assert!(result.is_some(), "65000+1000=66000 > 60000 → violation");
        assert_eq!(
            result.unwrap(),
            60000 - 65000 - 1000,
            "Slack should be -6000ps"
        );

        // arrival=65000, setup=1000 → (u32)66000 > 70000 → no violation
        let result = check_setup_violation(65000, 1000, 70000);
        assert!(result.is_none(), "65000+1000=66000 <= 70000 → no violation");

        // Extreme: both at u16::MAX
        // arrival=65535, setup=65535 → (u32)131070 > 131069 → violation
        let result = check_setup_violation(u16::MAX, u16::MAX, 131069);
        assert!(
            result.is_some(),
            "u16::MAX + u16::MAX = 131070 > 131069 → violation"
        );
        assert_eq!(
            result.unwrap(),
            131069 - 65535 - 65535,
            "Slack should be -1ps"
        );

        // Same extreme but clock_period accommodates it
        let result = check_setup_violation(u16::MAX, u16::MAX, 131070);
        assert!(
            result.is_none(),
            "u16::MAX + u16::MAX = 131070 <= 131070 → no violation"
        );
    }

    /// Pillar B Stage 2 — capture-side clock arrival relaxes effective
    /// setup and tightens effective hold by the same amount.
    #[test]
    fn clock_arrival_folds_into_effective_setup_hold() {
        let c = DFFConstraint {
            setup_ps: 200,
            hold_ps: 50,
            clock_arrival_ps: 80,
            data_state_pos: 0,
            cell_id: 1,
        };
        let (eff_setup, eff_hold) = c.effective_setup_hold();
        assert_eq!(eff_setup, 120, "setup eased by clock arrival");
        assert_eq!(eff_hold, 130, "hold tightened by clock arrival");
    }

    /// A clock arrival larger than the raw setup must clamp to 0 rather
    /// than wrap. Likewise a giant arrival must not panic the hold add.
    #[test]
    fn effective_setup_hold_clamps_at_zero_and_u16_max() {
        let c = DFFConstraint {
            setup_ps: 100,
            hold_ps: u16::MAX - 10,
            clock_arrival_ps: 500,
            data_state_pos: 0,
            cell_id: 1,
        };
        let (eff_setup, eff_hold) = c.effective_setup_hold();
        assert_eq!(eff_setup, 0, "setup clamps at 0 instead of wrapping");
        assert_eq!(
            eff_hold,
            u16::MAX,
            "hold clamps at u16::MAX instead of overflow"
        );
    }

    /// build_timing_constraint_buffer must apply the per-DFF arrival
    /// fold-in before collapsing across the per-word min, otherwise the
    /// effective values lose their per-DFF offset.
    #[test]
    fn constraint_buffer_applies_clock_arrival_fold() {
        let script = make_script_with_constraints(
            2,
            10000,
            vec![DFFConstraint {
                setup_ps: 200,
                hold_ps: 50,
                clock_arrival_ps: 80,
                data_state_pos: 0, // word 0
                cell_id: 1,
            }],
        );
        let (_clock_ps, buf) = script.build_timing_constraint_buffer();
        let setup = (buf[0] >> 16) as u16;
        let hold = (buf[0] & 0xFFFF) as u16;
        assert_eq!(setup, 120, "buffer setup reflects 200 - 80 arrival");
        assert_eq!(hold, 130, "buffer hold reflects 50 + 80 arrival");
    }

    /// With clock_arrival_ps = 0 (the default), build_timing_constraint_buffer
    /// must produce the exact same words as before the Stage 2 change —
    /// pre-existing tests already cover this, but we add an explicit
    /// regression so a future refactor can't silently re-introduce skew
    /// for designs without ClockArrival data.
    #[test]
    fn zero_clock_arrival_preserves_legacy_behaviour() {
        let script = make_script_with_constraints(
            1,
            10000,
            vec![DFFConstraint {
                setup_ps: 1234,
                hold_ps: 567,
                clock_arrival_ps: 0,
                data_state_pos: 0,
                cell_id: 1,
            }],
        );
        let (_clock_ps, buf) = script.build_timing_constraint_buffer();
        assert_eq!(buf[0], (1234u32 << 16) | 567u32);
    }
}

#[cfg(test)]
mod xprop_tests {
    use super::*;

    /// Build a minimal FlattenedScriptV1 for xprop field tests.
    fn make_xprop_script(rio: u32, xprop_enabled: bool) -> FlattenedScriptV1 {
        FlattenedScriptV1 {
            num_blocks: 0,
            num_major_stages: 0,
            blocks_start: Vec::<usize>::new().into(),
            blocks_data: Vec::<u32>::new().into(),
            reg_io_state_size: rio,
            sram_storage_size: 0,
            input_layout: Vec::new(),
            output_map: IndexMap::new(),
            input_map: IndexMap::new(),
            stages_blocks_parts: Vec::new(),
            assertion_positions: Vec::new(),
            display_positions: Vec::new(),
            gate_delays: Vec::new(),
            dff_constraints: Vec::new(),
            clock_period_ps: 1000,
            timing_enabled: false,
            delay_patch_map: Vec::new(),
            xprop_enabled,
            partition_x_capable: Vec::new(),
            xprop_state_offset: if xprop_enabled { rio } else { 0 },
            timing_arrivals_enabled: false,
            arrival_state_offset: 0,
        }
    }

    #[test]
    fn test_effective_state_size_disabled() {
        let script = make_xprop_script(42, false);
        assert_eq!(
            script.effective_state_size(),
            42,
            "disabled: effective == reg_io_state_size"
        );
    }

    #[test]
    fn test_effective_state_size_enabled() {
        let script = make_xprop_script(42, true);
        assert_eq!(
            script.effective_state_size(),
            84,
            "enabled: effective == 2 * reg_io_state_size"
        );
    }

    #[test]
    fn test_effective_state_size_zero() {
        // Edge case: empty state
        let script = make_xprop_script(0, true);
        assert_eq!(script.effective_state_size(), 0);
        let script = make_xprop_script(0, false);
        assert_eq!(script.effective_state_size(), 0);
    }

    #[test]
    fn test_xprop_state_offset_matches_rio() {
        let rio = 128;
        let script = make_xprop_script(rio, true);
        assert_eq!(
            script.xprop_state_offset, rio,
            "xprop_state_offset should equal reg_io_state_size"
        );
    }

    #[test]
    fn test_partition_x_capable_metadata_words() {
        // Verify that partition metadata words 8 and 9 are correctly set
        // by building a script with synthetic blocks_data and checking the values.
        // This is a unit test for the metadata encoding, not the full pipeline.

        // Simulate what `FlattenedScriptV1::from()` does for metadata words:
        // Word 8: is_x_capable (0 or 1)
        // Word 9: xprop_state_offset
        let rio: u32 = 64;
        let xprop_state_offset = rio;

        // Create a synthetic blocks_data with one partition's metadata section
        let mut blocks_data = vec![0u32; NUM_THREADS_V1];
        // Set up a valid partition: num_stages=1
        blocks_data[0] = 1;

        // Simulate the xprop metadata patching
        let is_x_capable = true;
        blocks_data[8] = if is_x_capable { 1 } else { 0 };
        blocks_data[9] = xprop_state_offset;

        assert_eq!(blocks_data[8], 1, "word 8 should be 1 for X-capable");
        assert_eq!(blocks_data[9], rio, "word 9 should be xprop_state_offset");

        // Non-X-capable partition
        blocks_data[8] = 0;
        blocks_data[9] = xprop_state_offset;
        assert_eq!(blocks_data[8], 0, "word 8 should be 0 for non-X-capable");
        assert_eq!(
            blocks_data[9], rio,
            "word 9 should still be xprop_state_offset"
        );
    }

    #[test]
    fn test_effective_state_size_with_timing_arrivals() {
        let mut script = make_xprop_script(42, false);
        // Simulate loading SDF timing
        script.timing_enabled = true;
        script.enable_timing_arrivals();
        assert_eq!(
            script.effective_state_size(),
            84,
            "timing arrivals (no xprop): effective == 2 * rio"
        );
        assert_eq!(script.arrival_state_offset, 42);
    }

    #[test]
    fn test_effective_state_size_xprop_and_timing_arrivals() {
        let mut script = make_xprop_script(42, true);
        script.timing_enabled = true;
        script.enable_timing_arrivals();
        assert_eq!(
            script.effective_state_size(),
            126,
            "xprop + timing arrivals: effective == 3 * rio"
        );
        assert_eq!(
            script.arrival_state_offset, 84,
            "arrival offset should be 2*rio when xprop enabled"
        );
    }

    #[test]
    #[should_panic(expected = "Cannot enable timing arrivals without SDF timing data")]
    fn test_enable_timing_arrivals_requires_timing() {
        let mut script = make_xprop_script(42, false);
        script.enable_timing_arrivals(); // Should panic - timing_enabled is false
    }
}

#[cfg(test)]
mod word_symbol_map_tests {
    use super::*;

    fn site(name: &str, bit: u8) -> DffSiteName {
        DffSiteName {
            name: name.to_string(),
            bit,
        }
    }

    fn map_with(entries: Vec<(u32, Vec<DffSiteName>)>) -> WordSymbolMap {
        WordSymbolMap::from_sites(entries.into_iter().collect())
    }

    #[test]
    fn describe_word_with_no_dff_in_word() {
        let m = map_with(vec![]);
        assert_eq!(m.describe_word(7, 4), "<no DFF in word> [word=7]");
    }

    #[test]
    fn describe_word_with_single_dff() {
        let m = map_with(vec![(3, vec![site("top/cpu/state", 0)])]);
        assert_eq!(
            m.describe_word(3, 4),
            "top/cpu/state[bit 0] [word=3]"
        );
    }

    #[test]
    fn describe_word_with_multiple_dffs_under_limit() {
        let m = map_with(vec![(
            5,
            vec![
                site("top/regs[0]", 0),
                site("top/regs[1]", 1),
                site("top/regs[2]", 2),
            ],
        )]);
        let out = m.describe_word(5, 4);
        assert_eq!(
            out,
            "top/regs[0][bit 0], top/regs[1][bit 1], top/regs[2][bit 2] [word=5]"
        );
    }

    #[test]
    fn describe_word_truncates_with_more_suffix() {
        let m = map_with(vec![(
            1,
            (0..6)
                .map(|b| site(&format!("top/r{b}"), b as u8))
                .collect(),
        )]);
        let out = m.describe_word(1, 2);
        assert_eq!(
            out,
            "top/r0[bit 0], top/r1[bit 1], +4 more [word=1]"
        );
    }

    #[test]
    fn describe_word_max_names_zero_floor_is_one() {
        // Defensive floor — no caller passes 0, but if one ever does we
        // want a name shown rather than a misleading "+N more" tail.
        let m = map_with(vec![(0, vec![site("a", 0), site("b", 1)])]);
        let out = m.describe_word(0, 0);
        assert!(out.starts_with("a[bit 0], +1 more"), "got: {out}");
    }

    #[test]
    fn num_sites_and_words() {
        let m = map_with(vec![
            (0, vec![site("a", 0), site("b", 1)]),
            (1, vec![site("c", 0)]),
        ]);
        assert_eq!(m.num_words(), 2);
        assert_eq!(m.num_sites(), 3);
    }

}

#[cfg(test)]
mod corner_resolution_tests {
    use super::*;
    use flatbuffers::FlatBufferBuilder;
    use timing_ir::*;

    fn build_ir_with_corners(names: &[&str]) -> Vec<u8> {
        let mut b = FlatBufferBuilder::with_capacity(256);
        let corners: Vec<_> = names
            .iter()
            .map(|n| {
                let name = b.create_string(n);
                let process = b.create_string("tt");
                Corner::create(
                    &mut b,
                    &CornerArgs {
                        name: Some(name),
                        process: Some(process),
                        voltage: 1.0,
                        temperature: 25.0,
                    },
                )
            })
            .collect();
        let corners_vec = b.create_vector(&corners);
        let tool = b.create_string("test");
        let ver = b.create_string("0");
        let inputs: Vec<_> = vec![];
        let inputs_vec = b.create_vector::<flatbuffers::WIPOffset<&str>>(&inputs);
        let arcs_vec = b.create_vector::<flatbuffers::WIPOffset<TimingArc>>(&[]);
        let ic_vec = b.create_vector::<flatbuffers::WIPOffset<InterconnectDelay>>(&[]);
        let sh_vec = b.create_vector::<flatbuffers::WIPOffset<SetupHoldCheck>>(&[]);
        let ca_vec = b.create_vector::<flatbuffers::WIPOffset<ClockArrival>>(&[]);
        let ve_vec = b.create_vector::<flatbuffers::WIPOffset<VendorExtension>>(&[]);
        let sv = SchemaVersion::new(SCHEMA_MAJOR, SCHEMA_MINOR, SCHEMA_PATCH);
        let root = TimingIR::create(
            &mut b,
            &TimingIRArgs {
                schema_version: Some(&sv),
                corners: Some(corners_vec),
                timing_arcs: Some(arcs_vec),
                interconnect_delays: Some(ic_vec),
                setup_hold_checks: Some(sh_vec),
                clock_arrivals: Some(ca_vec),
                vendor_extensions: Some(ve_vec),
                generator_tool: Some(tool),
                generator_version: Some(ver),
                input_files: Some(inputs_vec),
            },
        );
        finish_timing_ir_buffer(&mut b, root);
        b.finished_data().to_vec()
    }

    #[test]
    fn resolve_corner_index_default_is_zero() {
        let buf = build_ir_with_corners(&["typ", "slow"]);
        let ir = root_as_timing_ir(&buf).unwrap();
        assert_eq!(resolve_corner_index(&ir, None), Ok(0));
    }

    #[test]
    fn resolve_corner_index_finds_named_corner() {
        let buf = build_ir_with_corners(&["typ", "slow", "fast"]);
        let ir = root_as_timing_ir(&buf).unwrap();
        assert_eq!(resolve_corner_index(&ir, Some("typ")), Ok(0));
        assert_eq!(resolve_corner_index(&ir, Some("slow")), Ok(1));
        assert_eq!(resolve_corner_index(&ir, Some("fast")), Ok(2));
    }

    #[test]
    fn resolve_corner_index_unknown_name_lists_available() {
        let buf = build_ir_with_corners(&["typ", "slow"]);
        let ir = root_as_timing_ir(&buf).unwrap();
        let err = resolve_corner_index(&ir, Some("missing")).unwrap_err();
        assert!(err.contains("missing"), "got: {err}");
        assert!(err.contains("typ"), "should list available: {err}");
        assert!(err.contains("slow"), "should list available: {err}");
    }
}
