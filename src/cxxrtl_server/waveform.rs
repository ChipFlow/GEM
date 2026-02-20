// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Waveform query implementation for the CXXRTL protocol.
//!
//! Reads signal values from the simulation state buffer and encodes them
//! as base64(u32) per the protocol specification.

use super::protocol::{Sample, TimePoint};
use super::signals::BoundReference;

/// Extract a single bit from the state buffer.
///
/// `state_slice` is the u32 slice for a single cycle's state.
/// `bit_pos` is the absolute bit position within that slice.
/// Returns 0 or 1.
fn read_state_bit(state_slice: &[u32], bit_pos: u32) -> u8 {
    if bit_pos == u32::MAX {
        return 0; // tied constant
    }
    let word_idx = (bit_pos >> 5) as usize;
    let bit_idx = bit_pos & 31;
    if word_idx < state_slice.len() {
        ((state_slice[word_idx] >> bit_idx) & 1) as u8
    } else {
        0
    }
}

/// Encode signal values for a bound reference at a given cycle.
///
/// Returns a base64-encoded string of packed u32 little-endian words,
/// one word per signal (padded to 32-bit boundary).
pub fn encode_item_values(
    bound_ref: &BoundReference,
    state_slice: &[u32],
) -> String {
    let mut words: Vec<u32> = Vec::with_capacity(bound_ref.total_u32_words);

    for entry in &bound_ref.entries {
        let num_words = ((entry.width as usize) + 31) / 32;
        let mut entry_words = vec![0u32; num_words];

        for (bit_i, &bit_pos) in entry.state_bit_positions.iter().enumerate() {
            let val = read_state_bit(state_slice, bit_pos);
            if val != 0 {
                let word_i = bit_i / 32;
                let bit_in_word = bit_i % 32;
                if word_i < entry_words.len() {
                    entry_words[word_i] |= 1 << bit_in_word;
                }
            }
        }

        words.extend_from_slice(&entry_words);
    }

    // Convert to little-endian bytes and base64 encode
    let bytes: Vec<u8> = words
        .iter()
        .flat_map(|w| w.to_le_bytes())
        .collect();

    base64_encode(&bytes)
}

/// Simple base64 encoder (RFC 4648, no padding).
///
/// We use standard base64 with padding as the protocol specifies RFC 4648.
fn base64_encode(data: &[u8]) -> String {
    const ALPHABET: &[u8; 64] =
        b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    let mut result = String::with_capacity((data.len() + 2) / 3 * 4);
    let chunks = data.chunks(3);

    for chunk in chunks {
        let mut buf = [0u8; 3];
        buf[..chunk.len()].copy_from_slice(chunk);

        let b0 = buf[0] as usize;
        let b1 = buf[1] as usize;
        let b2 = buf[2] as usize;

        result.push(ALPHABET[(b0 >> 2) & 0x3F] as char);
        result.push(ALPHABET[((b0 << 4) | (b1 >> 4)) & 0x3F] as char);

        if chunk.len() > 1 {
            result.push(ALPHABET[((b1 << 2) | (b2 >> 6)) & 0x3F] as char);
        } else {
            result.push('=');
        }

        if chunk.len() > 2 {
            result.push(ALPHABET[b2 & 0x3F] as char);
        } else {
            result.push('=');
        }
    }

    result
}

/// Query waveform data for a time interval.
///
/// - `states`: the full state buffer (all cycles concatenated)
/// - `state_size`: number of u32 words per cycle
/// - `offsets_timestamps`: (offset, timestamp) pairs for each simulated cycle
/// - `timescale_fs`: femtoseconds per VCD timescale unit
/// - `bound_ref`: the signal reference to query
/// - `begin`/`end`: the requested time interval
/// - `collapse`: if true, emit one sample per distinct timestamp
///
/// Returns a list of samples within the interval.
pub fn query_interval(
    states: &[u32],
    state_size: u32,
    offsets_timestamps: &[(usize, u64)],
    timescale_fs: u64,
    bound_ref: Option<&BoundReference>,
    begin: &TimePoint,
    end: &TimePoint,
    collapse: bool,
) -> Vec<Sample> {
    let begin_fs = begin.to_femtoseconds().unwrap_or(0);
    let end_fs = end.to_femtoseconds().unwrap_or(u64::MAX);

    let mut samples = Vec::new();
    let mut last_timestamp = None;

    // Cycle 0 is the initial state (time 0.0)
    // Cycle i+1 is the state after processing offsets_timestamps[i]
    let total_cycles = offsets_timestamps.len() + 1;

    for cycle_i in 0..total_cycles {
        let timestamp_fs = if cycle_i == 0 {
            0u64
        } else {
            offsets_timestamps[cycle_i - 1].1.saturating_mul(timescale_fs)
        };

        if timestamp_fs < begin_fs || timestamp_fs > end_fs {
            continue;
        }

        // Collapse: skip if same timestamp as previous
        if collapse {
            if last_timestamp == Some(timestamp_fs) {
                continue;
            }
            last_timestamp = Some(timestamp_fs);
        }

        let time = TimePoint::from_timestamp(
            if cycle_i == 0 { 0 } else { offsets_timestamps[cycle_i - 1].1 },
            timescale_fs,
        );

        let item_values = bound_ref.map(|br| {
            let state_offset = cycle_i * state_size as usize;
            let state_end = state_offset + state_size as usize;
            if state_end <= states.len() {
                encode_item_values(br, &states[state_offset..state_end])
            } else {
                // Out of range: return zeros
                encode_item_values(br, &[])
            }
        });

        samples.push(Sample {
            time: time.0,
            item_values,
            diagnostics: None, // Diagnostics handled separately in Phase 4
        });
    }

    samples
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cxxrtl_server::signals::BoundEntry;

    #[test]
    fn test_read_state_bit() {
        let state = [0b1010_1010u32, 0b1111_0000];
        assert_eq!(read_state_bit(&state, 0), 0);
        assert_eq!(read_state_bit(&state, 1), 1);
        assert_eq!(read_state_bit(&state, 7), 1);
        assert_eq!(read_state_bit(&state, 32), 0);
        assert_eq!(read_state_bit(&state, 36), 1);
        assert_eq!(read_state_bit(&state, u32::MAX), 0);
    }

    #[test]
    fn test_base64_encode() {
        // Test vectors from RFC 4648
        assert_eq!(base64_encode(b""), "");
        assert_eq!(base64_encode(b"f"), "Zg==");
        assert_eq!(base64_encode(b"fo"), "Zm8=");
        assert_eq!(base64_encode(b"foo"), "Zm9v");
    }

    #[test]
    fn test_encode_item_values_single_bit() {
        let bound = BoundReference {
            entries: vec![BoundEntry {
                width: 1,
                state_bit_positions: vec![1],
            }],
            total_u32_words: 1,
        };
        // State where bit 1 is set
        let state = [0b10u32];
        let encoded = encode_item_values(&bound, &state);
        // Should encode u32 value 1 (bit 0 of output word set because bit 1 of state is set)
        let bytes = 1u32.to_le_bytes();
        assert_eq!(encoded, base64_encode(&bytes));
    }

    #[test]
    fn test_encode_item_values_multi_bit() {
        let bound = BoundReference {
            entries: vec![BoundEntry {
                width: 4,
                state_bit_positions: vec![0, 1, 2, 3],
            }],
            total_u32_words: 1,
        };
        let state = [0b1010u32]; // bits 1 and 3 set
        let encoded = encode_item_values(&bound, &state);
        // Output word should be 0b1010 = 10
        let bytes = 0b1010u32.to_le_bytes();
        assert_eq!(encoded, base64_encode(&bytes));
    }
}
