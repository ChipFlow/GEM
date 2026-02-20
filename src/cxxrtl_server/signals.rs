// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Signal reference registry for the CXXRTL protocol.
//!
//! Manages `reference_items` bindings that map client-provided reference IDs
//! to sets of signal locations in the simulation state buffer.

use std::collections::HashMap;

use super::design::ResolvedSignal;
use super::protocol::ItemDesignation;

/// A bound reference: a set of signals that can be queried together.
#[derive(Debug, Clone)]
pub struct BoundReference {
    /// The signal entries in this reference, in order.
    pub entries: Vec<BoundEntry>,
    /// Total number of u32 words needed to encode all entries.
    pub total_u32_words: usize,
}

/// A single entry within a bound reference.
#[derive(Debug, Clone)]
pub struct BoundEntry {
    /// Width of this signal in bits.
    pub width: u32,
    /// Bit positions in the state buffer (one per bit, LSB first).
    /// `u32::MAX` means the bit is tied to a constant.
    pub state_bit_positions: Vec<u32>,
}

/// The signal registry, managing named references.
pub struct SignalRegistry {
    /// All resolved signals from the design, keyed by their full CXXRTL name.
    signal_map: HashMap<String, ResolvedSignal>,
    /// Active references.
    references: HashMap<String, BoundReference>,
}

impl SignalRegistry {
    /// Create a new registry from the resolved design signals.
    pub fn new(signals: Vec<ResolvedSignal>) -> Self {
        let mut signal_map = HashMap::new();
        for sig in signals {
            signal_map.insert(sig.name.clone(), sig);
        }
        SignalRegistry {
            signal_map,
            references: HashMap::new(),
        }
    }

    /// Bind or unbind a reference.
    ///
    /// If `items` is `None`, the reference is deallocated.
    /// If `items` is `Some(...)`, the reference is (re-)bound to those signals.
    pub fn reference_items(
        &mut self,
        reference: &str,
        items: Option<&[ItemDesignation]>,
    ) -> Result<(), String> {
        match items {
            None => {
                self.references.remove(reference);
                Ok(())
            }
            Some(designations) => {
                let mut entries = Vec::new();
                let mut total_words = 0usize;

                for designation in designations {
                    match designation {
                        ItemDesignation::Node(name) => {
                            let sig = self
                                .signal_map
                                .get(name)
                                .ok_or_else(|| format!("unknown item: {}", name))?;
                            let words = (sig.width as usize + 31) / 32;
                            total_words += words;
                            entries.push(BoundEntry {
                                width: sig.width,
                                state_bit_positions: sig.state_bit_positions.clone(),
                            });
                        }
                        ItemDesignation::MemoryRows(name, _first, _last) => {
                            // Memory row queries are not yet supported
                            return Err(format!(
                                "memory row queries not yet supported for: {}",
                                name
                            ));
                        }
                    }
                }

                self.references.insert(
                    reference.to_string(),
                    BoundReference {
                        entries,
                        total_u32_words: total_words,
                    },
                );
                Ok(())
            }
        }
    }

    /// Get a bound reference by name.
    pub fn get_reference(&self, reference: &str) -> Option<&BoundReference> {
        self.references.get(reference)
    }

    /// Check if a signal name exists.
    pub fn has_signal(&self, name: &str) -> bool {
        self.signal_map.contains_key(name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_signals() -> Vec<ResolvedSignal> {
        vec![
            ResolvedSignal {
                name: "clk".to_string(),
                width: 1,
                state_bit_positions: vec![0],
                is_input: true,
                is_output: false,
            },
            ResolvedSignal {
                name: "data".to_string(),
                width: 8,
                state_bit_positions: vec![1, 2, 3, 4, 5, 6, 7, 8],
                is_input: false,
                is_output: true,
            },
        ]
    }

    #[test]
    fn test_bind_and_query() {
        let mut registry = SignalRegistry::new(make_test_signals());
        registry
            .reference_items(
                "ref1",
                Some(&[
                    ItemDesignation::Node("clk".to_string()),
                    ItemDesignation::Node("data".to_string()),
                ]),
            )
            .unwrap();

        let bound = registry.get_reference("ref1").unwrap();
        assert_eq!(bound.entries.len(), 2);
        assert_eq!(bound.entries[0].width, 1);
        assert_eq!(bound.entries[1].width, 8);
        // clk = 1 bit → 1 word, data = 8 bits → 1 word
        assert_eq!(bound.total_u32_words, 2);
    }

    #[test]
    fn test_deallocate() {
        let mut registry = SignalRegistry::new(make_test_signals());
        registry
            .reference_items("ref1", Some(&[ItemDesignation::Node("clk".to_string())]))
            .unwrap();
        assert!(registry.get_reference("ref1").is_some());

        registry.reference_items("ref1", None).unwrap();
        assert!(registry.get_reference("ref1").is_none());
    }

    #[test]
    fn test_unknown_signal() {
        let mut registry = SignalRegistry::new(make_test_signals());
        let result =
            registry.reference_items("ref1", Some(&[ItemDesignation::Node("nope".to_string())]));
        assert!(result.is_err());
    }
}
