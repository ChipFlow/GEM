// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Design hierarchy browsing for the CXXRTL protocol.
//!
//! Builds scope trees and item lists from the NetlistDB and AIG/script
//! signal maps. The CXXRTL protocol uses space-separated scope paths
//! (e.g. `"top cpu alu"`) while the netlist uses `/`-separated HierNames.

use std::collections::{HashMap, HashSet};

use netlistdb::{Direction, NetlistDB};

use crate::aig::AIG;
use crate::flatten::FlattenedScriptV1;

use super::protocol::{ItemInfo, ScopeDefinition, ScopeInfo, ScopeInstantiation};

/// Scope separator used in CXXRTL protocol (U+0020 space).
const SCOPE_SEP: char = ' ';

/// Extract all unique scope paths from the netlist cell hierarchy.
///
/// Every cell's parent hierarchy becomes a scope. The root scope is `""`.
pub fn extract_scopes(netlistdb: &NetlistDB) -> HashMap<String, ScopeInfo> {
    let mut scopes = HashMap::new();

    // Root scope
    scopes.insert(
        String::new(),
        ScopeInfo {
            scope_type: "module".to_string(),
            definition: Some(ScopeDefinition {
                src: None,
                name: Some(netlistdb.name.to_string()),
                attributes: HashMap::new(),
            }),
            instantiation: None,
        },
    );

    // Walk all cells and extract their hierarchy paths as scopes
    let mut seen_scopes: HashSet<String> = HashSet::new();
    seen_scopes.insert(String::new());

    for cell_id in 1..netlistdb.num_cells {
        let cellname = &netlistdb.cellnames[cell_id];
        let celltype = &netlistdb.celltypes[cell_id];

        // The cell hierarchy gives us the scope path.
        // For a cell at path a/b/c, the scopes are "", "a", "a b", "a b c"
        // (where a/b/c is the cell instance, not a scope itself for leaf cells,
        // but its parent a/b is a scope).
        let parts: Vec<&str> = cellname.iter().map(|s| s.as_str()).collect();
        // parts is bottom-up: [c, b, a], reverse to [a, b, c]
        let parts_topdown: Vec<&str> = parts.into_iter().rev().collect();

        // The cell itself (e.g. "a b c") — for non-leaf cells this could
        // be a scope. For leaf cells, we register parent scopes only.
        // Register all ancestor scopes.
        for depth in 1..parts_topdown.len() {
            let scope_path = parts_topdown[..depth].join(&SCOPE_SEP.to_string());
            if seen_scopes.insert(scope_path.clone()) {
                let inst_name = parts_topdown[depth - 1];
                scopes.insert(
                    scope_path,
                    ScopeInfo {
                        scope_type: "module".to_string(),
                        definition: Some(ScopeDefinition {
                            src: None,
                            name: None,
                            attributes: HashMap::new(),
                        }),
                        instantiation: Some(ScopeInstantiation {
                            src: None,
                            attributes: {
                                let mut attrs = HashMap::new();
                                attrs.insert(
                                    "instance_name".to_string(),
                                    serde_json::Value::String(inst_name.to_string()),
                                );
                                attrs
                            },
                        }),
                    },
                );
            }
        }

        // If this is a non-leaf cell (has sub-instances), register the full path too.
        // For a flat netlist all cells are leaf, so we only register parent scopes.
        // The full cell path is registered as a scope with its celltype.
        let full_path = parts_topdown.join(&SCOPE_SEP.to_string());
        if !full_path.is_empty() && seen_scopes.insert(full_path.clone()) {
            scopes.insert(
                full_path,
                ScopeInfo {
                    scope_type: "module".to_string(),
                    definition: Some(ScopeDefinition {
                        src: None,
                        name: Some(celltype.to_string()),
                        attributes: HashMap::new(),
                    }),
                    instantiation: Some(ScopeInstantiation {
                        src: None,
                        attributes: HashMap::new(),
                    }),
                },
            );
        }
    }

    scopes
}

/// Resolved signal information: maps a signal name to its state buffer location.
#[derive(Debug, Clone)]
pub struct ResolvedSignal {
    /// Signal name as it appears in the CXXRTL item list.
    pub name: String,
    /// Bit width of this signal.
    pub width: u32,
    /// Bit positions in the state buffer (one per bit, LSB first).
    /// Each entry is the absolute bit position in a cycle's state slice.
    pub state_bit_positions: Vec<u32>,
    /// Whether this is an input port.
    pub is_input: bool,
    /// Whether this is an output port.
    pub is_output: bool,
}

/// Build the item list for the CXXRTL protocol.
///
/// This extracts all observable signals from the design — primary I/O ports
/// and flip-flop outputs — and maps them to their positions in the state
/// buffer.
///
/// Returns a map from scope path to (item_name → ItemInfo), and a flat
/// map of all resolved signals for use by the signal registry.
pub fn extract_items(
    netlistdb: &NetlistDB,
    aig: &AIG,
    script: &FlattenedScriptV1,
) -> (HashMap<String, HashMap<String, ItemInfo>>, Vec<ResolvedSignal>) {
    let mut items_by_scope: HashMap<String, HashMap<String, ItemInfo>> = HashMap::new();
    let mut resolved_signals = Vec::new();

    // Group top-module pins by port name to reconstruct bus widths.
    // pinnames for top-module pins (cell_id=0) have the form (HierName::empty(), pin_type, bus_id).
    let mut port_groups: HashMap<String, Vec<(usize, Option<isize>)>> = HashMap::new();

    for pin_id in netlistdb.cell2pin.iter_set(0) {
        let (ref _hier, ref pin_type, bus_id) = netlistdb.pinnames[pin_id];
        let port_name = pin_type.to_string();
        port_groups
            .entry(port_name)
            .or_default()
            .push((pin_id, bus_id));
    }

    // Process each port group
    for (port_name, mut pins) in port_groups {
        // Sort by bus index (None first, then ascending)
        pins.sort_by_key(|(_, bus_id)| bus_id.unwrap_or(0));

        let width = pins.len() as u32;
        let lsb_at = pins
            .first()
            .and_then(|(_, bus_id)| *bus_id)
            .unwrap_or(0) as i32;

        let mut is_input = false;
        let mut is_output = false;
        let mut bit_positions = Vec::new();

        for &(pin_id, _) in &pins {
            let dir = &netlistdb.pindirect[pin_id];
            match dir {
                Direction::O => {
                    // Top-module output port = signal driven externally (input to design)
                    is_input = true;
                }
                Direction::I => {
                    // Top-module input pin = output from design perspective
                    is_output = true;
                }
                _ => {}
            }

            let aigpin_iv = aig.pin2aigpin_iv[pin_id];
            let aigpin = aigpin_iv >> 1;

            // Find this signal's position in the state buffer
            if let Some(&state_pos) = script.output_map.get(&aigpin_iv) {
                bit_positions.push(state_pos);
            } else if let Some(&state_pos) = script.input_map.get(&aigpin) {
                bit_positions.push(state_pos);
            } else if aigpin <= 1 {
                // Tied to constant 0 or 1
                bit_positions.push(u32::MAX);
            } else {
                // Signal not observable in state buffer
                bit_positions.push(u32::MAX);
            }
        }

        let scope_path = String::new(); // top-module ports are in root scope
        let items = items_by_scope.entry(scope_path.clone()).or_default();
        items.insert(
            port_name.clone(),
            ItemInfo {
                item_type: "node".to_string(),
                width,
                lsb_at,
                depth: None,
                zero_at: None,
                settable: false,
                input: Some(is_input),
                output: Some(is_output),
                attributes: HashMap::new(),
            },
        );

        resolved_signals.push(ResolvedSignal {
            name: port_name,
            width,
            state_bit_positions: bit_positions,
            is_input,
            is_output,
        });
    }

    // Add DFF outputs as observable signals.
    // DFFs are indexed by cell_id in aig.dffs.
    for (&cell_id, dff) in &aig.dffs {
        let cellname = &netlistdb.cellnames[cell_id];
        let parts: Vec<&str> = cellname.iter().map(|s| s.as_str()).collect();
        let parts_topdown: Vec<&str> = parts.into_iter().rev().collect();

        let (scope_path, item_name) = if parts_topdown.is_empty() {
            (String::new(), format!("dff_{}", cell_id))
        } else if parts_topdown.len() == 1 {
            (String::new(), parts_topdown[0].to_string())
        } else {
            let scope = parts_topdown[..parts_topdown.len() - 1].join(&SCOPE_SEP.to_string());
            let name = parts_topdown.last().unwrap().to_string();
            (scope, name)
        };

        // The Q output of the DFF
        let q_aigpin = dff.q;
        if let Some(&state_pos) = script.input_map.get(&q_aigpin) {
            let item_name_q = format!("{}_Q", item_name);
            let items = items_by_scope.entry(scope_path.clone()).or_default();
            items.insert(
                item_name_q.clone(),
                ItemInfo {
                    item_type: "node".to_string(),
                    width: 1,
                    lsb_at: 0,
                    depth: None,
                    zero_at: None,
                    settable: false,
                    input: Some(false),
                    output: Some(false),
                    attributes: HashMap::new(),
                },
            );

            resolved_signals.push(ResolvedSignal {
                name: format!("{}{}{}", scope_path, if scope_path.is_empty() { "" } else { " " }, item_name_q),
                width: 1,
                state_bit_positions: vec![state_pos],
                is_input: false,
                is_output: false,
            });
        }
    }

    // Add SRAM blocks as memory items.
    // RAMBlock has fixed 32-bit data width and 2^AIGPDK_SRAM_ADDR_WIDTH depth.
    for (&cell_id, _ram) in &aig.srams {
        let cellname = &netlistdb.cellnames[cell_id];
        let parts: Vec<&str> = cellname.iter().map(|s| s.as_str()).collect();
        let parts_topdown: Vec<&str> = parts.into_iter().rev().collect();

        let (scope_path, item_name) = if parts_topdown.is_empty() {
            (String::new(), format!("sram_{}", cell_id))
        } else if parts_topdown.len() == 1 {
            (String::new(), parts_topdown[0].to_string())
        } else {
            let scope = parts_topdown[..parts_topdown.len() - 1].join(&SCOPE_SEP.to_string());
            let name = parts_topdown.last().unwrap().to_string();
            (scope, name)
        };

        let items = items_by_scope.entry(scope_path).or_default();
        items.insert(
            item_name,
            ItemInfo {
                item_type: "memory".to_string(),
                width: 32, // AIGPDK SRAMs are always 32-bit wide
                lsb_at: 0,
                depth: Some(1 << crate::aigpdk::AIGPDK_SRAM_ADDR_WIDTH),
                zero_at: Some(0),
                settable: false,
                input: None,
                output: None,
                attributes: HashMap::new(),
            },
        );
    }

    (items_by_scope, resolved_signals)
}

/// Get items for a specific scope, or all items if scope is None.
pub fn get_items_for_scope(
    items_by_scope: &HashMap<String, HashMap<String, ItemInfo>>,
    scope: Option<&str>,
) -> HashMap<String, ItemInfo> {
    match scope {
        Some(scope_path) => items_by_scope
            .get(scope_path)
            .cloned()
            .unwrap_or_default(),
        None => {
            // Return all items across all scopes, prefixed with scope path
            let mut all = HashMap::new();
            for (scope_path, items) in items_by_scope {
                for (name, info) in items {
                    let full_name = if scope_path.is_empty() {
                        name.clone()
                    } else {
                        format!("{} {}", scope_path, name)
                    };
                    all.insert(full_name, info.clone());
                }
            }
            all
        }
    }
}

/// Get scopes that are children of the given scope.
pub fn get_child_scopes(
    all_scopes: &HashMap<String, ScopeInfo>,
    parent: Option<&str>,
) -> HashMap<String, ScopeInfo> {
    let parent_path = parent.unwrap_or("");
    all_scopes
        .iter()
        .filter(|(path, _)| {
            if parent_path.is_empty() {
                // Children of root: exactly one component
                !path.is_empty() && !path.contains(SCOPE_SEP)
            } else {
                // Children of parent: starts with parent, one more component
                path.starts_with(parent_path)
                    && path.len() > parent_path.len()
                    && path.as_bytes().get(parent_path.len()) == Some(&(SCOPE_SEP as u8))
                    && path[parent_path.len() + 1..].split(SCOPE_SEP).count() == 1
            }
        })
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect()
}

// We need Clone for ScopeInfo and ItemInfo to support the filtering operations
impl Clone for ScopeInfo {
    fn clone(&self) -> Self {
        ScopeInfo {
            scope_type: self.scope_type.clone(),
            definition: self.definition.clone(),
            instantiation: self.instantiation.clone(),
        }
    }
}

impl Clone for ScopeDefinition {
    fn clone(&self) -> Self {
        ScopeDefinition {
            src: self.src.clone(),
            name: self.name.clone(),
            attributes: self.attributes.clone(),
        }
    }
}

impl Clone for ScopeInstantiation {
    fn clone(&self) -> Self {
        ScopeInstantiation {
            src: self.src.clone(),
            attributes: self.attributes.clone(),
        }
    }
}

impl Clone for ItemInfo {
    fn clone(&self) -> Self {
        ItemInfo {
            item_type: self.item_type.clone(),
            width: self.width,
            lsb_at: self.lsb_at,
            depth: self.depth,
            zero_at: self.zero_at,
            settable: self.settable,
            input: self.input,
            output: self.output,
            attributes: self.attributes.clone(),
        }
    }
}
