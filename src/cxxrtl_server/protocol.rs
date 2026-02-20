// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! CXXRTL debug protocol message types and JSON serialization.
//!
//! Implements the protocol defined at <https://cxxrtl.org/protocol.html>.
//! Messages are JSON objects exchanged over a null-terminated TCP stream.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Protocol version we implement.
pub const PROTOCOL_VERSION: u32 = 0;

// ---------------------------------------------------------------------------
// Time representation
// ---------------------------------------------------------------------------

/// A simulation time point: `"<seconds>.<femtoseconds>"`.
///
/// The CXXRTL protocol represents time as a string with format `^\d+\.\d+$`
/// where the first part is seconds (max 2147483647) and the second part is
/// femtoseconds (max 999999999999999).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct TimePoint(pub String);

impl TimePoint {
    /// Create a time point from seconds and femtoseconds.
    pub fn new(seconds: u64, femtoseconds: u64) -> Self {
        TimePoint(format!("{}.{}", seconds, femtoseconds))
    }

    /// The zero time point.
    pub fn zero() -> Self {
        TimePoint("0.0".to_string())
    }

    /// Create from a VCD-style timestamp (in timescale units).
    ///
    /// For now we treat each timestamp as a femtosecond count, which can
    /// be adjusted by providing a timescale multiplier.
    pub fn from_timestamp(ts: u64, timescale_fs: u64) -> Self {
        let total_fs = ts.saturating_mul(timescale_fs);
        let seconds = total_fs / 1_000_000_000_000_000;
        let femtoseconds = total_fs % 1_000_000_000_000_000;
        TimePoint::new(seconds, femtoseconds)
    }

    /// Parse a time point string into (seconds, femtoseconds).
    pub fn parse(&self) -> Option<(u64, u64)> {
        let parts: Vec<&str> = self.0.split('.').collect();
        if parts.len() != 2 {
            return None;
        }
        let seconds = parts[0].parse::<u64>().ok()?;
        let femtoseconds = parts[1].parse::<u64>().ok()?;
        Some((seconds, femtoseconds))
    }

    /// Convert to total femtoseconds.
    pub fn to_femtoseconds(&self) -> Option<u64> {
        let (s, fs) = self.parse()?;
        Some(s.saturating_mul(1_000_000_000_000_000).saturating_add(fs))
    }
}

// ---------------------------------------------------------------------------
// Greeting
// ---------------------------------------------------------------------------

/// Client greeting message.
#[derive(Debug, Deserialize)]
pub struct ClientGreeting {
    #[serde(rename = "type")]
    pub msg_type: String,
    pub version: u32,
}

/// Server greeting message.
#[derive(Debug, Serialize)]
pub struct ServerGreeting {
    #[serde(rename = "type")]
    pub msg_type: String,
    pub version: u32,
    pub commands: Vec<String>,
    pub events: Vec<String>,
    pub features: GreetingFeatures,
}

#[derive(Debug, Serialize)]
pub struct GreetingFeatures {
    pub item_values_encoding: Vec<String>,
}

impl ServerGreeting {
    pub fn new() -> Self {
        ServerGreeting {
            msg_type: "greeting".to_string(),
            version: PROTOCOL_VERSION,
            commands: vec![
                "list_scopes".to_string(),
                "list_items".to_string(),
                "reference_items".to_string(),
                "query_interval".to_string(),
                "get_simulation_status".to_string(),
                "run_simulation".to_string(),
                "pause_simulation".to_string(),
            ],
            events: vec![
                "simulation_paused".to_string(),
                "simulation_finished".to_string(),
            ],
            features: GreetingFeatures {
                item_values_encoding: vec!["base64(u32)".to_string()],
            },
        }
    }
}

// ---------------------------------------------------------------------------
// Commands (client → server)
// ---------------------------------------------------------------------------

/// A raw incoming JSON message. We parse the `type` and `command` fields
/// to dispatch.
#[derive(Debug, Deserialize)]
pub struct RawMessage {
    #[serde(rename = "type")]
    pub msg_type: String,
    #[serde(default)]
    pub command: Option<String>,
    #[serde(default)]
    pub event: Option<String>,
    #[serde(default)]
    pub version: Option<u32>,
}

/// Parsed command from the client.
#[derive(Debug)]
pub enum Command {
    ListScopes {
        scope: Option<String>,
    },
    ListItems {
        scope: Option<String>,
    },
    ReferenceItems {
        reference: String,
        items: Option<Vec<ItemDesignation>>,
    },
    QueryInterval {
        interval: (String, String),
        collapse: bool,
        items: Option<String>,
        item_values_encoding: Option<String>,
        diagnostics: bool,
    },
    GetSimulationStatus,
    RunSimulation {
        until_time: Option<String>,
        until_diagnostics: Vec<String>,
        sample_item_values: bool,
    },
    PauseSimulation,
}

/// An item designation in a `reference_items` command.
#[derive(Debug, Clone)]
pub enum ItemDesignation {
    /// A node: `["name"]`
    Node(String),
    /// Memory rows: `["name", first, last]`
    MemoryRows(String, i64, i64),
}

impl Command {
    /// Parse a command from a raw JSON value.
    pub fn from_json(value: &serde_json::Value) -> Result<Command, String> {
        let cmd = value
            .get("command")
            .and_then(|v| v.as_str())
            .ok_or("missing 'command' field")?;

        match cmd {
            "list_scopes" => Ok(Command::ListScopes {
                scope: value.get("scope").and_then(|v| v.as_str()).map(String::from),
            }),
            "list_items" => Ok(Command::ListItems {
                scope: value.get("scope").and_then(|v| v.as_str()).map(String::from),
            }),
            "reference_items" => {
                let reference = value
                    .get("reference")
                    .and_then(|v| v.as_str())
                    .ok_or("missing 'reference' field")?
                    .to_string();
                let items = match value.get("items") {
                    Some(serde_json::Value::Null) => None,
                    Some(serde_json::Value::Array(arr)) => {
                        let mut designations = Vec::new();
                        for item in arr {
                            let arr = item.as_array().ok_or("item designation must be array")?;
                            let name = arr
                                .first()
                                .and_then(|v| v.as_str())
                                .ok_or("item name must be string")?
                                .to_string();
                            if arr.len() == 1 {
                                designations.push(ItemDesignation::Node(name));
                            } else if arr.len() == 3 {
                                let first =
                                    arr[1].as_i64().ok_or("first index must be integer")?;
                                let last = arr[2].as_i64().ok_or("last index must be integer")?;
                                designations.push(ItemDesignation::MemoryRows(name, first, last));
                            } else {
                                return Err("invalid item designation length".to_string());
                            }
                        }
                        Some(designations)
                    }
                    None => None,
                    _ => return Err("'items' must be array or null".to_string()),
                };
                Ok(Command::ReferenceItems { reference, items })
            }
            "query_interval" => {
                let interval = value
                    .get("interval")
                    .and_then(|v| v.as_array())
                    .ok_or("missing 'interval' field")?;
                if interval.len() != 2 {
                    return Err("interval must have 2 elements".to_string());
                }
                let begin = interval[0]
                    .as_str()
                    .ok_or("interval begin must be string")?
                    .to_string();
                let end = interval[1]
                    .as_str()
                    .ok_or("interval end must be string")?
                    .to_string();
                let collapse = value
                    .get("collapse")
                    .and_then(|v| v.as_bool())
                    .unwrap_or(false);
                let items = value
                    .get("items")
                    .and_then(|v| v.as_str())
                    .map(String::from);
                let item_values_encoding = value
                    .get("item_values_encoding")
                    .and_then(|v| v.as_str())
                    .map(String::from);
                let diagnostics = value
                    .get("diagnostics")
                    .and_then(|v| v.as_bool())
                    .unwrap_or(false);
                Ok(Command::QueryInterval {
                    interval: (begin, end),
                    collapse,
                    items,
                    item_values_encoding,
                    diagnostics,
                })
            }
            "get_simulation_status" => Ok(Command::GetSimulationStatus),
            "run_simulation" => {
                let until_time = value
                    .get("until_time")
                    .and_then(|v| v.as_str())
                    .map(String::from);
                let until_diagnostics = value
                    .get("until_diagnostics")
                    .and_then(|v| v.as_array())
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect()
                    })
                    .unwrap_or_default();
                let sample_item_values = value
                    .get("sample_item_values")
                    .and_then(|v| v.as_bool())
                    .unwrap_or(true);
                Ok(Command::RunSimulation {
                    until_time,
                    until_diagnostics,
                    sample_item_values,
                })
            }
            "pause_simulation" => Ok(Command::PauseSimulation),
            other => Err(format!("unknown command: {}", other)),
        }
    }
}

// ---------------------------------------------------------------------------
// Responses (server → client)
// ---------------------------------------------------------------------------

/// Scope information returned by `list_scopes`.
#[derive(Debug, Serialize)]
pub struct ScopeInfo {
    #[serde(rename = "type")]
    pub scope_type: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub definition: Option<ScopeDefinition>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instantiation: Option<ScopeInstantiation>,
}

#[derive(Debug, Serialize)]
pub struct ScopeDefinition {
    pub src: Option<String>,
    pub name: Option<String>,
    #[serde(skip_serializing_if = "HashMap::is_empty")]
    pub attributes: HashMap<String, serde_json::Value>,
}

#[derive(Debug, Serialize)]
pub struct ScopeInstantiation {
    pub src: Option<String>,
    #[serde(skip_serializing_if = "HashMap::is_empty")]
    pub attributes: HashMap<String, serde_json::Value>,
}

/// Item information returned by `list_items`.
#[derive(Debug, Serialize)]
pub struct ItemInfo {
    #[serde(rename = "type")]
    pub item_type: String,
    pub width: u32,
    pub lsb_at: i32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub depth: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub zero_at: Option<i32>,
    pub settable: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub input: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub output: Option<bool>,
    #[serde(skip_serializing_if = "HashMap::is_empty")]
    pub attributes: HashMap<String, serde_json::Value>,
}

/// A diagnostic entry in a sample.
#[derive(Debug, Serialize)]
pub struct Diagnostic {
    #[serde(rename = "type")]
    pub diag_type: String,
    pub text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub src: Option<String>,
}

/// A sample in a `query_interval` response.
#[derive(Debug, Serialize)]
pub struct Sample {
    pub time: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub item_values: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub diagnostics: Option<Vec<Diagnostic>>,
}

/// Simulation status values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimulationStatus {
    Running,
    Paused,
    Finished,
}

impl SimulationStatus {
    pub fn as_str(&self) -> &'static str {
        match self {
            SimulationStatus::Running => "running",
            SimulationStatus::Paused => "paused",
            SimulationStatus::Finished => "finished",
        }
    }
}

// ---------------------------------------------------------------------------
// Response builders
// ---------------------------------------------------------------------------

/// Build a `list_scopes` response.
pub fn response_list_scopes(scopes: HashMap<String, ScopeInfo>) -> serde_json::Value {
    serde_json::json!({
        "type": "response",
        "command": "list_scopes",
        "scopes": scopes,
    })
}

/// Build a `list_items` response.
pub fn response_list_items(items: HashMap<String, ItemInfo>) -> serde_json::Value {
    serde_json::json!({
        "type": "response",
        "command": "list_items",
        "items": items,
    })
}

/// Build a `reference_items` response.
pub fn response_reference_items() -> serde_json::Value {
    serde_json::json!({
        "type": "response",
        "command": "reference_items",
    })
}

/// Build a `query_interval` response.
pub fn response_query_interval(samples: Vec<Sample>) -> serde_json::Value {
    serde_json::json!({
        "type": "response",
        "command": "query_interval",
        "samples": samples,
    })
}

/// Build a `get_simulation_status` response.
pub fn response_simulation_status(
    status: SimulationStatus,
    latest_time: &TimePoint,
    next_sample_time: Option<&TimePoint>,
) -> serde_json::Value {
    let mut resp = serde_json::json!({
        "type": "response",
        "command": "get_simulation_status",
        "status": status.as_str(),
        "latest_time": latest_time,
    });
    if let Some(nst) = next_sample_time {
        resp["next_sample_time"] = serde_json::json!(nst);
    }
    resp
}

/// Build a `run_simulation` response.
pub fn response_run_simulation() -> serde_json::Value {
    serde_json::json!({
        "type": "response",
        "command": "run_simulation",
    })
}

/// Build a `pause_simulation` response.
pub fn response_pause_simulation(time: &TimePoint) -> serde_json::Value {
    serde_json::json!({
        "type": "response",
        "command": "pause_simulation",
        "time": time,
    })
}

/// Build a `simulation_paused` event.
pub fn event_simulation_paused(time: &TimePoint, cause: &str) -> serde_json::Value {
    serde_json::json!({
        "type": "event",
        "event": "simulation_paused",
        "time": time,
        "cause": cause,
    })
}

/// Build a `simulation_finished` event.
pub fn event_simulation_finished(time: &TimePoint) -> serde_json::Value {
    serde_json::json!({
        "type": "event",
        "event": "simulation_finished",
        "time": time,
    })
}

/// Build an error response.
pub fn response_error(error: &str, message: &str) -> serde_json::Value {
    serde_json::json!({
        "type": "error",
        "error": error,
        "message": message,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_time_point_zero() {
        let tp = TimePoint::zero();
        assert_eq!(tp.0, "0.0");
        assert_eq!(tp.parse(), Some((0, 0)));
        assert_eq!(tp.to_femtoseconds(), Some(0));
    }

    #[test]
    fn test_time_point_roundtrip() {
        let tp = TimePoint::new(1, 500000000000000);
        assert_eq!(tp.0, "1.500000000000000");
        let (s, fs) = tp.parse().unwrap();
        assert_eq!(s, 1);
        assert_eq!(fs, 500000000000000);
    }

    #[test]
    fn test_time_point_from_timestamp() {
        // 1ns = 1_000_000 fs, with timescale of 1ps = 1000 fs
        let tp = TimePoint::from_timestamp(1000, 1_000_000_000_000); // 1000 * 1ps in fs
        assert_eq!(tp.parse(), Some((1, 0)));
    }

    #[test]
    fn test_server_greeting_serializes() {
        let greeting = ServerGreeting::new();
        let json = serde_json::to_string(&greeting).unwrap();
        assert!(json.contains("\"version\":0"));
        assert!(json.contains("list_scopes"));
        assert!(json.contains("base64(u32)"));
    }

    #[test]
    fn test_command_parse_list_scopes() {
        let val = serde_json::json!({
            "type": "command",
            "command": "list_scopes",
            "scope": null
        });
        let cmd = Command::from_json(&val).unwrap();
        assert!(matches!(cmd, Command::ListScopes { scope: None }));
    }

    #[test]
    fn test_command_parse_reference_items() {
        let val = serde_json::json!({
            "type": "command",
            "command": "reference_items",
            "reference": "ref1",
            "items": [["clk"], ["mem", 0, 15]]
        });
        let cmd = Command::from_json(&val).unwrap();
        match cmd {
            Command::ReferenceItems { reference, items } => {
                assert_eq!(reference, "ref1");
                let items = items.unwrap();
                assert_eq!(items.len(), 2);
                assert!(matches!(&items[0], ItemDesignation::Node(n) if n == "clk"));
                assert!(
                    matches!(&items[1], ItemDesignation::MemoryRows(n, 0, 15) if n == "mem")
                );
            }
            _ => panic!("expected ReferenceItems"),
        }
    }

    #[test]
    fn test_command_parse_reference_items_deallocate() {
        let val = serde_json::json!({
            "type": "command",
            "command": "reference_items",
            "reference": "ref1",
            "items": null
        });
        let cmd = Command::from_json(&val).unwrap();
        match cmd {
            Command::ReferenceItems { reference, items } => {
                assert_eq!(reference, "ref1");
                assert!(items.is_none());
            }
            _ => panic!("expected ReferenceItems"),
        }
    }

    #[test]
    fn test_error_response() {
        let resp = response_error("invalid_command", "unknown command: foo");
        assert_eq!(resp["type"], "error");
        assert_eq!(resp["error"], "invalid_command");
    }
}
