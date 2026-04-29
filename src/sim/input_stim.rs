// SPDX-FileCopyrightText: Copyright (c) 2026 ChipFlow
// SPDX-License-Identifier: Apache-2.0
//! Input-stimulus dispatcher for cosim.
//!
//! Reads an `input.json` file in chipflow's command-list format and
//! dispatches `action` commands into per-peripheral queues, gated by
//! `wait` commands that synchronize on the firmware's output events.
//!
//! Schema (chipflow-compatible, with one extension):
//! ```json
//! {
//!   "commands": [
//!     { "type": "wait",   "peripheral": "uart_0",  "event": "tx",  "payload": 240 },
//!     { "type": "action", "peripheral": "gpio_1",  "event": "set", "payload": "00111100" },
//!     { "type": "stop",   "reason": "test complete" }
//!   ]
//! }
//! ```
//!
//! `stop` is a Jacquard extension; chipflow's harness loops until
//! `num_steps` is exhausted instead.

use std::collections::HashMap;
use std::path::Path;

use serde::Deserialize;
use serde_json::Value;

/// One entry in the input command list.
#[derive(Debug, Clone, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum InputCommand {
    /// Block until the firmware emits a matching output event, then advance
    /// past this command and queue any actions that follow it.
    Wait {
        peripheral: String,
        event: String,
        #[serde(default)]
        payload: Value,
    },
    /// Drive an input on the given peripheral. Queued for the next time
    /// that peripheral is stepped.
    Action {
        peripheral: String,
        event: String,
        #[serde(default)]
        payload: Value,
    },
    /// Halt simulation cleanly. The cosim loop checks `dispatcher.stopped()`
    /// and exits the next time it's about to start a new batch.
    Stop {
        #[serde(default)]
        reason: Option<String>,
    },
}

#[derive(Debug, Deserialize)]
struct InputFile {
    commands: Vec<InputCommand>,
}

/// One queued input action waiting to be applied to a peripheral.
#[derive(Debug, Clone)]
pub struct QueuedAction {
    pub event: String,
    pub payload: Value,
}

/// Dispatcher state for a single cosim run.
pub struct InputDispatcher {
    commands: Vec<InputCommand>,
    /// Index of the next command to evaluate. After `fetch_actions_into_queue`,
    /// this either points at a `Wait`, a `Stop`, or one past the end.
    cursor: usize,
    /// Pending actions per peripheral. Drained by peripheral models as they
    /// step.
    queued: HashMap<String, Vec<QueuedAction>>,
    /// Set when a `Stop` command is reached.
    stopped: bool,
    stop_reason: Option<String>,
}

impl InputDispatcher {
    /// Load commands from a JSON file path.
    pub fn from_file(path: &Path) -> std::io::Result<Self> {
        let raw = std::fs::read_to_string(path)?;
        let parsed: InputFile = serde_json::from_str(&raw).map_err(|e| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("invalid input commands {}: {}", path.display(), e),
            )
        })?;
        Ok(Self::from_commands(parsed.commands))
    }

    pub fn from_commands(commands: Vec<InputCommand>) -> Self {
        let mut d = Self {
            commands,
            cursor: 0,
            queued: HashMap::new(),
            stopped: false,
            stop_reason: None,
        };
        d.fetch_actions_into_queue();
        d
    }

    /// Total commands loaded (debug/diagnostics).
    pub fn len(&self) -> usize {
        self.commands.len()
    }

    /// True iff there are no commands loaded.
    pub fn is_empty(&self) -> bool {
        self.commands.is_empty()
    }

    /// Cursor position (debug/diagnostics).
    pub fn cursor(&self) -> usize {
        self.cursor
    }

    /// True after a `Stop` command has been reached.
    pub fn stopped(&self) -> bool {
        self.stopped
    }

    /// Reason recorded with the most recent `Stop`, if any.
    pub fn stop_reason(&self) -> Option<&str> {
        self.stop_reason.as_deref()
    }

    /// Advance the cursor past contiguous `Action` and `Stop` entries,
    /// queuing actions and recording stop. Stops at the first `Wait` or
    /// the end of the list.
    fn fetch_actions_into_queue(&mut self) {
        while self.cursor < self.commands.len() {
            match &self.commands[self.cursor] {
                InputCommand::Wait { .. } => break,
                InputCommand::Action {
                    peripheral,
                    event,
                    payload,
                } => {
                    self.queued
                        .entry(peripheral.clone())
                        .or_default()
                        .push(QueuedAction {
                            event: event.clone(),
                            payload: payload.clone(),
                        });
                    self.cursor += 1;
                }
                InputCommand::Stop { reason } => {
                    self.stopped = true;
                    self.stop_reason = reason.clone();
                    self.cursor += 1;
                    // Stop ends the dispatch — nothing more to fetch.
                    break;
                }
            }
        }
    }

    /// Notify the dispatcher of a firmware output event. If it matches the
    /// current `Wait`, advance past it and fetch the next batch of actions.
    pub fn on_event(&mut self, peripheral: &str, event: &str, payload: &Value) {
        if self.stopped || self.cursor >= self.commands.len() {
            return;
        }
        if let InputCommand::Wait {
            peripheral: wp,
            event: we,
            payload: wpl,
        } = &self.commands[self.cursor]
        {
            if wp == peripheral && we == event && wpl == payload {
                self.cursor += 1;
                self.fetch_actions_into_queue();
            }
        }
    }

    /// Drain queued actions for a peripheral. Returns empty if nothing is
    /// queued.
    pub fn pending_actions(&mut self, peripheral: &str) -> Vec<QueuedAction> {
        self.queued.remove(peripheral).unwrap_or_default()
    }

    /// Number of commands remaining unconsumed (for end-of-run diagnostics).
    pub fn remaining(&self) -> usize {
        self.commands.len().saturating_sub(self.cursor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn cmd_json(s: &str) -> Vec<InputCommand> {
        let f: InputFile = serde_json::from_str(s).unwrap();
        f.commands
    }

    #[test]
    fn parses_chipflow_schema() {
        let cmds = cmd_json(
            r#"{
              "commands": [
                { "type": "wait", "peripheral": "uart_0", "event": "tx", "payload": 51 },
                { "type": "action", "peripheral": "gpio_1", "event": "set", "payload": "00111100" },
                { "type": "stop" }
              ]
            }"#,
        );
        assert_eq!(cmds.len(), 3);
        assert!(matches!(cmds[0], InputCommand::Wait { .. }));
        assert!(matches!(cmds[1], InputCommand::Action { .. }));
        assert!(matches!(cmds[2], InputCommand::Stop { reason: None }));
    }

    #[test]
    fn initial_actions_get_queued() {
        let cmds = cmd_json(
            r#"{
              "commands": [
                { "type": "action", "peripheral": "gpio_0", "event": "set", "payload": "00" },
                { "type": "action", "peripheral": "gpio_1", "event": "set", "payload": "11" },
                { "type": "wait", "peripheral": "uart_0", "event": "tx", "payload": 1 }
              ]
            }"#,
        );
        let mut d = InputDispatcher::from_commands(cmds);
        assert_eq!(d.cursor(), 2); // sitting on the wait
        let a0 = d.pending_actions("gpio_0");
        assert_eq!(a0.len(), 1);
        assert_eq!(a0[0].event, "set");
        let a1 = d.pending_actions("gpio_1");
        assert_eq!(a1.len(), 1);
        // Re-drain returns nothing
        assert_eq!(d.pending_actions("gpio_0").len(), 0);
    }

    #[test]
    fn matching_wait_advances_cursor_and_queues_next_actions() {
        let cmds = cmd_json(
            r#"{
              "commands": [
                { "type": "wait", "peripheral": "uart_0", "event": "tx", "payload": 51 },
                { "type": "action", "peripheral": "gpio_1", "event": "set", "payload": "01" }
              ]
            }"#,
        );
        let mut d = InputDispatcher::from_commands(cmds);
        assert_eq!(d.cursor(), 0);
        assert!(d.pending_actions("gpio_1").is_empty());

        // Wrong event → no progress
        d.on_event("uart_0", "tx", &json!(99));
        assert_eq!(d.cursor(), 0);

        // Right event → cursor advances, action queued
        d.on_event("uart_0", "tx", &json!(51));
        assert_eq!(d.cursor(), 2);
        assert_eq!(d.pending_actions("gpio_1").len(), 1);
    }

    #[test]
    fn stop_halts_dispatch() {
        let cmds = cmd_json(
            r#"{
              "commands": [
                { "type": "stop", "reason": "test ok" },
                { "type": "action", "peripheral": "gpio_0", "event": "set", "payload": "00" }
              ]
            }"#,
        );
        let d = InputDispatcher::from_commands(cmds);
        assert!(d.stopped());
        assert_eq!(d.stop_reason(), Some("test ok"));
        // Action after stop is not queued.
        assert!(!d.queued.contains_key("gpio_0"));
    }

    #[test]
    fn stop_via_wait_match() {
        let cmds = cmd_json(
            r#"{
              "commands": [
                { "type": "wait", "peripheral": "uart_0", "event": "tx", "payload": 10 },
                { "type": "stop" }
              ]
            }"#,
        );
        let mut d = InputDispatcher::from_commands(cmds);
        assert!(!d.stopped());
        d.on_event("uart_0", "tx", &json!(10));
        assert!(d.stopped());
    }

    #[test]
    fn empty_command_list_is_noop() {
        let d = InputDispatcher::from_commands(vec![]);
        assert!(!d.stopped());
        assert_eq!(d.remaining(), 0);
    }
}
