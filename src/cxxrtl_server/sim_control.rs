// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! Simulation control for the CXXRTL protocol.
//!
//! Manages the simulation thread's run/pause state and provides the bridge
//! between protocol commands and the GPU dispatch loop.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};

use super::protocol::{SimulationStatus, TimePoint};

/// Simulation control state shared between the protocol handler thread
/// and the simulation thread.
pub struct SimControl {
    /// Current simulation cycle (how many cycles have been completed).
    current_cycle: AtomicU64,
    /// Target cycle to simulate up to (inclusive). The simulation thread
    /// will stop after completing this cycle.
    target_cycle: AtomicU64,
    /// Flag to request immediate pause.
    pause_requested: AtomicBool,
    /// Whether the simulation has finished (no more input cycles).
    finished: AtomicBool,
    /// Whether the simulation is currently running (actively processing cycles).
    running: AtomicBool,
    /// Whether values should be recorded for each cycle.
    sample_values: AtomicBool,
    /// Diagnostic types that should cause a pause.
    until_diagnostics: Mutex<Vec<String>>,
    /// Condition variable to wake the simulation thread when a run command arrives.
    run_signal: Condvar,
    /// Mutex paired with run_signal.
    run_mutex: Mutex<bool>,
    /// Condition variable to notify the protocol thread when simulation pauses.
    pause_signal: Condvar,
    /// Mutex paired with pause_signal.
    pause_mutex: Mutex<bool>,
}

/// The cause of a simulation pause.
#[derive(Debug, Clone)]
pub enum PauseCause {
    /// Reached the requested time limit.
    UntilTime,
    /// A diagnostic matching `until_diagnostics` was triggered.
    UntilDiagnostics,
    /// Client requested a pause.
    ClientPause,
}

impl PauseCause {
    pub fn as_str(&self) -> &'static str {
        match self {
            PauseCause::UntilTime => "until_time",
            PauseCause::UntilDiagnostics => "until_diagnostics",
            PauseCause::ClientPause => "pause",
        }
    }
}

impl SimControl {
    /// Create a new simulation control in the paused state.
    pub fn new() -> Arc<Self> {
        Arc::new(SimControl {
            current_cycle: AtomicU64::new(0),
            target_cycle: AtomicU64::new(0),
            pause_requested: AtomicBool::new(false),
            finished: AtomicBool::new(false),
            running: AtomicBool::new(false),
            sample_values: AtomicBool::new(true),
            until_diagnostics: Mutex::new(Vec::new()),
            run_signal: Condvar::new(),
            run_mutex: Mutex::new(false),
            pause_signal: Condvar::new(),
            pause_mutex: Mutex::new(false),
        })
    }

    /// Get the current simulation status.
    pub fn status(&self) -> SimulationStatus {
        if self.finished.load(Ordering::Acquire) {
            SimulationStatus::Finished
        } else if self.running.load(Ordering::Acquire) {
            SimulationStatus::Running
        } else {
            SimulationStatus::Paused
        }
    }

    /// Get the current cycle number.
    pub fn current_cycle(&self) -> u64 {
        self.current_cycle.load(Ordering::Acquire)
    }

    /// Whether the simulation should sample values each cycle.
    pub fn should_sample_values(&self) -> bool {
        self.sample_values.load(Ordering::Acquire)
    }

    /// Get the current until_diagnostics filter.
    pub fn until_diagnostics(&self) -> Vec<String> {
        self.until_diagnostics.lock().unwrap().clone()
    }

    /// Request the simulation to run up to the target cycle.
    ///
    /// Called from the protocol handler thread.
    pub fn request_run(
        &self,
        target_cycle: u64,
        sample_values: bool,
        until_diagnostics: Vec<String>,
    ) {
        self.target_cycle.store(target_cycle, Ordering::Release);
        self.sample_values.store(sample_values, Ordering::Release);
        *self.until_diagnostics.lock().unwrap() = until_diagnostics;
        self.pause_requested.store(false, Ordering::Release);
        self.running.store(true, Ordering::Release);

        // Wake the simulation thread
        let mut started = self.run_mutex.lock().unwrap();
        *started = true;
        self.run_signal.notify_one();
    }

    /// Request the simulation to pause.
    ///
    /// Called from the protocol handler thread.
    pub fn request_pause(&self) {
        self.pause_requested.store(true, Ordering::Release);
    }

    /// Wait for the simulation to start (blocking).
    ///
    /// Called from the simulation thread. Returns `false` if the simulation
    /// should terminate entirely.
    pub fn wait_for_run_command(&self) -> bool {
        let mut started = self.run_mutex.lock().unwrap();
        while !*started {
            if self.finished.load(Ordering::Acquire) {
                return false;
            }
            started = self.run_signal.wait(started).unwrap();
        }
        *started = false;
        true
    }

    /// Check whether the simulation thread should stop after the current cycle.
    ///
    /// Called from the simulation thread between cycles.
    pub fn should_stop(&self, cycle: u64) -> Option<PauseCause> {
        if self.pause_requested.load(Ordering::Acquire) {
            return Some(PauseCause::ClientPause);
        }
        if cycle >= self.target_cycle.load(Ordering::Acquire) {
            return Some(PauseCause::UntilTime);
        }
        None
    }

    /// Notify that a diagnostic was triggered. Returns `Some(cause)` if
    /// the simulation should pause due to `until_diagnostics`.
    pub fn check_diagnostic(&self, diag_type: &str) -> Option<PauseCause> {
        let diagnostics = self.until_diagnostics.lock().unwrap();
        if diagnostics.iter().any(|d| d == diag_type) {
            Some(PauseCause::UntilDiagnostics)
        } else {
            None
        }
    }

    /// Report that the simulation thread has paused/stopped.
    ///
    /// Called from the simulation thread.
    pub fn report_paused(&self, cycle: u64) {
        self.current_cycle.store(cycle, Ordering::Release);
        self.running.store(false, Ordering::Release);

        let mut paused = self.pause_mutex.lock().unwrap();
        *paused = true;
        self.pause_signal.notify_one();
    }

    /// Report that the simulation has finished (no more cycles).
    ///
    /// Called from the simulation thread.
    pub fn report_finished(&self, cycle: u64) {
        self.current_cycle.store(cycle, Ordering::Release);
        self.running.store(false, Ordering::Release);
        self.finished.store(true, Ordering::Release);

        let mut paused = self.pause_mutex.lock().unwrap();
        *paused = true;
        self.pause_signal.notify_one();
    }

    /// Advance the current cycle counter.
    ///
    /// Called from the simulation thread after each cycle.
    pub fn advance_cycle(&self) {
        self.current_cycle.fetch_add(1, Ordering::AcqRel);
    }

    /// Wait for the simulation to pause (blocking).
    ///
    /// Called from the protocol handler thread.
    pub fn wait_for_pause(&self) {
        let mut paused = self.pause_mutex.lock().unwrap();
        while !*paused {
            paused = self.pause_signal.wait(paused).unwrap();
        }
        *paused = false;
    }

    /// Convert a cycle number to a TimePoint using the provided timestamps.
    pub fn cycle_to_time(
        cycle: u64,
        offsets_timestamps: &[(usize, u64)],
        timescale_fs: u64,
    ) -> TimePoint {
        if cycle == 0 {
            return TimePoint::zero();
        }
        let idx = (cycle as usize).saturating_sub(1);
        if idx < offsets_timestamps.len() {
            TimePoint::from_timestamp(offsets_timestamps[idx].1, timescale_fs)
        } else {
            // Past the end: use the last timestamp
            if let Some(last) = offsets_timestamps.last() {
                TimePoint::from_timestamp(last.1, timescale_fs)
            } else {
                TimePoint::zero()
            }
        }
    }

    /// Convert a TimePoint to a target cycle number.
    pub fn time_to_cycle(
        time: &TimePoint,
        offsets_timestamps: &[(usize, u64)],
        timescale_fs: u64,
    ) -> u64 {
        let target_fs = match time.to_femtoseconds() {
            Some(fs) => fs,
            None => return 0,
        };

        // Find the last cycle whose timestamp is <= target_fs
        for (i, &(_, ts)) in offsets_timestamps.iter().enumerate().rev() {
            let cycle_fs = ts.saturating_mul(timescale_fs);
            if cycle_fs <= target_fs {
                return (i + 1) as u64;
            }
        }
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_state() {
        let ctrl = SimControl::new();
        assert_eq!(ctrl.status(), SimulationStatus::Paused);
        assert_eq!(ctrl.current_cycle(), 0);
    }

    #[test]
    fn test_cycle_to_time() {
        let timestamps = vec![(0, 100u64), (1, 200), (2, 300)];
        let timescale_fs = 1_000_000_000_000u64; // 1ps

        let t0 = SimControl::cycle_to_time(0, &timestamps, timescale_fs);
        assert_eq!(t0.0, "0.0");

        let t1 = SimControl::cycle_to_time(1, &timestamps, timescale_fs);
        let (s, fs) = t1.parse().unwrap();
        assert_eq!(s, 0);
        assert_eq!(fs, 100_000_000_000_000); // 100ps in fs
    }

    #[test]
    fn test_time_to_cycle() {
        let timestamps = vec![(0, 100u64), (1, 200), (2, 300)];
        let timescale_fs = 1_000_000_000_000u64; // 1ps

        let c = SimControl::time_to_cycle(&TimePoint::zero(), &timestamps, timescale_fs);
        assert_eq!(c, 0);

        // 150ps should map to cycle 1 (last cycle at or before 150ps)
        let c = SimControl::time_to_cycle(
            &TimePoint::new(0, 150_000_000_000_000),
            &timestamps,
            timescale_fs,
        );
        assert_eq!(c, 1);

        // 300ps should map to cycle 3
        let c = SimControl::time_to_cycle(
            &TimePoint::new(0, 300_000_000_000_000),
            &timestamps,
            timescale_fs,
        );
        assert_eq!(c, 3);
    }
}
