//! End-of-run structured timing report (`--timing-report <path.json>`).
//!
//! Schema versioning policy mirrors the timing IR (ADR 0002): explicit
//! `schema_version` field, additive-only extension, breaking changes
//! require a major bump and migration notes (per ADR 0008's stability
//! contract).
//!
//! `worst_slack` is currently populated only from violation events.
//! Closest-to-violation tracking when no violation ever occurred needs
//! GPU-side near-miss instrumentation and is not in this module's scope.

use std::cmp::Ordering;
use std::path::Path;

use serde::{Deserialize, Serialize};

/// Schema version for the timing report JSON. Bump the major component
/// for breaking changes; minor for additive fields per the ADR 0008
/// stability contract.
///
/// 1.0.0 — initial schema (commit 58a7a04).
/// 1.1.0 — additive: `stats.violations_truncated`; per-cycle violations
///         array now bounded by default (worst-slack + per-word + totals
///         still reflect every event).
pub const SCHEMA_VERSION: &str = "1.1.0";

/// One setup or hold violation record. Mirrors the `EventType::*Violation`
/// payload, with `word_id` resolved to a human-readable `site` string via
/// the caller's `WordSymbolMap`-backed resolver.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct ViolationRecord {
    pub cycle: u32,
    pub kind: ViolationKind,
    pub word_id: u32,
    pub site: String,
    pub arrival_ps: u32,
    pub constraint_ps: u32,
    pub slack_ps: i32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ViolationKind {
    Setup,
    Hold,
}

/// Run-time metadata. All fields optional except those Jacquard always
/// knows (clock period, version).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RunMetadata {
    pub design: Option<String>,
    pub vector_source: Option<String>,
    pub timing_source: Option<String>,
    pub clock_period_ps: u64,
    pub cycles_run: u32,
    pub jacquard_version: String,
}

/// Aggregate counters mirroring `SimStats` violation-related fields,
/// plus a `violations_truncated` count for the case where the per-cycle
/// `violations` array hit its cap and dropped further records.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ReportStats {
    pub setup_violations: u32,
    pub hold_violations: u32,
    pub events_dropped: u32,
    /// Count of violation events observed but not retained in the
    /// per-cycle `violations` array because the cap was reached. The
    /// `setup_violations`/`hold_violations` totals and `worst_slack`
    /// rankings always reflect every observed event; only the
    /// per-cycle list is bounded.
    #[serde(default)]
    pub violations_truncated: u32,
}

/// Per-word aggregate: violation counts and worst slack across the run.
/// `site` matches the formatted descriptor `WordSymbolMap` produces for
/// `word_id`. `Option`s distinguish "no violation of this kind seen"
/// from a literal-zero observation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct PerWordSummary {
    pub word_id: u32,
    pub site: String,
    pub setup_violations: u32,
    pub hold_violations: u32,
    pub worst_setup_slack_ps: Option<i32>,
    pub worst_hold_slack_ps: Option<i32>,
    pub worst_arrival_ps: Option<u32>,
}

/// Top-N worst-slack rankings per kind.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WorstSlack {
    pub setup: Vec<ViolationRecord>,
    pub hold: Vec<ViolationRecord>,
}

/// Top-level report document.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimingReport {
    pub schema_version: String,
    pub metadata: RunMetadata,
    pub stats: ReportStats,
    pub violations: Vec<ViolationRecord>,
    pub per_word: Vec<PerWordSummary>,
    pub worst_slack: WorstSlack,
}

impl TimingReport {
    pub fn new(metadata: RunMetadata) -> Self {
        Self {
            schema_version: SCHEMA_VERSION.to_string(),
            metadata,
            stats: ReportStats::default(),
            violations: Vec::new(),
            per_word: Vec::new(),
            worst_slack: WorstSlack::default(),
        }
    }

    /// Serialize as pretty JSON to the given writer.
    pub fn write_pretty<W: std::io::Write>(&self, w: &mut W) -> serde_json::Result<()> {
        serde_json::to_writer_pretty(w, self)
    }

    /// Convenience wrapper: write the report to a file path. Creates
    /// parent directories if needed.
    pub fn write_to_path(&self, path: &Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        let f = std::fs::File::create(path)?;
        let mut w = std::io::BufWriter::new(f);
        self.write_pretty(&mut w)
            .map_err(|e| std::io::Error::other(e.to_string()))?;
        Ok(())
    }

    /// Render the report as a multi-line text summary suitable for
    /// stdout. Matches `--timing-summary`. Stable enough for human
    /// inspection but explicitly NOT a parseable contract per ADR 0008
    /// (`--timing-report` JSON is the parseable form).
    ///
    /// ADR 0008 also lists "corner" and "margin percentage" as desired
    /// summary content; both are deferred — corner because the
    /// metadata struct doesn't carry it yet (next IR roundtrip), margin
    /// because it duplicates information the consumer can compute from
    /// `slack_ps` and `clock_period_ps` already in the report.
    pub fn format_summary(&self) -> String {
        // Top-N for the per-word count breakdown. Distinct from
        // jacquard.rs::WORST_SLACK_TOP_N (which gates the JSON ranking
        // capacity); this one bounds the human-facing table width.
        const SUMMARY_TOP_N: usize = 5;

        use std::fmt::Write as _;
        let mut out = String::new();
        // `writeln!` into a String via fmt::Write is infallible.
        writeln!(out, "=== Jacquard Timing Summary ===").unwrap();
        if let Some(d) = &self.metadata.design {
            writeln!(out, "Design:        {d}").unwrap();
        }
        match &self.metadata.vector_source {
            Some(v) => writeln!(
                out,
                "Vectors:       {v} ({} cycles)",
                self.metadata.cycles_run
            )
            .unwrap(),
            None => writeln!(out, "Vectors:       {} cycles", self.metadata.cycles_run).unwrap(),
        }
        writeln!(out, "Clock period:  {} ps", self.metadata.clock_period_ps).unwrap();
        if let Some(t) = &self.metadata.timing_source {
            writeln!(out, "Timing source: {t}").unwrap();
        }
        writeln!(out).unwrap();

        writeln!(out, "Violations:").unwrap();
        writeln!(out, "  Setup: {}", self.stats.setup_violations).unwrap();
        writeln!(out, "  Hold:  {}", self.stats.hold_violations).unwrap();
        let total = self.stats.setup_violations + self.stats.hold_violations;
        writeln!(out, "  Total: {total}").unwrap();
        if self.stats.events_dropped > 0 {
            writeln!(
                out,
                "  Note:  {} events dropped (buffer overflow)",
                self.stats.events_dropped
            )
            .unwrap();
        }
        if self.stats.violations_truncated > 0 {
            writeln!(
                out,
                "  Note:  {} violation records dropped from the per-cycle list (cap reached); totals + worst-slack still reflect all events",
                self.stats.violations_truncated
            )
            .unwrap();
        }
        writeln!(out).unwrap();

        writeln!(out, "Worst slack:").unwrap();
        write_worst(&mut out, "Setup", self.worst_slack.setup.first());
        write_worst(&mut out, "Hold", self.worst_slack.hold.first());

        if !self.per_word.is_empty() {
            writeln!(out).unwrap();
            let shown = self.per_word.len().min(SUMMARY_TOP_N);
            writeln!(
                out,
                "Top {shown} by violation count (of {} total words with violations):",
                self.per_word.len()
            )
            .unwrap();
            for w in self.per_word.iter().take(shown) {
                let total = w.setup_violations + w.hold_violations;
                writeln!(
                    out,
                    "  {} ({} violations): worst setup={} hold={} arrival={}",
                    w.site,
                    total,
                    fmt_optional_ps(w.worst_setup_slack_ps),
                    fmt_optional_ps(w.worst_hold_slack_ps),
                    fmt_optional_ps(w.worst_arrival_ps.map(|a| a as i64)),
                )
                .unwrap();
            }
        }

        out
    }
}

fn write_worst(out: &mut String, label: &str, v: Option<&ViolationRecord>) {
    use std::fmt::Write as _;
    match v {
        Some(v) => writeln!(
            out,
            "  {label}: {}ps  at {}  (cycle {})",
            v.slack_ps, v.site, v.cycle
        )
        .unwrap(),
        None => writeln!(out, "  {label}: (none)").unwrap(),
    }
}

/// Render an `Option<i64>` (representing picoseconds) as `"-"` for
/// `None` and `"<n>ps"` for `Some`. Generic over the integer side via
/// the `Into<i64>` callsite cast above.
fn fmt_optional_ps(v: Option<impl Into<i64>>) -> String {
    match v {
        Some(n) => format!("{}ps", n.into()),
        None => "-".into(),
    }
}

/// Default cap on the per-cycle `violations` array. ~80 bytes per
/// record × 100k = ~8MB JSON; covers any reasonable run without
/// risking unbounded memory on a violation-storm scenario.
pub const DEFAULT_MAX_VIOLATIONS: usize = 100_000;

/// Accumulator that observes every violation as it occurs and keeps:
/// running per-word aggregates, top-N worst-slack rankings, and a
/// bounded per-cycle list. Per-word + worst-slack reflect every
/// observed violation regardless of the per-cycle cap. Materialised
/// into a `TimingReport` at end of run via `finalize`.
pub struct ReportBuilder {
    metadata: RunMetadata,
    violations: Vec<ViolationRecord>,
    per_word: std::collections::HashMap<u32, PerWordSummary>,
    setup_worst: WorstSlackTracker,
    hold_worst: WorstSlackTracker,
    /// `None` = unbounded; `Some(n)` = keep the first `n` records.
    max_violations: Option<usize>,
    truncated: u32,
}

impl ReportBuilder {
    /// Build a new accumulator. `max_violations` of `None` disables the
    /// per-cycle cap (use with caution on violation-storm runs);
    /// `Some(n)` retains the first `n` records and counts the rest in
    /// `stats.violations_truncated`.
    pub fn new(
        metadata: RunMetadata,
        worst_slack_n: usize,
        max_violations: Option<usize>,
    ) -> Self {
        Self {
            metadata,
            violations: Vec::new(),
            per_word: std::collections::HashMap::new(),
            setup_worst: WorstSlackTracker::with_capacity(worst_slack_n),
            hold_worst: WorstSlackTracker::with_capacity(worst_slack_n),
            max_violations,
            truncated: 0,
        }
    }

    pub fn observe(&mut self, v: ViolationRecord) {
        // Worst-slack and per-word always see every violation; only the
        // per-cycle list is bounded.
        match v.kind {
            ViolationKind::Setup => self.setup_worst.push(v.clone()),
            ViolationKind::Hold => self.hold_worst.push(v.clone()),
        }
        update_per_word(&mut self.per_word, &v);
        match self.max_violations {
            Some(cap) if self.violations.len() >= cap => {
                self.truncated = self.truncated.saturating_add(1);
            }
            _ => self.violations.push(v),
        }
    }

    /// Finalise into a `TimingReport`. The builder fills in
    /// `cycles_run` (from the argument) and `violations_truncated`
    /// (from its own counter). The caller-supplied
    /// `stats.violations_truncated` is overwritten.
    pub fn finalize(mut self, cycles_run: u32, mut stats: ReportStats) -> TimingReport {
        self.metadata.cycles_run = cycles_run;
        stats.violations_truncated = self.truncated;
        let per_word = sort_per_word(self.per_word.into_values().collect());
        TimingReport {
            schema_version: SCHEMA_VERSION.to_string(),
            metadata: self.metadata,
            stats,
            violations: self.violations,
            per_word,
            worst_slack: WorstSlack {
                setup: self.setup_worst.into_sorted_vec(),
                hold: self.hold_worst.into_sorted_vec(),
            },
        }
    }
}

/// Bounded top-N tracker keeping the most-negative slacks (closest to
/// or beyond violation). Internally a `Vec` since N is small (default 10).
struct WorstSlackTracker {
    capacity: usize,
    items: Vec<ViolationRecord>,
}

impl WorstSlackTracker {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            capacity: capacity.max(1),
            items: Vec::with_capacity(capacity.max(1)),
        }
    }

    fn push(&mut self, v: ViolationRecord) {
        // Worst = most negative slack; we keep the smallest N.
        if self.items.len() < self.capacity {
            self.items.push(v);
            return;
        }
        // Find the largest (least negative) currently in the set; replace if v is smaller.
        let (idx, max) = self
            .items
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.slack_ps.cmp(&b.slack_ps))
            .expect("non-empty by construction");
        if v.slack_ps < max.slack_ps {
            self.items[idx] = v;
        }
    }

    fn into_sorted_vec(mut self) -> Vec<ViolationRecord> {
        self.items.sort_by(|a, b| match a.slack_ps.cmp(&b.slack_ps) {
            Ordering::Equal => a.cycle.cmp(&b.cycle),
            other => other,
        });
        self.items
    }
}

/// Fold one violation into the running per-word table. Used during
/// `ReportBuilder::observe` so the aggregate sees every event even
/// when the per-cycle list is capped.
fn update_per_word(
    by_word: &mut std::collections::HashMap<u32, PerWordSummary>,
    v: &ViolationRecord,
) {
    let entry = by_word.entry(v.word_id).or_insert_with(|| PerWordSummary {
        word_id: v.word_id,
        site: v.site.clone(),
        setup_violations: 0,
        hold_violations: 0,
        worst_setup_slack_ps: None,
        worst_hold_slack_ps: None,
        worst_arrival_ps: None,
    });
    match v.kind {
        ViolationKind::Setup => {
            entry.setup_violations += 1;
            entry.worst_setup_slack_ps = Some(match entry.worst_setup_slack_ps {
                Some(s) => s.min(v.slack_ps),
                None => v.slack_ps,
            });
        }
        ViolationKind::Hold => {
            entry.hold_violations += 1;
            entry.worst_hold_slack_ps = Some(match entry.worst_hold_slack_ps {
                Some(s) => s.min(v.slack_ps),
                None => v.slack_ps,
            });
        }
    }
    entry.worst_arrival_ps = Some(match entry.worst_arrival_ps {
        Some(a) => a.max(v.arrival_ps),
        None => v.arrival_ps,
    });
}

/// Sort the per-word table for output: descending by total violation
/// count, then ascending by word_id for deterministic ties.
fn sort_per_word(mut summaries: Vec<PerWordSummary>) -> Vec<PerWordSummary> {
    summaries.sort_by(|a, b| {
        let total_a = a.setup_violations + a.hold_violations;
        let total_b = b.setup_violations + b.hold_violations;
        total_b.cmp(&total_a).then(a.word_id.cmp(&b.word_id))
    });
    summaries
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_violation(
        cycle: u32,
        kind: ViolationKind,
        word_id: u32,
        slack_ps: i32,
        arrival_ps: u32,
    ) -> ViolationRecord {
        ViolationRecord {
            cycle,
            kind,
            word_id,
            site: format!("dff_w{word_id}"),
            arrival_ps,
            constraint_ps: 1000,
            slack_ps,
        }
    }

    #[test]
    fn schema_version_is_semver_like() {
        assert!(SCHEMA_VERSION.split('.').count() == 3);
    }

    #[test]
    fn empty_report_serializes_with_schema_version() {
        let r = TimingReport::new(RunMetadata {
            jacquard_version: "0.1.0".into(),
            clock_period_ps: 1000,
            ..Default::default()
        });
        let s = serde_json::to_string(&r).unwrap();
        let expected = format!(r#""schema_version":"{SCHEMA_VERSION}""#);
        assert!(s.contains(&expected), "got: {s}");
        assert!(s.contains(r#""violations":[]"#));
        assert!(s.contains(r#""per_word":[]"#));
    }

    #[test]
    fn round_trip_preserves_data() {
        let mut b = ReportBuilder::new(
            RunMetadata {
                jacquard_version: "0.1.0".into(),
                clock_period_ps: 1000,
                ..Default::default()
            },
            5,
            None,
        );
        b.observe(make_violation(10, ViolationKind::Setup, 5, -100, 1100));
        b.observe(make_violation(11, ViolationKind::Hold, 7, -20, 30));
        let report = b.finalize(50, ReportStats { setup_violations: 1, hold_violations: 1, ..Default::default() });
        let json = serde_json::to_string(&report).unwrap();
        let parsed: TimingReport = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.violations.len(), 2);
        assert_eq!(parsed.metadata.cycles_run, 50);
        assert_eq!(parsed.stats.setup_violations, 1);
    }

    #[test]
    fn worst_slack_tracker_keeps_n_most_negative() {
        let mut t = WorstSlackTracker::with_capacity(2);
        for slack in [-10, -50, -5, -100, -30] {
            t.push(make_violation(0, ViolationKind::Setup, 0, slack, 0));
        }
        let kept: Vec<i32> = t.into_sorted_vec().iter().map(|v| v.slack_ps).collect();
        assert_eq!(kept, vec![-100, -50]);
    }

    #[test]
    fn worst_slack_tracker_capacity_one() {
        // `with_capacity(0)` floors to 1; the test name reflects the
        // resulting capacity, not the input.
        let mut t = WorstSlackTracker::with_capacity(0);
        t.push(make_violation(0, ViolationKind::Setup, 0, -10, 0));
        t.push(make_violation(1, ViolationKind::Setup, 1, -5, 0));
        let kept = t.into_sorted_vec();
        assert_eq!(kept.len(), 1);
        assert_eq!(kept[0].slack_ps, -10);
    }

    #[test]
    fn worst_slack_tracker_ties_break_by_cycle() {
        let mut t = WorstSlackTracker::with_capacity(3);
        t.push(make_violation(20, ViolationKind::Setup, 0, -50, 0));
        t.push(make_violation(10, ViolationKind::Setup, 0, -50, 0));
        t.push(make_violation(30, ViolationKind::Setup, 0, -50, 0));
        let kept: Vec<u32> = t.into_sorted_vec().iter().map(|v| v.cycle).collect();
        assert_eq!(kept, vec![10, 20, 30]);
    }

    fn aggregate_via_builder(violations: Vec<ViolationRecord>) -> Vec<PerWordSummary> {
        let mut b = ReportBuilder::new(RunMetadata::default(), 1, None);
        for v in violations {
            b.observe(v);
        }
        b.finalize(0, ReportStats::default()).per_word
    }

    #[test]
    fn per_word_aggregates_setup_and_hold_separately() {
        let agg = aggregate_via_builder(vec![
            make_violation(1, ViolationKind::Setup, 5, -100, 1100),
            make_violation(2, ViolationKind::Setup, 5, -50, 1050),
            make_violation(3, ViolationKind::Hold, 5, -10, 5),
            make_violation(4, ViolationKind::Setup, 7, -200, 1200),
        ]);
        assert_eq!(agg.len(), 2);
        // sorted by total violations desc → word 5 (3) before word 7 (1)
        assert_eq!(agg[0].word_id, 5);
        assert_eq!(agg[0].setup_violations, 2);
        assert_eq!(agg[0].hold_violations, 1);
        assert_eq!(agg[0].worst_setup_slack_ps, Some(-100));
        assert_eq!(agg[0].worst_hold_slack_ps, Some(-10));
        assert_eq!(agg[0].worst_arrival_ps, Some(1100));
        assert_eq!(agg[1].word_id, 7);
        assert_eq!(agg[1].worst_setup_slack_ps, Some(-200));
        assert_eq!(agg[1].worst_hold_slack_ps, None);
    }

    #[test]
    fn write_to_path_creates_parent_dirs() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("nested/sub/report.json");
        let r = TimingReport::new(RunMetadata {
            jacquard_version: "0.1.0".into(),
            clock_period_ps: 1000,
            ..Default::default()
        });
        r.write_to_path(&path).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        let parsed: TimingReport = serde_json::from_str(&text).unwrap();
        assert_eq!(parsed.schema_version, SCHEMA_VERSION);
    }

    #[test]
    fn per_word_worst_arrival_tracks_max_across_kinds_and_order() {
        // Hold violations report small arrivals; setup violations large.
        // worst_arrival_ps must reflect the maximum regardless of kind or
        // observation order. Order: hold(arrival=10), setup(900), hold(20).
        let agg = aggregate_via_builder(vec![
            make_violation(1, ViolationKind::Hold, 5, -40, 10),
            make_violation(2, ViolationKind::Setup, 5, -100, 900),
            make_violation(3, ViolationKind::Hold, 5, -20, 20),
        ]);
        assert_eq!(agg.len(), 1);
        assert_eq!(agg[0].worst_arrival_ps, Some(900));
    }

    #[test]
    fn violations_array_caps_at_default() {
        // DEFAULT_MAX_VIOLATIONS is 100k; sanity-test with a smaller cap.
        let mut b = ReportBuilder::new(RunMetadata::default(), 5, Some(3));
        for cycle in 0..10 {
            b.observe(make_violation(cycle, ViolationKind::Setup, 0, -10, 0));
        }
        let report = b.finalize(10, ReportStats::default());
        assert_eq!(report.violations.len(), 3);
        assert_eq!(report.stats.violations_truncated, 7);
        // Worst-slack tracker still saw all 10.
        assert_eq!(report.worst_slack.setup.len(), 5);
        // Per-word now reflects every observed violation (incrementally
        // updated in observe(), not re-derived from the truncated list).
        assert_eq!(report.per_word[0].setup_violations, 10);
    }

    #[test]
    fn violations_array_unbounded_when_cap_none() {
        let mut b = ReportBuilder::new(RunMetadata::default(), 5, None);
        for cycle in 0..50 {
            b.observe(make_violation(cycle, ViolationKind::Setup, 0, -10, 0));
        }
        let report = b.finalize(50, ReportStats::default());
        assert_eq!(report.violations.len(), 50);
        assert_eq!(report.stats.violations_truncated, 0);
    }

    #[test]
    fn truncated_count_appears_in_summary() {
        let mut b = ReportBuilder::new(RunMetadata::default(), 5, Some(2));
        for cycle in 0..5 {
            b.observe(make_violation(cycle, ViolationKind::Setup, 0, -10, 0));
        }
        let report = b.finalize(5, ReportStats::default());
        let summary = report.format_summary();
        assert!(
            summary.contains("3 violation records dropped"),
            "got: {summary}"
        );
    }

    #[test]
    fn report_builder_routes_setup_and_hold_to_separate_trackers() {
        let mut b = ReportBuilder::new(RunMetadata::default(), 3, None);
        b.observe(make_violation(1, ViolationKind::Setup, 0, -100, 0));
        b.observe(make_violation(2, ViolationKind::Hold, 1, -5, 0));
        b.observe(make_violation(3, ViolationKind::Setup, 0, -200, 0));
        let report = b.finalize(10, ReportStats::default());
        assert_eq!(report.worst_slack.setup.len(), 2);
        assert_eq!(report.worst_slack.hold.len(), 1);
        assert_eq!(report.worst_slack.setup[0].slack_ps, -200);
    }

    #[test]
    fn format_summary_includes_metadata_and_violation_totals() {
        let mut b = ReportBuilder::new(
            RunMetadata {
                design: Some("tiny.gv".into()),
                vector_source: Some("boot.vcd".into()),
                timing_source: Some("tiny.jtir".into()),
                clock_period_ps: 1000,
                cycles_run: 0,
                jacquard_version: "0.1.0".into(),
            },
            5,
            None,
        );
        b.observe(make_violation(10, ViolationKind::Setup, 5, -100, 1100));
        b.observe(make_violation(11, ViolationKind::Hold, 7, -20, 30));
        let report = b.finalize(50, ReportStats { setup_violations: 1, hold_violations: 1, ..Default::default() });
        let text = report.format_summary();
        assert!(text.contains("Design:        tiny.gv"), "got:\n{text}");
        assert!(text.contains("Vectors:       boot.vcd (50 cycles)"), "got:\n{text}");
        assert!(text.contains("Clock period:  1000 ps"), "got:\n{text}");
        assert!(text.contains("Setup: 1"), "got:\n{text}");
        assert!(text.contains("Hold:  1"), "got:\n{text}");
        assert!(text.contains("Total: 2"), "got:\n{text}");
        assert!(text.contains("Setup: -100ps"), "got:\n{text}");
        assert!(text.contains("Hold: -20ps"), "got:\n{text}");
    }

    #[test]
    fn format_summary_handles_no_violations() {
        let report = TimingReport::new(RunMetadata {
            design: Some("clean.gv".into()),
            jacquard_version: "0.1.0".into(),
            clock_period_ps: 1000,
            cycles_run: 100,
            ..Default::default()
        });
        let text = report.format_summary();
        assert!(text.contains("Setup: 0"));
        assert!(text.contains("Hold:  0"));
        assert!(text.contains("Setup: (none)"));
        assert!(text.contains("Hold: (none)"));
        assert!(!text.contains("Top 0"), "should not print empty top-N section");
    }

    #[test]
    fn format_summary_warns_on_dropped_events() {
        let mut b = ReportBuilder::new(RunMetadata::default(), 5, None);
        b.observe(make_violation(1, ViolationKind::Setup, 0, -10, 0));
        let report = b.finalize(10, ReportStats { setup_violations: 1, events_dropped: 42, ..Default::default() });
        let text = report.format_summary();
        assert!(text.contains("42 events dropped"), "got:\n{text}");
    }

    /// The sample fixture is pinned at schema_version 1.0.0 so that
    /// fields added in later minor versions (e.g. `stats.violations_truncated`
    /// in 1.1.0) regression-test their `#[serde(default)]` attribute.
    /// If this ever fails to parse against the current types, the
    /// schema has drifted in a non-additive way and the stability
    /// contract has been broken.
    #[test]
    fn sample_fixture_parses_with_serde_default_for_newer_fields() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/timing_ir/sample_reports/two_violations.json");
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        let parsed: TimingReport =
            serde_json::from_str(&text).expect("sample fixture failed to parse");
        assert_eq!(parsed.schema_version, "1.0.0");
        assert_eq!(parsed.violations.len(), 3);
        assert_eq!(parsed.per_word.len(), 2);
        assert_eq!(parsed.worst_slack.setup.len(), 2);
        assert_eq!(parsed.worst_slack.hold.len(), 1);
        // 1.1.0 field absent in the fixture; serde default kicks in.
        assert_eq!(parsed.stats.violations_truncated, 0);
    }
}
