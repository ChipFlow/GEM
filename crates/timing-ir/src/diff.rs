//! Structured diff between two timing-IR documents.
//!
//! Diff is keyed by (cell instance, pins, condition) so that arcs/checks
//! appear in the same slot across both inputs regardless of ordering.
//! Delay-value comparisons honour an absolute-ps tolerance and a relative
//! tolerance; either being satisfied counts as a match. This matches the
//! convention used by OpenSTA-style regression suites.
//!
//! The diff report is serialisable (via serde) for CI consumption and
//! also has a human-readable text rendering via `Display`.

use std::collections::BTreeMap;

use serde::Serialize;

use crate::schema::jacquard::timing_ir as ir;

/// Tolerances for comparing timing values. A delay pair is considered
/// equal if it satisfies EITHER the absolute or relative tolerance.
#[derive(Clone, Copy, Debug)]
pub struct DiffOptions {
    /// Absolute tolerance in picoseconds. Default `5.0`.
    pub absolute_ps: f64,
    /// Relative tolerance as a fraction. Default `0.02` (2%).
    pub relative: f64,
}

impl Default for DiffOptions {
    fn default() -> Self {
        Self {
            absolute_ps: 5.0,
            relative: 0.02,
        }
    }
}

impl DiffOptions {
    /// Returns `true` if `a` and `b` are within tolerance.
    pub fn within_tolerance(&self, a: f64, b: f64) -> bool {
        let diff = (a - b).abs();
        if diff <= self.absolute_ps {
            return true;
        }
        let max = a.abs().max(b.abs());
        max > 0.0 && diff / max <= self.relative
    }
}

/// Setup/hold edge variant.
///
/// Locally redefined (instead of using the FlatBuffers-generated
/// `ir::CheckEdge`) so the diff report can serialise via serde and the
/// keys can be ordered for stable output.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub enum Edge {
    Posedge,
    Negedge,
}

impl From<ir::CheckEdge> for Edge {
    fn from(e: ir::CheckEdge) -> Self {
        match e {
            ir::CheckEdge::Posedge => Edge::Posedge,
            ir::CheckEdge::Negedge => Edge::Negedge,
            // Unrecognised CheckEdge values indicate a future-schema input
            // we do not understand. Fail loud rather than silently coerce.
            other => panic!("unknown CheckEdge variant {}", other.0),
        }
    }
}

impl std::fmt::Display for Edge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Edge::Posedge => f.write_str("Posedge"),
            Edge::Negedge => f.write_str("Negedge"),
        }
    }
}

/// Key for matching timing arcs across IRs.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct ArcKey {
    pub cell_instance: String,
    pub driver_pin: String,
    pub load_pin: String,
    pub condition: String,
}

impl std::fmt::Display for ArcKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.condition.is_empty() {
            write!(
                f,
                "{}/{}→{}",
                self.cell_instance, self.driver_pin, self.load_pin
            )
        } else {
            write!(
                f,
                "{}/{}→{} [{}]",
                self.cell_instance, self.driver_pin, self.load_pin, self.condition
            )
        }
    }
}

/// Key for matching interconnect delays across IRs.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct InterconnectKey {
    pub net: String,
    pub from_pin: String,
    pub to_pin: String,
}

impl std::fmt::Display for InterconnectKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} ({}→{})", self.net, self.from_pin, self.to_pin)
    }
}

/// Key for matching setup/hold checks across IRs.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct SetupHoldKey {
    pub cell_instance: String,
    pub d_pin: String,
    pub clk_pin: String,
    pub edge: Edge,
    pub condition: String,
}

/// Key for matching per-DFF clock arrivals across IRs.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub struct ClockArrivalKey {
    pub cell_instance: String,
    pub clk_pin: String,
}

impl std::fmt::Display for ClockArrivalKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}", self.cell_instance, self.clk_pin)
    }
}

impl std::fmt::Display for SetupHoldKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}/{}@{} edge={}",
            self.cell_instance, self.d_pin, self.clk_pin, self.edge
        )?;
        if !self.condition.is_empty() {
            write!(f, " [{}]", self.condition)?;
        }
        Ok(())
    }
}

/// A single value mismatch past tolerance, with a human-readable reason.
#[derive(Clone, Debug, Serialize)]
pub struct ValueMismatch<K: Serialize> {
    pub key: K,
    pub reason: String,
}

/// Category diff for one annotation kind.
///
/// `Default` is implemented manually so the key type `K` does not itself
/// need to be `Default`.
#[derive(Clone, Debug, Serialize)]
pub struct CategoryDiff<K: Serialize> {
    pub only_in_a: Vec<K>,
    pub only_in_b: Vec<K>,
    pub value_mismatches: Vec<ValueMismatch<K>>,
}

impl<K: Serialize> Default for CategoryDiff<K> {
    fn default() -> Self {
        Self {
            only_in_a: Vec::new(),
            only_in_b: Vec::new(),
            value_mismatches: Vec::new(),
        }
    }
}

impl<K: Serialize> CategoryDiff<K> {
    /// `true` if this category contains no extras and no value mismatches.
    pub fn is_clean(&self) -> bool {
        self.only_in_a.is_empty() && self.only_in_b.is_empty() && self.value_mismatches.is_empty()
    }
}

/// Schema-version comparison result. If inputs disagree on the schema
/// major, consumers should treat further diffing as approximate.
#[derive(Clone, Debug, Serialize)]
pub struct SchemaVersionMatch {
    pub a: (u16, u16, u16),
    pub b: (u16, u16, u16),
    pub major_matches: bool,
}

/// Per-corner comparison. Corner sets compare structurally — same names
/// in the same order means full match.
#[derive(Clone, Debug, Default, Serialize)]
pub struct CornerDiff {
    pub a_corner_names: Vec<String>,
    pub b_corner_names: Vec<String>,
    pub mismatches: Vec<String>,
}

impl CornerDiff {
    /// `true` if the corner sets agree.
    pub fn is_clean(&self) -> bool {
        self.mismatches.is_empty()
    }
}

/// Diff options as recorded in the report (echoed back so the consumer
/// can see what tolerance produced this diff).
#[derive(Clone, Copy, Debug, Serialize)]
pub struct DiffOptionsReport {
    pub absolute_ps: f64,
    pub relative: f64,
}

impl From<DiffOptions> for DiffOptionsReport {
    fn from(o: DiffOptions) -> Self {
        Self {
            absolute_ps: o.absolute_ps,
            relative: o.relative,
        }
    }
}

/// Top-level diff report.
#[derive(Clone, Debug, Serialize)]
pub struct DiffReport {
    pub schema_version: SchemaVersionMatch,
    pub corners: CornerDiff,
    pub timing_arcs: CategoryDiff<ArcKey>,
    pub interconnect_delays: CategoryDiff<InterconnectKey>,
    pub setup_hold_checks: CategoryDiff<SetupHoldKey>,
    pub clock_arrivals: CategoryDiff<ClockArrivalKey>,
    pub vendor_extension_counts: (usize, usize),
    pub options: DiffOptionsReport,
}

impl DiffReport {
    /// `true` if the report contains no diffs past tolerance.
    ///
    /// Schema-version mismatches are *not* diffs on their own — they are
    /// flagged so a consumer can interpret the rest of the report
    /// accordingly, but comparing IRs with different schema majors is a
    /// legitimate use case during migrations.
    pub fn is_clean(&self) -> bool {
        self.corners.is_clean()
            && self.timing_arcs.is_clean()
            && self.interconnect_delays.is_clean()
            && self.setup_hold_checks.is_clean()
            && self.clock_arrivals.is_clean()
    }
}

fn arc_key(arc: &ir::TimingArc) -> ArcKey {
    ArcKey {
        cell_instance: arc.cell_instance().unwrap_or("").to_string(),
        driver_pin: arc.driver_pin().unwrap_or("").to_string(),
        load_pin: arc.load_pin().unwrap_or("").to_string(),
        condition: arc.condition().unwrap_or("").to_string(),
    }
}

fn interconnect_key(d: &ir::InterconnectDelay) -> InterconnectKey {
    InterconnectKey {
        net: d.net().unwrap_or("").to_string(),
        from_pin: d.from_pin().unwrap_or("").to_string(),
        to_pin: d.to_pin().unwrap_or("").to_string(),
    }
}

fn setup_hold_key(c: &ir::SetupHoldCheck) -> SetupHoldKey {
    SetupHoldKey {
        cell_instance: c.cell_instance().unwrap_or("").to_string(),
        d_pin: c.d_pin().unwrap_or("").to_string(),
        clk_pin: c.clk_pin().unwrap_or("").to_string(),
        edge: c.edge().into(),
        condition: c.condition().unwrap_or("").to_string(),
    }
}

fn clock_arrival_key(a: &ir::ClockArrival) -> ClockArrivalKey {
    ClockArrivalKey {
        cell_instance: a.cell_instance().unwrap_or("").to_string(),
        clk_pin: a.clk_pin().unwrap_or("").to_string(),
    }
}

fn collect_timing_values(
    v: Option<flatbuffers::Vector<'_, ir::TimingValue>>,
) -> Vec<(u32, f64, f64, f64)> {
    match v {
        Some(vec) => (0..vec.len())
            .map(|i| {
                let tv = vec.get(i);
                (tv.corner_index(), tv.min(), tv.typ(), tv.max())
            })
            .collect(),
        None => Vec::new(),
    }
}

/// Compare two [TimingValue]-sequences. Returns a list of textual reasons
/// for any mismatches past tolerance.
fn compare_value_lists(
    label: &str,
    a: &[(u32, f64, f64, f64)],
    b: &[(u32, f64, f64, f64)],
    opts: &DiffOptions,
) -> Vec<String> {
    let mut reasons = Vec::new();
    if a.len() != b.len() {
        reasons.push(format!(
            "{label}: length differs (a={}, b={})",
            a.len(),
            b.len()
        ));
        return reasons;
    }
    for (a_v, b_v) in a.iter().zip(b.iter()) {
        if a_v.0 != b_v.0 {
            reasons.push(format!(
                "{label}: corner_index differs at position (a={}, b={})",
                a_v.0, b_v.0
            ));
            continue;
        }
        for (field_label, a_f, b_f) in [
            ("min", a_v.1, b_v.1),
            ("typ", a_v.2, b_v.2),
            ("max", a_v.3, b_v.3),
        ] {
            if !opts.within_tolerance(a_f, b_f) {
                reasons.push(format!(
                    "{label}[corner {}] {}: {:.3} vs {:.3} (diff {:.3} ps)",
                    a_v.0,
                    field_label,
                    a_f,
                    b_f,
                    (a_f - b_f).abs()
                ));
            }
        }
    }
    reasons
}

/// Diff one annotation category given pre-built key→value maps.
///
/// Encapsulates the only-in-A / only-in-B / value-mismatch classification
/// shared by arcs, interconnect delays, and setup/hold checks. The
/// `compare` closure produces one textual reason per mismatch found
/// between two values that share a key.
fn finalize_diff<K, T>(
    a_map: &BTreeMap<K, T>,
    b_map: &BTreeMap<K, T>,
    mut compare: impl FnMut(&T, &T) -> Vec<String>,
) -> CategoryDiff<K>
where
    K: Ord + Clone + Serialize,
{
    let mut diff = CategoryDiff::<K>::default();
    for (key, a_t) in a_map {
        match b_map.get(key) {
            None => diff.only_in_a.push(key.clone()),
            Some(b_t) => {
                for reason in compare(a_t, b_t) {
                    diff.value_mismatches.push(ValueMismatch {
                        key: key.clone(),
                        reason,
                    });
                }
            }
        }
    }
    for key in b_map.keys() {
        if !a_map.contains_key(key) {
            diff.only_in_b.push(key.clone());
        }
    }
    diff
}

/// Compute a structured diff between two IRs.
pub fn diff_irs(a: &ir::TimingIR, b: &ir::TimingIR, opts: &DiffOptions) -> DiffReport {
    let av = a.schema_version();
    let bv = b.schema_version();
    let schema_version = SchemaVersionMatch {
        a: av
            .map(|v| (v.major(), v.minor(), v.patch()))
            .unwrap_or((0, 0, 0)),
        b: bv
            .map(|v| (v.major(), v.minor(), v.patch()))
            .unwrap_or((0, 0, 0)),
        major_matches: av.map(|v| v.major()) == bv.map(|v| v.major()),
    };

    let mut corners = CornerDiff::default();
    if let Some(v) = a.corners() {
        for i in 0..v.len() {
            corners
                .a_corner_names
                .push(v.get(i).name().unwrap_or("").to_string());
        }
    }
    if let Some(v) = b.corners() {
        for i in 0..v.len() {
            corners
                .b_corner_names
                .push(v.get(i).name().unwrap_or("").to_string());
        }
    }
    if corners.a_corner_names != corners.b_corner_names {
        corners.mismatches.push(format!(
            "corner names/order differ: {:?} vs {:?}",
            corners.a_corner_names, corners.b_corner_names
        ));
    }

    let timing_arcs = diff_arcs(a, b, opts);
    let interconnect_delays = diff_interconnects(a, b, opts);
    let setup_hold_checks = diff_setup_hold(a, b, opts);
    let clock_arrivals = diff_clock_arrivals(a, b, opts);

    let a_ext_count = a.vendor_extensions().map(|v| v.len()).unwrap_or(0);
    let b_ext_count = b.vendor_extensions().map(|v| v.len()).unwrap_or(0);

    DiffReport {
        schema_version,
        corners,
        timing_arcs,
        interconnect_delays,
        setup_hold_checks,
        clock_arrivals,
        vendor_extension_counts: (a_ext_count, b_ext_count),
        options: (*opts).into(),
    }
}

fn diff_arcs(a: &ir::TimingIR, b: &ir::TimingIR, opts: &DiffOptions) -> CategoryDiff<ArcKey> {
    let a_map = build_arc_map(a);
    let b_map = build_arc_map(b);
    finalize_diff(&a_map, &b_map, |a_arc, b_arc| {
        let a_rise = collect_timing_values(a_arc.rise_delay());
        let b_rise = collect_timing_values(b_arc.rise_delay());
        let a_fall = collect_timing_values(a_arc.fall_delay());
        let b_fall = collect_timing_values(b_arc.fall_delay());
        let mut reasons = compare_value_lists("rise_delay", &a_rise, &b_rise, opts);
        reasons.extend(compare_value_lists("fall_delay", &a_fall, &b_fall, opts));
        reasons
    })
}

fn diff_interconnects(
    a: &ir::TimingIR,
    b: &ir::TimingIR,
    opts: &DiffOptions,
) -> CategoryDiff<InterconnectKey> {
    let a_map = build_interconnect_map(a);
    let b_map = build_interconnect_map(b);
    finalize_diff(&a_map, &b_map, |a_d, b_d| {
        let a_v = collect_timing_values(a_d.delay());
        let b_v = collect_timing_values(b_d.delay());
        compare_value_lists("delay", &a_v, &b_v, opts)
    })
}

fn diff_setup_hold(
    a: &ir::TimingIR,
    b: &ir::TimingIR,
    opts: &DiffOptions,
) -> CategoryDiff<SetupHoldKey> {
    let a_map = build_setup_hold_map(a);
    let b_map = build_setup_hold_map(b);
    finalize_diff(&a_map, &b_map, |a_c, b_c| {
        let a_s = collect_timing_values(a_c.setup());
        let b_s = collect_timing_values(b_c.setup());
        let a_h = collect_timing_values(a_c.hold());
        let b_h = collect_timing_values(b_c.hold());
        let mut reasons = compare_value_lists("setup", &a_s, &b_s, opts);
        reasons.extend(compare_value_lists("hold", &a_h, &b_h, opts));
        reasons
    })
}

fn diff_clock_arrivals(
    a: &ir::TimingIR,
    b: &ir::TimingIR,
    opts: &DiffOptions,
) -> CategoryDiff<ClockArrivalKey> {
    let a_map = build_clock_arrival_map(a);
    let b_map = build_clock_arrival_map(b);
    finalize_diff(&a_map, &b_map, |a_a, b_a| {
        let a_v = collect_timing_values(a_a.arrival());
        let b_v = collect_timing_values(b_a.arrival());
        compare_value_lists("arrival", &a_v, &b_v, opts)
    })
}

fn build_arc_map<'a>(ir: &'a ir::TimingIR) -> BTreeMap<ArcKey, ir::TimingArc<'a>> {
    let mut map = BTreeMap::new();
    if let Some(arcs) = ir.timing_arcs() {
        for i in 0..arcs.len() {
            let arc = arcs.get(i);
            map.insert(arc_key(&arc), arc);
        }
    }
    map
}

fn build_interconnect_map<'a>(
    ir: &'a ir::TimingIR,
) -> BTreeMap<InterconnectKey, ir::InterconnectDelay<'a>> {
    let mut map = BTreeMap::new();
    if let Some(v) = ir.interconnect_delays() {
        for i in 0..v.len() {
            let d = v.get(i);
            map.insert(interconnect_key(&d), d);
        }
    }
    map
}

fn build_setup_hold_map<'a>(
    ir: &'a ir::TimingIR,
) -> BTreeMap<SetupHoldKey, ir::SetupHoldCheck<'a>> {
    let mut map = BTreeMap::new();
    if let Some(v) = ir.setup_hold_checks() {
        for i in 0..v.len() {
            let c = v.get(i);
            map.insert(setup_hold_key(&c), c);
        }
    }
    map
}

fn build_clock_arrival_map<'a>(
    ir: &'a ir::TimingIR,
) -> BTreeMap<ClockArrivalKey, ir::ClockArrival<'a>> {
    let mut map = BTreeMap::new();
    if let Some(v) = ir.clock_arrivals() {
        for i in 0..v.len() {
            let a = v.get(i);
            map.insert(clock_arrival_key(&a), a);
        }
    }
    map
}

impl std::fmt::Display for DiffReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "timing-ir diff (tolerance: {:.3} ps abs, {:.3} rel)",
            self.options.absolute_ps, self.options.relative
        )?;
        writeln!(
            f,
            "  schema version: a={:?}, b={:?}, major_matches={}",
            self.schema_version.a, self.schema_version.b, self.schema_version.major_matches
        )?;
        if !self.corners.is_clean() {
            writeln!(f, "  corners:")?;
            for m in &self.corners.mismatches {
                writeln!(f, "    {m}")?;
            }
        }
        write_category(f, "timing arcs", &self.timing_arcs)?;
        write_category(f, "interconnect delays", &self.interconnect_delays)?;
        write_category(f, "setup/hold checks", &self.setup_hold_checks)?;
        write_category(f, "clock arrivals", &self.clock_arrivals)?;
        writeln!(
            f,
            "  vendor extensions: a={}, b={}",
            self.vendor_extension_counts.0, self.vendor_extension_counts.1
        )?;
        writeln!(
            f,
            "result: {}",
            if self.is_clean() { "CLEAN" } else { "DIFFS" }
        )?;
        Ok(())
    }
}

fn write_category<K: std::fmt::Display + Serialize>(
    f: &mut std::fmt::Formatter<'_>,
    label: &str,
    cat: &CategoryDiff<K>,
) -> std::fmt::Result {
    if cat.is_clean() {
        writeln!(f, "  {label}: clean")?;
        return Ok(());
    }
    writeln!(f, "  {label}:")?;
    for k in &cat.only_in_a {
        writeln!(f, "    only in A: {k}")?;
    }
    for k in &cat.only_in_b {
        writeln!(f, "    only in B: {k}")?;
    }
    for m in &cat.value_mismatches {
        writeln!(f, "    mismatch: {} — {}", m.key, m.reason)?;
    }
    Ok(())
}
