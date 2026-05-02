//! Build a timing-IR FlatBuffers document from parsed dump records.
//!
//! Phase 0 WS2 wires up corners, timing arcs, setup/hold checks, and
//! interconnect (wire) delays. Vendor extensions are still parsed and
//! counted here but not yet emitted.

use flatbuffers::{FlatBufferBuilder, WIPOffset};
use timing_ir as ir;

use crate::dump::{
    ArcRecord, ClockArrivalRecord, CornerRecord, DumpDocument, DumpRecord, Edge,
    InterconnectRecord, Origin, SetupHoldRecord,
};

/// Counts of records seen during build, useful for the WS5
/// `--min-arcs` parser-success assertion in the binary.
#[derive(Clone, Copy, Debug, Default)]
pub struct BuildStats {
    pub corners: usize,
    pub arcs: usize,
    pub interconnects: usize,
    pub setup_hold_checks: usize,
    pub clock_arrivals: usize,
    pub vendor_extensions: usize,
}

/// Build a finished IR buffer from a parsed dump document.
///
/// `generator_version` is the `opensta-to-ir` crate version (typically
/// `env!("CARGO_PKG_VERSION")` from the binary's `main.rs`).
pub fn build_ir(doc: &DumpDocument, generator_version: &str) -> (Vec<u8>, BuildStats) {
    let mut b = FlatBufferBuilder::with_capacity(4096);
    let mut stats = BuildStats::default();

    let corner_records: Vec<&CornerRecord> = doc
        .records
        .iter()
        .filter_map(|r| match r {
            DumpRecord::Corner(c) => Some(c),
            _ => None,
        })
        .collect();
    stats.corners = corner_records.len();

    let mut corner_offsets = Vec::with_capacity(corner_records.len());
    for c in &corner_records {
        let name = b.create_string(&c.name);
        let process = b.create_string(&c.process);
        corner_offsets.push(ir::Corner::create(
            &mut b,
            &ir::CornerArgs {
                name: Some(name),
                process: Some(process),
                voltage: c.voltage as f32,
                temperature: c.temperature as f32,
            },
        ));
    }
    let corners_vec = b.create_vector(&corner_offsets);

    // Group records by their identity key. Multi-corner producers emit
    // one record per (key, corner) combination; the builder dedupes
    // them into a single IR record carrying a per-corner `[TimingValue]`
    // vector. Single-corner producers emit one record per key, which
    // collapses to a one-entry vector — same shape, no special case.
    use indexmap::IndexMap;
    let mut arcs_by_key: IndexMap<(String, String, String, String), Vec<&ArcRecord>> =
        IndexMap::new();
    let mut interconnects_by_key: IndexMap<(String, String, String), Vec<&InterconnectRecord>> =
        IndexMap::new();
    let mut setup_hold_by_key: IndexMap<
        (String, String, String, Edge, String),
        Vec<&SetupHoldRecord>,
    > = IndexMap::new();
    let mut clock_arrivals_by_key: IndexMap<(String, String), Vec<&ClockArrivalRecord>> =
        IndexMap::new();

    for record in &doc.records {
        match record {
            DumpRecord::Arc(arc) => {
                arcs_by_key
                    .entry((
                        arc.cell_instance.clone(),
                        arc.driver_pin.clone(),
                        arc.load_pin.clone(),
                        arc.condition.clone(),
                    ))
                    .or_default()
                    .push(arc);
            }
            DumpRecord::SetupHold(check) => {
                setup_hold_by_key
                    .entry((
                        check.cell_instance.clone(),
                        check.d_pin.clone(),
                        check.clk_pin.clone(),
                        check.edge,
                        check.condition.clone(),
                    ))
                    .or_default()
                    .push(check);
            }
            DumpRecord::Interconnect(ic) => {
                interconnects_by_key
                    .entry((ic.net.clone(), ic.from_pin.clone(), ic.to_pin.clone()))
                    .or_default()
                    .push(ic);
            }
            DumpRecord::ClockArrival(ca) => {
                clock_arrivals_by_key
                    .entry((ca.cell_instance.clone(), ca.clk_pin.clone()))
                    .or_default()
                    .push(ca);
            }
            DumpRecord::VendorExt(_) => stats.vendor_extensions += 1,
            DumpRecord::Corner(_) => {} // already handled
        }
    }
    stats.arcs = arcs_by_key.len();
    stats.setup_hold_checks = setup_hold_by_key.len();
    stats.interconnects = interconnects_by_key.len();
    stats.clock_arrivals = clock_arrivals_by_key.len();

    let arc_offsets: Vec<_> = arcs_by_key
        .values()
        .map(|arcs| build_arc(&mut b, arcs))
        .collect();
    let setup_hold_offsets: Vec<_> = setup_hold_by_key
        .values()
        .map(|checks| build_setup_hold(&mut b, checks))
        .collect();
    let interconnect_offsets: Vec<_> = interconnects_by_key
        .values()
        .map(|ics| build_interconnect(&mut b, ics))
        .collect();
    let clock_arrival_offsets: Vec<_> = clock_arrivals_by_key
        .values()
        .map(|cas| build_clock_arrival(&mut b, cas))
        .collect();
    let arcs_vec = b.create_vector(&arc_offsets);
    let setup_hold_vec = b.create_vector(&setup_hold_offsets);
    let interconnect_vec = b.create_vector(&interconnect_offsets);
    let clock_arrivals_vec = b.create_vector(&clock_arrival_offsets);

    // Vendor extensions still pass through as count-only.
    let vendor_ext_vec = b.create_vector::<WIPOffset<ir::VendorExtension>>(&[]);

    let generator_tool = b.create_string(&format!("opensta-to-ir {generator_version}"));
    let generator_version_str = b.create_string(generator_version);
    let input_file_strs: Vec<_> = doc
        .header
        .input_files
        .iter()
        .map(|s| b.create_string(s))
        .collect();
    let input_files_vec = b.create_vector(&input_file_strs);

    let schema_version =
        ir::SchemaVersion::new(ir::SCHEMA_MAJOR, ir::SCHEMA_MINOR, ir::SCHEMA_PATCH);

    let root = ir::TimingIR::create(
        &mut b,
        &ir::TimingIRArgs {
            schema_version: Some(&schema_version),
            corners: Some(corners_vec),
            timing_arcs: Some(arcs_vec),
            interconnect_delays: Some(interconnect_vec),
            setup_hold_checks: Some(setup_hold_vec),
            clock_arrivals: Some(clock_arrivals_vec),
            vendor_extensions: Some(vendor_ext_vec),
            generator_tool: Some(generator_tool),
            generator_version: Some(generator_version_str),
            input_files: Some(input_files_vec),
        },
    );
    ir::finish_timing_ir_buffer(&mut b, root);

    (b.finished_data().to_vec(), stats)
}

fn build_arc<'a>(
    b: &mut FlatBufferBuilder<'a>,
    arcs: &[&ArcRecord],
) -> WIPOffset<ir::TimingArc<'a>> {
    let head = arcs[0];
    let cell_instance = b.create_string(&head.cell_instance);
    let driver_pin = b.create_string(&head.driver_pin);
    let load_pin = b.create_string(&head.load_pin);
    let condition = b.create_string(&head.condition);

    let rise: Vec<_> = arcs
        .iter()
        .map(|a| ir::TimingValue::new(a.corner_index, a.rise_min, a.rise_typ, a.rise_max))
        .collect();
    let fall: Vec<_> = arcs
        .iter()
        .map(|a| ir::TimingValue::new(a.corner_index, a.fall_min, a.fall_typ, a.fall_max))
        .collect();
    let rise_vec = b.create_vector(&rise);
    let fall_vec = b.create_vector(&fall);

    let provenance = build_provenance(b, head.origin);

    ir::TimingArc::create(
        b,
        &ir::TimingArcArgs {
            cell_instance: Some(cell_instance),
            driver_pin: Some(driver_pin),
            load_pin: Some(load_pin),
            rise_delay: Some(rise_vec),
            fall_delay: Some(fall_vec),
            condition: Some(condition),
            provenance: Some(provenance),
        },
    )
}

fn build_interconnect<'a>(
    b: &mut FlatBufferBuilder<'a>,
    ics: &[&InterconnectRecord],
) -> WIPOffset<ir::InterconnectDelay<'a>> {
    let head = ics[0];
    let net = b.create_string(&head.net);
    let from_pin = b.create_string(&head.from_pin);
    let to_pin = b.create_string(&head.to_pin);

    let delay: Vec<_> = ics
        .iter()
        .map(|ic| ir::TimingValue::new(ic.corner_index, ic.min, ic.typ, ic.max))
        .collect();
    let delay_vec = b.create_vector(&delay);

    let provenance = build_provenance(b, head.origin);

    ir::InterconnectDelay::create(
        b,
        &ir::InterconnectDelayArgs {
            net: Some(net),
            from_pin: Some(from_pin),
            to_pin: Some(to_pin),
            delay: Some(delay_vec),
            provenance: Some(provenance),
        },
    )
}

fn build_clock_arrival<'a>(
    b: &mut FlatBufferBuilder<'a>,
    cas: &[&ClockArrivalRecord],
) -> WIPOffset<ir::ClockArrival<'a>> {
    let head = cas[0];
    let cell_instance = b.create_string(&head.cell_instance);
    let clk_pin = b.create_string(&head.clk_pin);

    let arrival: Vec<_> = cas
        .iter()
        .map(|ca| ir::TimingValue::new(ca.corner_index, ca.min, ca.typ, ca.max))
        .collect();
    let arrival_vec = b.create_vector(&arrival);

    let provenance = build_provenance(b, head.origin);

    ir::ClockArrival::create(
        b,
        &ir::ClockArrivalArgs {
            cell_instance: Some(cell_instance),
            clk_pin: Some(clk_pin),
            arrival: Some(arrival_vec),
            provenance: Some(provenance),
        },
    )
}

fn build_setup_hold<'a>(
    b: &mut FlatBufferBuilder<'a>,
    checks: &[&SetupHoldRecord],
) -> WIPOffset<ir::SetupHoldCheck<'a>> {
    let head = checks[0];
    let cell_instance = b.create_string(&head.cell_instance);
    let d_pin = b.create_string(&head.d_pin);
    let clk_pin = b.create_string(&head.clk_pin);
    let condition = b.create_string(&head.condition);

    let setup: Vec<_> = checks
        .iter()
        .map(|c| {
            ir::TimingValue::new(c.corner_index, c.setup_min, c.setup_typ, c.setup_max)
        })
        .collect();
    let hold: Vec<_> = checks
        .iter()
        .map(|c| ir::TimingValue::new(c.corner_index, c.hold_min, c.hold_typ, c.hold_max))
        .collect();
    let setup_vec = b.create_vector(&setup);
    let hold_vec = b.create_vector(&hold);

    let provenance = build_provenance(b, head.origin);

    let edge = match head.edge {
        Edge::Posedge => ir::CheckEdge::Posedge,
        Edge::Negedge => ir::CheckEdge::Negedge,
    };

    ir::SetupHoldCheck::create(
        b,
        &ir::SetupHoldCheckArgs {
            cell_instance: Some(cell_instance),
            d_pin: Some(d_pin),
            clk_pin: Some(clk_pin),
            edge,
            setup: Some(setup_vec),
            hold: Some(hold_vec),
            condition: Some(condition),
            provenance: Some(provenance),
        },
    )
}

fn build_provenance<'a>(
    b: &mut FlatBufferBuilder<'a>,
    origin: Origin,
) -> WIPOffset<ir::Provenance<'a>> {
    let source_tool = b.create_string("opensta-to-ir");
    let source_file = b.create_string("");
    let ir_origin = match origin {
        Origin::Asserted => ir::Origin::Asserted,
        Origin::Computed => ir::Origin::Computed,
        Origin::Defaulted => ir::Origin::Defaulted,
    };
    ir::Provenance::create(
        b,
        &ir::ProvenanceArgs {
            source_tool: Some(source_tool),
            source_file: Some(source_file),
            origin: ir_origin,
        },
    )
}
