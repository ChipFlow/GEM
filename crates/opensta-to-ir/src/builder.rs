//! Build a timing-IR FlatBuffers document from parsed dump records.
//!
//! Phase 0 WS2 (Phase 2.3) wires up corners, timing arcs, and setup/hold
//! checks. Interconnect delays and vendor extensions are present in the
//! parser and counted here, but their IR emission lands in later slices.

use flatbuffers::{FlatBufferBuilder, WIPOffset};
use timing_ir as ir;

use crate::dump::{
    ArcRecord, CornerRecord, DumpDocument, DumpRecord, Edge, Origin, SetupHoldRecord,
};

/// Counts of records seen during build, useful for the WS5
/// `--min-arcs` parser-success assertion in the binary.
#[derive(Clone, Copy, Debug, Default)]
pub struct BuildStats {
    pub corners: usize,
    pub arcs: usize,
    pub interconnects: usize,
    pub setup_hold_checks: usize,
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

    let mut arc_offsets = Vec::new();
    let mut setup_hold_offsets = Vec::new();
    for record in &doc.records {
        match record {
            DumpRecord::Arc(arc) => {
                arc_offsets.push(build_arc(&mut b, arc));
                stats.arcs += 1;
            }
            DumpRecord::SetupHold(check) => {
                setup_hold_offsets.push(build_setup_hold(&mut b, check));
                stats.setup_hold_checks += 1;
            }
            DumpRecord::Interconnect(_) => stats.interconnects += 1,
            DumpRecord::VendorExt(_) => stats.vendor_extensions += 1,
            DumpRecord::Corner(_) => {} // already handled
        }
    }
    let arcs_vec = b.create_vector(&arc_offsets);
    let setup_hold_vec = b.create_vector(&setup_hold_offsets);

    // Empty vectors for record kinds not yet wired. Interconnect and
    // vendor extensions land in later slices.
    let interconnect_vec = b.create_vector::<WIPOffset<ir::InterconnectDelay>>(&[]);
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
            vendor_extensions: Some(vendor_ext_vec),
            generator_tool: Some(generator_tool),
            generator_version: Some(generator_version_str),
            input_files: Some(input_files_vec),
        },
    );
    ir::finish_timing_ir_buffer(&mut b, root);

    (b.finished_data().to_vec(), stats)
}

fn build_arc<'a>(b: &mut FlatBufferBuilder<'a>, arc: &ArcRecord) -> WIPOffset<ir::TimingArc<'a>> {
    let cell_instance = b.create_string(&arc.cell_instance);
    let driver_pin = b.create_string(&arc.driver_pin);
    let load_pin = b.create_string(&arc.load_pin);
    let condition = b.create_string(&arc.condition);

    let rise = [ir::TimingValue::new(
        arc.corner_index,
        arc.rise_min,
        arc.rise_typ,
        arc.rise_max,
    )];
    let fall = [ir::TimingValue::new(
        arc.corner_index,
        arc.fall_min,
        arc.fall_typ,
        arc.fall_max,
    )];
    let rise_vec = b.create_vector(&rise);
    let fall_vec = b.create_vector(&fall);

    let provenance = build_provenance(b, arc.origin);

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

fn build_setup_hold<'a>(
    b: &mut FlatBufferBuilder<'a>,
    check: &SetupHoldRecord,
) -> WIPOffset<ir::SetupHoldCheck<'a>> {
    let cell_instance = b.create_string(&check.cell_instance);
    let d_pin = b.create_string(&check.d_pin);
    let clk_pin = b.create_string(&check.clk_pin);
    let condition = b.create_string(&check.condition);

    let setup = [ir::TimingValue::new(
        check.corner_index,
        check.setup_min,
        check.setup_typ,
        check.setup_max,
    )];
    let hold = [ir::TimingValue::new(
        check.corner_index,
        check.hold_min,
        check.hold_typ,
        check.hold_max,
    )];
    let setup_vec = b.create_vector(&setup);
    let hold_vec = b.create_vector(&hold);

    let provenance = build_provenance(b, check.origin);

    let edge = match check.edge {
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
