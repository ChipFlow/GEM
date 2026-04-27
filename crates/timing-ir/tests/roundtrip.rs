//! Round-trip: build a small IR, serialize, deserialize, assert equality.
//!
//! This is the WS1 acceptance test from `docs/plans/phase-0-ir-and-oracle.md` —
//! schema round-trip works.

use flatbuffers::FlatBufferBuilder;
use timing_ir::*;

#[test]
fn roundtrip_minimal_ir() {
    let mut builder = FlatBufferBuilder::with_capacity(1024);

    let corner_name = builder.create_string("tt_25C_1v80");
    let corner_process = builder.create_string("tt");
    let corner = Corner::create(
        &mut builder,
        &CornerArgs {
            name: Some(corner_name),
            process: Some(corner_process),
            voltage: 1.80,
            temperature: 25.0,
        },
    );
    let corners_vec = builder.create_vector(&[corner]);

    let prov_tool = builder.create_string("timing-ir-test");
    let prov_file = builder.create_string("synthetic://roundtrip");
    let provenance = Provenance::create(
        &mut builder,
        &ProvenanceArgs {
            source_tool: Some(prov_tool),
            source_file: Some(prov_file),
            origin: Origin::Asserted,
        },
    );

    let cell_instance = builder.create_string("u1");
    let driver_pin = builder.create_string("A");
    let load_pin = builder.create_string("Y");
    let condition = builder.create_string("");

    let rise_values = [TimingValue::new(0, 100.0, 120.0, 140.0)];
    let fall_values = [TimingValue::new(0, 110.0, 130.0, 150.0)];
    let rise_delay = builder.create_vector(&rise_values);
    let fall_delay = builder.create_vector(&fall_values);

    let arc = TimingArc::create(
        &mut builder,
        &TimingArcArgs {
            cell_instance: Some(cell_instance),
            driver_pin: Some(driver_pin),
            load_pin: Some(load_pin),
            rise_delay: Some(rise_delay),
            fall_delay: Some(fall_delay),
            condition: Some(condition),
            provenance: Some(provenance),
        },
    );
    let arcs_vec = builder.create_vector(&[arc]);

    let generator_tool = builder.create_string("timing-ir-roundtrip");
    let generator_version = builder.create_string("0.1.0");
    let input_file = builder.create_string("synthetic://roundtrip.sdf");
    let input_files_vec = builder.create_vector(&[input_file]);

    let schema_version = SchemaVersion::new(SCHEMA_MAJOR, SCHEMA_MINOR, SCHEMA_PATCH);

    let ir = TimingIR::create(
        &mut builder,
        &TimingIRArgs {
            schema_version: Some(&schema_version),
            corners: Some(corners_vec),
            timing_arcs: Some(arcs_vec),
            interconnect_delays: None,
            setup_hold_checks: None,
            vendor_extensions: None,
            generator_tool: Some(generator_tool),
            generator_version: Some(generator_version),
            input_files: Some(input_files_vec),
        },
    );
    finish_timing_ir_buffer(&mut builder, ir);

    let buf = builder.finished_data();

    let ir_back = root_as_timing_ir(buf).expect("buffer must be a valid TimingIR");

    let v = ir_back.schema_version().expect("schema_version present");
    assert_eq!(v.major(), SCHEMA_MAJOR);
    assert_eq!(v.minor(), SCHEMA_MINOR);
    assert_eq!(v.patch(), SCHEMA_PATCH);

    assert_eq!(ir_back.generator_tool(), Some("timing-ir-roundtrip"));
    assert_eq!(ir_back.generator_version(), Some("0.1.0"));
    let input_files = ir_back.input_files().expect("input_files present");
    assert_eq!(input_files.len(), 1);
    assert_eq!(input_files.get(0), "synthetic://roundtrip.sdf");

    let corners = ir_back.corners().expect("corners present");
    assert_eq!(corners.len(), 1);
    let c0 = corners.get(0);
    assert_eq!(c0.name(), Some("tt_25C_1v80"));
    assert_eq!(c0.process(), Some("tt"));
    assert!((c0.voltage() - 1.80).abs() < 1e-6);
    assert!((c0.temperature() - 25.0).abs() < 1e-6);

    let arcs = ir_back.timing_arcs().expect("arcs present");
    assert_eq!(arcs.len(), 1);
    let arc0 = arcs.get(0);
    assert_eq!(arc0.cell_instance(), Some("u1"));
    assert_eq!(arc0.driver_pin(), Some("A"));
    assert_eq!(arc0.load_pin(), Some("Y"));
    assert_eq!(arc0.condition(), Some(""));

    let rise = arc0.rise_delay().expect("rise_delay present");
    assert_eq!(rise.len(), 1);
    let r0 = rise.get(0);
    assert_eq!(r0.corner_index(), 0);
    assert!((r0.min() - 100.0).abs() < 1e-6);
    assert!((r0.typ() - 120.0).abs() < 1e-6);
    assert!((r0.max() - 140.0).abs() < 1e-6);

    let fall = arc0.fall_delay().expect("fall_delay present");
    assert_eq!(fall.len(), 1);
    let f0 = fall.get(0);
    assert!((f0.min() - 110.0).abs() < 1e-6);
    assert!((f0.typ() - 130.0).abs() < 1e-6);
    assert!((f0.max() - 150.0).abs() < 1e-6);

    let prov = arc0.provenance().expect("provenance present");
    assert_eq!(prov.source_tool(), Some("timing-ir-test"));
    assert_eq!(prov.source_file(), Some("synthetic://roundtrip"));
    assert_eq!(prov.origin(), Origin::Asserted);
}

#[test]
fn file_identifier_constant_matches_schema() {
    // Sanity check that the crate's FILE_IDENTIFIER stays in sync with the
    // schema's file_identifier "JTIR" directive.
    assert_eq!(FILE_IDENTIFIER, "JTIR");
    assert_eq!(
        FILE_IDENTIFIER.len(),
        4,
        "FlatBuffers file identifiers must be 4 bytes"
    );
}
