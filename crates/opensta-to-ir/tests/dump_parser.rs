//! Unit tests for the dump-format parser (no OpenSTA needed).

use opensta_to_ir::dump::{parse_dump, DumpRecord, Edge, Origin};

const MINIMAL_DUMP: &str = "\
# format-version: 1
# generator-tool: opensta-to-ir 0.1.0
# generator-opensta: OpenSTA fake-version
# input-files: a.lib, b.v
CORNER\t0\ttt_25C_1v80\ttt\t1.80\t25.0
ARC\tu1\tA\tY\t0\t100.0\t120.0\t140.0\t110.0\t130.0\t150.0\t\tAsserted
# end
";

#[test]
fn minimal_dump_parses() {
    let doc = parse_dump(MINIMAL_DUMP).expect("parses");
    assert_eq!(doc.header.format_version, 1);
    assert_eq!(doc.header.generator_tool, "opensta-to-ir 0.1.0");
    assert_eq!(doc.header.input_files, vec!["a.lib", "b.v"]);
    assert_eq!(doc.records.len(), 2);

    let DumpRecord::Corner(c) = &doc.records[0] else {
        panic!("expected CORNER first");
    };
    assert_eq!(c.index, 0);
    assert_eq!(c.name, "tt_25C_1v80");
    assert!((c.voltage - 1.80).abs() < 1e-6);

    let DumpRecord::Arc(a) = &doc.records[1] else {
        panic!("expected ARC second");
    };
    assert_eq!(a.cell_instance, "u1");
    assert_eq!(a.driver_pin, "A");
    assert_eq!(a.load_pin, "Y");
    assert_eq!(a.corner_index, 0);
    assert!((a.rise_max - 140.0).abs() < 1e-6);
    assert!((a.fall_min - 110.0).abs() < 1e-6);
    assert_eq!(a.condition, "");
    assert_eq!(a.origin, Origin::Asserted);
}

#[test]
fn missing_format_version_fails() {
    let dump = "\
# generator-tool: opensta-to-ir 0.1.0
CORNER\t0\tdefault\ttt\t1.0\t25.0
# end
";
    let err = parse_dump(dump).expect_err("missing format-version");
    assert!(err.message.contains("format-version"), "got: {err}");
}

#[test]
fn wrong_format_version_fails() {
    let dump = "\
# format-version: 99
CORNER\t0\tdefault\ttt\t1.0\t25.0
# end
";
    let err = parse_dump(dump).expect_err("wrong version");
    assert!(
        err.message.contains("unsupported format-version"),
        "got: {err}"
    );
}

#[test]
fn missing_end_marker_fails() {
    let dump = "\
# format-version: 1
CORNER\t0\tdefault\ttt\t1.0\t25.0
";
    let err = parse_dump(dump).expect_err("missing # end");
    assert!(err.message.contains("# end"), "got: {err}");
}

#[test]
fn unknown_record_kind_fails() {
    let dump = "\
# format-version: 1
WIDGET\tfoo
# end
";
    let err = parse_dump(dump).expect_err("unknown record");
    assert!(err.message.contains("unknown record kind"), "got: {err}");
}

#[test]
fn arc_wrong_field_count_fails() {
    // ARC requires 12 fields after the kind; supply 11.
    let dump = "\
# format-version: 1
ARC\tu1\tA\tY\t0\t100.0\t120.0\t140.0\t110.0\t130.0\t150.0\tAsserted
# end
";
    let err = parse_dump(dump).expect_err("wrong field count");
    assert!(err.message.contains("expected 12"), "got: {err}");
}

#[test]
fn unknown_origin_fails() {
    let dump = "\
# format-version: 1
ARC\tu1\tA\tY\t0\t1.0\t2.0\t3.0\t4.0\t5.0\t6.0\t\tBogus
# end
";
    let err = parse_dump(dump).expect_err("unknown origin");
    assert!(err.message.contains("unknown origin"), "got: {err}");
}

#[test]
fn interconnect_and_setup_hold_records_parse() {
    let dump = "\
# format-version: 1
CORNER\t0\tdefault\ttt\t1.0\t25.0
INTERCONNECT\tnet1\tu1/Y\tu2/A\t0\t10.0\t12.0\t15.0\tComputed
SETUP_HOLD\tu3\tD\tCLK\tPosedge\t0\t80.0\t80.0\t80.0\t20.0\t20.0\t20.0\t\tAsserted
# end
";
    let doc = parse_dump(dump).expect("parses");
    assert_eq!(doc.records.len(), 3);

    let DumpRecord::Interconnect(ic) = &doc.records[1] else {
        panic!("expected INTERCONNECT");
    };
    assert_eq!(ic.net, "net1");
    assert_eq!(ic.origin, Origin::Computed);

    let DumpRecord::SetupHold(sh) = &doc.records[2] else {
        panic!("expected SETUP_HOLD");
    };
    assert_eq!(sh.cell_instance, "u3");
    assert_eq!(sh.edge, Edge::Posedge);
    assert!((sh.setup_typ - 80.0).abs() < 1e-6);
}

#[test]
fn clock_arrival_record_parses() {
    let dump = "\
# format-version: 1
CORNER\t0\tdefault\ttt\t1.0\t25.0
CLOCK_ARRIVAL\tu_dff_0\tCLK\t0\t180.0\t195.0\t210.0\tComputed
# end
";
    let doc = parse_dump(dump).expect("parses");
    assert_eq!(doc.records.len(), 2);

    let DumpRecord::ClockArrival(ca) = &doc.records[1] else {
        panic!("expected CLOCK_ARRIVAL");
    };
    assert_eq!(ca.cell_instance, "u_dff_0");
    assert_eq!(ca.clk_pin, "CLK");
    assert_eq!(ca.corner_index, 0);
    assert!((ca.min - 180.0).abs() < 1e-6);
    assert!((ca.max - 210.0).abs() < 1e-6);
    assert_eq!(ca.origin, Origin::Computed);
}

#[test]
fn clock_arrival_wrong_field_count_fails() {
    // CLOCK_ARRIVAL requires 7 fields after the kind; supply 6.
    let dump = "\
# format-version: 1
CLOCK_ARRIVAL\tu_dff_0\tCLK\t0\t180.0\t195.0\tComputed
# end
";
    let err = parse_dump(dump).expect_err("wrong field count");
    assert!(err.message.contains("expected 7"), "got: {err}");
}

#[test]
fn empty_input_files_list_is_empty_vec() {
    let dump = "\
# format-version: 1
# input-files:
CORNER\t0\tdefault\ttt\t1.0\t25.0
ARC\tu1\tA\tY\t0\t1.0\t2.0\t3.0\t4.0\t5.0\t6.0\t\tAsserted
# end
";
    let doc = parse_dump(dump).expect("parses");
    assert!(doc.header.input_files.is_empty());
}
