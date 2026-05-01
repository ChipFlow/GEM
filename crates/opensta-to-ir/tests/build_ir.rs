//! Integration test: dump → IR builder → readable FlatBuffers.

use opensta_to_ir::builder::build_ir;
use opensta_to_ir::dump::parse_dump;
use timing_ir::root_as_timing_ir;

const FIXTURE: &str = "\
# format-version: 1
# generator-tool: opensta-to-ir 0.1.0
# generator-opensta: OpenSTA fake-version
# input-files: a.lib, b.v
CORNER\t0\ttt_25C_1v80\ttt\t1.80\t25.0
ARC\tu1\tA\tY\t0\t100.0\t120.0\t140.0\t110.0\t130.0\t150.0\t\tAsserted
ARC\tu2\tA\tY\t0\t90.0\t110.0\t130.0\t100.0\t120.0\t140.0\t\tAsserted
# end
";

#[test]
fn dump_to_ir_roundtrip_via_flatbuffers() {
    let doc = parse_dump(FIXTURE).expect("dump parses");
    let (buf, stats) = build_ir(&doc, "0.1.0");
    assert_eq!(stats.corners, 1);
    assert_eq!(stats.arcs, 2);

    let ir = root_as_timing_ir(&buf).expect("readable IR");

    let v = ir.schema_version().unwrap();
    assert_eq!(v.major(), timing_ir::SCHEMA_MAJOR);

    let corners = ir.corners().expect("corners present");
    assert_eq!(corners.len(), 1);
    assert_eq!(corners.get(0).name(), Some("tt_25C_1v80"));

    let arcs = ir.timing_arcs().expect("arcs present");
    assert_eq!(arcs.len(), 2);
    let a0 = arcs.get(0);
    assert_eq!(a0.cell_instance(), Some("u1"));
    let rise = a0.rise_delay().unwrap();
    assert_eq!(rise.len(), 1);
    let r = rise.get(0);
    assert_eq!(r.corner_index(), 0);
    assert!((r.max() - 140.0).abs() < 1e-6);

    assert_eq!(ir.generator_tool(), Some("opensta-to-ir 0.1.0"));
    let inputs = ir.input_files().expect("input_files present");
    assert_eq!(inputs.len(), 2);
}

#[test]
fn interconnect_records_are_emitted_to_ir() {
    let dump = "\
# format-version: 1
CORNER\t0\tdefault\ttt\t1.0\t25.0
ARC\tu1\tA\tY\t0\t100.0\t120.0\t140.0\t110.0\t130.0\t150.0\t\tAsserted
INTERCONNECT\tn1\tu1/Y\tu2/A\t0\t10.0\t15.0\t20.0\tAsserted
INTERCONNECT\tn2\tu2/Y\tu3/A\t0\t5.5\t8.0\t12.5\tAsserted
# end
";
    let doc = parse_dump(dump).expect("parses");
    let (buf, stats) = build_ir(&doc, "0.1.0");
    assert_eq!(stats.interconnects, 2);

    let ir = root_as_timing_ir(&buf).expect("readable IR");
    let ics = ir
        .interconnect_delays()
        .expect("interconnect_delays present");
    assert_eq!(ics.len(), 2);

    let ic0 = ics.get(0);
    assert_eq!(ic0.net(), Some("n1"));
    assert_eq!(ic0.from_pin(), Some("u1/Y"));
    assert_eq!(ic0.to_pin(), Some("u2/A"));
    let d0 = ic0.delay().unwrap();
    assert_eq!(d0.len(), 1);
    let v0 = d0.get(0);
    assert_eq!(v0.corner_index(), 0);
    assert!((v0.min() - 10.0).abs() < 1e-6);
    assert!((v0.typ() - 15.0).abs() < 1e-6);
    assert!((v0.max() - 20.0).abs() < 1e-6);

    let ic1 = ics.get(1);
    assert_eq!(ic1.net(), Some("n2"));
    assert!((ic1.delay().unwrap().get(0).typ() - 8.0).abs() < 1e-6);
}

#[test]
fn clock_arrival_records_are_emitted_to_ir() {
    let dump = "\
# format-version: 1
CORNER\t0\tdefault\ttt\t1.0\t25.0
CLOCK_ARRIVAL\tu_dff_0\tCLK\t0\t180.0\t195.0\t210.0\tComputed
CLOCK_ARRIVAL\tu_dff_1\tCLK\t0\t220.5\t240.0\t260.0\tComputed
# end
";
    let doc = parse_dump(dump).expect("parses");
    let (buf, stats) = build_ir(&doc, "0.1.0");
    assert_eq!(stats.clock_arrivals, 2);

    let ir = root_as_timing_ir(&buf).expect("readable IR");
    let cas = ir.clock_arrivals().expect("clock_arrivals present");
    assert_eq!(cas.len(), 2);

    let ca0 = cas.get(0);
    assert_eq!(ca0.cell_instance(), Some("u_dff_0"));
    assert_eq!(ca0.clk_pin(), Some("CLK"));
    let arr0 = ca0.arrival().unwrap();
    assert_eq!(arr0.len(), 1);
    let v0 = arr0.get(0);
    assert_eq!(v0.corner_index(), 0);
    assert!((v0.min() - 180.0).abs() < 1e-6);
    assert!((v0.typ() - 195.0).abs() < 1e-6);
    assert!((v0.max() - 210.0).abs() < 1e-6);

    let ca1 = cas.get(1);
    assert_eq!(ca1.cell_instance(), Some("u_dff_1"));
    assert!((ca1.arrival().unwrap().get(0).max() - 260.0).abs() < 1e-6);
}

#[test]
fn empty_arc_list_produces_zero_arcs_in_ir() {
    let dump = "\
# format-version: 1
CORNER\t0\tdefault\ttt\t1.0\t25.0
# end
";
    let doc = parse_dump(dump).expect("parses");
    let (buf, stats) = build_ir(&doc, "0.1.0");
    assert_eq!(stats.arcs, 0);
    let ir = root_as_timing_ir(&buf).expect("readable IR");
    assert_eq!(ir.timing_arcs().unwrap().len(), 0);
}
