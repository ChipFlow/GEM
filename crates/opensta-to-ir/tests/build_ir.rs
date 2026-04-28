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
