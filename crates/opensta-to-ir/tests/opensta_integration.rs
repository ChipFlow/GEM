//! End-to-end integration test: invokes a real OpenSTA build and parses
//! its dump.
//!
//! Gated on `opensta_to_ir::opensta::find_opensta` returning `Some`. When
//! the OpenSTA binary is unbuilt, the test reports skip and exits clean
//! rather than failing. Build OpenSTA with `scripts/build-opensta.sh`.

use std::path::Path;
use std::process::Command;

use opensta_to_ir::opensta::find_opensta;
use tempfile::TempDir;
use timing_ir::root_as_timing_ir;

const TINY_VERILOG: &str = r#"
module tiny(A, B, Y);
  input A, B;
  output Y;
  AND2_00_0 u1 (.A(A), .B(B), .Y(Y));
endmodule
"#;

const DFF_VERILOG: &str = r#"
module dff_test(CLK, D, Q);
  input CLK, D;
  output Q;
  DFF d1 (.CLK(CLK), .D(D), .Q(Q));
endmodule
"#;

fn bin() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_opensta-to-ir"))
}

fn aigpdk_lib() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("aigpdk/aigpdk_nomem.lib")
}

#[test]
fn aigpdk_and2_emits_two_arcs() {
    let Some(_sta) = find_opensta(None) else {
        eprintln!("skipping: OpenSTA not built; run scripts/build-opensta.sh");
        return;
    };

    let dir = TempDir::new().unwrap();
    let v_path = dir.path().join("tiny.v");
    let out_path = dir.path().join("tiny.jtir");
    std::fs::write(&v_path, TINY_VERILOG).unwrap();

    let lib = aigpdk_lib();
    assert!(lib.exists(), "AIGPDK Liberty missing at {}", lib.display());

    let output = Command::new(bin())
        .arg("--liberty")
        .arg(&lib)
        .arg("--verilog")
        .arg(&v_path)
        .arg("--top")
        .arg("tiny")
        .arg("--output")
        .arg(&out_path)
        .output()
        .expect("run opensta-to-ir");

    assert_eq!(
        output.status.code(),
        Some(0),
        "stderr: {}\nstdout: {}",
        String::from_utf8_lossy(&output.stderr),
        String::from_utf8_lossy(&output.stdout),
    );

    let buf = std::fs::read(&out_path).expect("output IR written");
    let ir = root_as_timing_ir(&buf).expect("readable IR");

    let corners = ir.corners().expect("corners present");
    assert_eq!(corners.len(), 1);
    assert_eq!(corners.get(0).name(), Some("default"));

    // AND2_00_0 has two combinational paths: A→Y and B→Y. AIGPDK Liberty
    // assigns 1 ps to each, so we expect two arcs both at 1 ps rise/fall.
    let arcs = ir.timing_arcs().expect("arcs vector present");
    assert_eq!(arcs.len(), 2, "expected A→Y and B→Y arcs from AND2_00_0");

    let mut driver_pins: Vec<String> = (0..arcs.len())
        .map(|i| arcs.get(i).driver_pin().unwrap_or("").to_string())
        .collect();
    driver_pins.sort();
    assert_eq!(driver_pins, vec!["A".to_string(), "B".to_string()]);

    for i in 0..arcs.len() {
        let arc = arcs.get(i);
        assert_eq!(arc.cell_instance(), Some("u1"));
        assert_eq!(arc.load_pin(), Some("Y"));
        let rise = arc.rise_delay().unwrap();
        let r = rise.get(0);
        // AIGPDK 1ps with float scaling — accept anything up to 5 ps.
        assert!(r.max() < 5.0, "arc {} rise_max {} too large", i, r.max());
        assert!(r.max() > 0.0, "arc {} rise_max should be non-zero", i);
    }
}

#[test]
fn aigpdk_dff_emits_setup_hold_records() {
    let Some(_sta) = find_opensta(None) else {
        eprintln!("skipping: OpenSTA not built; run scripts/build-opensta.sh");
        return;
    };

    let dir = TempDir::new().unwrap();
    let v_path = dir.path().join("dff.v");
    let out_path = dir.path().join("dff.jtir");
    std::fs::write(&v_path, DFF_VERILOG).unwrap();

    let lib = aigpdk_lib();
    let output = Command::new(bin())
        .arg("--liberty")
        .arg(&lib)
        .arg("--verilog")
        .arg(&v_path)
        .arg("--top")
        .arg("dff_test")
        .arg("--output")
        .arg(&out_path)
        .output()
        .expect("run opensta-to-ir");

    assert_eq!(
        output.status.code(),
        Some(0),
        "stderr: {}\nstdout: {}",
        String::from_utf8_lossy(&output.stderr),
        String::from_utf8_lossy(&output.stdout),
    );

    let buf = std::fs::read(&out_path).expect("output IR written");
    let ir = root_as_timing_ir(&buf).expect("readable IR");

    // Expect at least one CLK→Q delay arc.
    let arcs = ir.timing_arcs().expect("arcs vector present");
    assert!(!arcs.is_empty(), "expected CLK→Q arc, got 0");

    // Setup/hold checks for the DFF — AIGPDK Liberty defines both rising
    // and falling, so we expect 2 records keyed by edge.
    let checks = ir.setup_hold_checks().expect("setup_hold vector present");
    assert!(
        !checks.is_empty(),
        "expected at least one SETUP_HOLD record, got 0"
    );

    let c0 = checks.get(0);
    assert_eq!(c0.cell_instance(), Some("d1"));
    assert_eq!(c0.d_pin(), Some("D"));
    assert_eq!(c0.clk_pin(), Some("CLK"));
    let setup = c0.setup().expect("setup values");
    assert_eq!(setup.len(), 1, "single corner");
    let hold = c0.hold().expect("hold values");
    assert_eq!(hold.len(), 1, "single corner");
}
