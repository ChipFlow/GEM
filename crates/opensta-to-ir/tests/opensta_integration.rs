//! End-to-end integration test: invokes a real OpenSTA build and parses
//! its dump.
//!
//! Gated on `opensta_to_ir::opensta::find_opensta` returning `Some`. When
//! the OpenSTA binary is unbuilt, the test reports skip and exits clean
//! rather than failing. Build OpenSTA with `scripts/build-opensta.sh`.
//!
//! Phase 0 WS2 (Phase 2.1-followup-A) Tcl driver emits a stub dump (one
//! default corner, no arcs). This test only verifies that the subprocess
//! pipe is plumbed correctly. Real timing extraction is the next slice
//! and will replace this test's assertions.

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
fn invokes_real_opensta_with_stub_tcl() {
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
        // Stub Tcl emits 0 arcs; relax the success-assertion floor.
        .arg("--allow-empty-parse")
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
    assert_eq!(
        corners.len(),
        1,
        "Phase 2.1-followup-A emits one default corner"
    );
    assert_eq!(corners.get(0).name(), Some("default"));

    // Phase 2.1-followup-B will populate timing arcs; this test is
    // intentionally permissive about that for now.
    let arcs = ir.timing_arcs().expect("arcs vector present");
    assert_eq!(arcs.len(), 0);
}
