//! End-to-end CLI tests using the `--from-dump` escape hatch.
//!
//! Phase 0 WS2 (Phase 2.1): the OpenSTA subprocess invocation isn't wired
//! yet, so these tests exercise the dump → IR pipeline via `--from-dump`.
//! Real-OpenSTA integration tests land in a follow-up commit.

use std::path::Path;
use std::process::Command;

use tempfile::TempDir;
use timing_ir::root_as_timing_ir;

const FIXTURE: &str = "\
# format-version: 1
# generator-tool: opensta-to-ir 0.1.0
# generator-opensta: fake
# input-files: a.lib
CORNER\t0\tdefault\ttt\t1.0\t25.0
ARC\tu1\tA\tY\t0\t100.0\t110.0\t120.0\t100.0\t110.0\t120.0\t\tAsserted
# end
";

const EMPTY_FIXTURE: &str = "\
# format-version: 1
# input-files:
CORNER\t0\tdefault\ttt\t1.0\t25.0
# end
";

fn bin() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_opensta-to-ir"))
}

#[test]
fn cli_from_dump_writes_ir() {
    let dir = TempDir::new().unwrap();
    let dump_path = dir.path().join("in.osd");
    let out_path = dir.path().join("out.jtir");
    std::fs::write(&dump_path, FIXTURE).unwrap();

    let status = Command::new(bin())
        .arg("--from-dump")
        .arg(&dump_path)
        .arg("--output")
        .arg(&out_path)
        .output()
        .expect("run binary");
    assert_eq!(
        status.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&status.stderr)
    );

    let buf = std::fs::read(&out_path).expect("output file written");
    let ir = root_as_timing_ir(&buf).expect("readable IR");
    assert_eq!(ir.timing_arcs().unwrap().len(), 1);
}

#[test]
fn cli_min_arcs_failure_exit_3() {
    let dir = TempDir::new().unwrap();
    let dump_path = dir.path().join("empty.osd");
    let out_path = dir.path().join("out.jtir");
    std::fs::write(&dump_path, EMPTY_FIXTURE).unwrap();

    let status = Command::new(bin())
        .arg("--from-dump")
        .arg(&dump_path)
        .arg("--output")
        .arg(&out_path)
        .output()
        .expect("run binary");
    assert_eq!(status.status.code(), Some(3));

    // --allow-empty-parse bypasses the check.
    let status_ok = Command::new(bin())
        .arg("--from-dump")
        .arg(&dump_path)
        .arg("--output")
        .arg(&out_path)
        .arg("--allow-empty-parse")
        .output()
        .expect("run binary");
    assert_eq!(status_ok.status.code(), Some(0));
}

#[test]
fn cli_without_from_dump_errors() {
    let dir = TempDir::new().unwrap();
    let out_path = dir.path().join("out.jtir");
    let status = Command::new(bin())
        .arg("--output")
        .arg(&out_path)
        .output()
        .expect("run binary");
    assert_eq!(status.status.code(), Some(1));
    let stderr = String::from_utf8_lossy(&status.stderr);
    assert!(stderr.contains("not yet implemented"), "stderr: {stderr}");
}
