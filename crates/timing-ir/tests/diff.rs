//! Unit + integration tests for the diff module and `timing-ir-diff` binary.
//!
//! Builds synthetic IRs in-memory and exercises each diff category.

use std::path::Path;

use flatbuffers::FlatBufferBuilder;
use timing_ir::diff::{diff_irs, DiffOptions};
use timing_ir::*;

// -----------------------------------------------------------------------------
// Helpers — build minimal IRs with specific arcs
// -----------------------------------------------------------------------------

/// Build an IR with one corner and a list of (driver_pin, load_pin, rise_max_ps, fall_max_ps).
///
/// Returns an owned `Vec<u8>` suitable for `root_as_timing_ir`.
fn build_ir_with_arcs(arcs: &[(&str, &str, f64, f64)]) -> Vec<u8> {
    let mut b = FlatBufferBuilder::with_capacity(1024);

    let corner_name = b.create_string("tt_25C_1v80");
    let corner_process = b.create_string("tt");
    let corner = Corner::create(
        &mut b,
        &CornerArgs {
            name: Some(corner_name),
            process: Some(corner_process),
            voltage: 1.80,
            temperature: 25.0,
        },
    );
    let corners_vec = b.create_vector(&[corner]);

    let prov_tool = b.create_string("test");
    let prov_file = b.create_string("synthetic");
    let provenance = Provenance::create(
        &mut b,
        &ProvenanceArgs {
            source_tool: Some(prov_tool),
            source_file: Some(prov_file),
            origin: Origin::Asserted,
        },
    );

    let mut arc_offsets = Vec::new();
    for (driver, load, rise_max, fall_max) in arcs {
        let cell = b.create_string("u1");
        let drv = b.create_string(driver);
        let ld = b.create_string(load);
        let cond = b.create_string("");
        let rise = [TimingValue::new(0, rise_max * 0.8, *rise_max, *rise_max)];
        let fall = [TimingValue::new(0, fall_max * 0.8, *fall_max, *fall_max)];
        let rise_vec = b.create_vector(&rise);
        let fall_vec = b.create_vector(&fall);
        let arc = TimingArc::create(
            &mut b,
            &TimingArcArgs {
                cell_instance: Some(cell),
                driver_pin: Some(drv),
                load_pin: Some(ld),
                rise_delay: Some(rise_vec),
                fall_delay: Some(fall_vec),
                condition: Some(cond),
                provenance: Some(provenance),
            },
        );
        arc_offsets.push(arc);
    }
    let arcs_vec = b.create_vector(&arc_offsets);

    let gen_tool = b.create_string("test");
    let gen_version = b.create_string("0.1.0");
    let schema_version = SchemaVersion::new(SCHEMA_MAJOR, SCHEMA_MINOR, SCHEMA_PATCH);

    let ir = TimingIR::create(
        &mut b,
        &TimingIRArgs {
            schema_version: Some(&schema_version),
            corners: Some(corners_vec),
            timing_arcs: Some(arcs_vec),
            interconnect_delays: None,
            setup_hold_checks: None,
            vendor_extensions: None,
            generator_tool: Some(gen_tool),
            generator_version: Some(gen_version),
            input_files: None,
        },
    );
    finish_timing_ir_buffer(&mut b, ir);
    b.finished_data().to_vec()
}

// -----------------------------------------------------------------------------
// Unit tests — diff_irs behaviour
// -----------------------------------------------------------------------------

#[test]
fn identical_irs_are_clean() {
    let buf = build_ir_with_arcs(&[("A", "Y", 100.0, 110.0)]);
    let a = root_as_timing_ir(&buf).unwrap();
    let b = root_as_timing_ir(&buf).unwrap();
    let report = diff_irs(&a, &b, &DiffOptions::default());
    assert!(
        report.is_clean(),
        "identical IRs should diff clean: {report}"
    );
}

#[test]
fn delay_mismatch_past_tolerance_detected() {
    let a_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0)]);
    // Bump rise_max by 20 ps — well past default 5 ps / 2% tolerance.
    let b_buf = build_ir_with_arcs(&[("A", "Y", 120.0, 100.0)]);
    let a = root_as_timing_ir(&a_buf).unwrap();
    let b = root_as_timing_ir(&b_buf).unwrap();
    let report = diff_irs(&a, &b, &DiffOptions::default());
    assert!(!report.is_clean(), "20 ps diff should not be clean");
    assert!(
        !report.timing_arcs.value_mismatches.is_empty(),
        "expected a value mismatch, got: {report}"
    );
}

#[test]
fn delay_mismatch_within_tolerance_is_clean() {
    let a_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0)]);
    // 3 ps change is inside the default 5 ps absolute tolerance.
    let b_buf = build_ir_with_arcs(&[("A", "Y", 103.0, 100.0)]);
    let a = root_as_timing_ir(&a_buf).unwrap();
    let b = root_as_timing_ir(&b_buf).unwrap();
    let report = diff_irs(&a, &b, &DiffOptions::default());
    assert!(report.is_clean(), "3 ps diff within tolerance: {report}");
}

#[test]
fn arc_only_in_a_detected() {
    let a_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0), ("B", "Y", 100.0, 100.0)]);
    let b_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0)]);
    let a = root_as_timing_ir(&a_buf).unwrap();
    let b = root_as_timing_ir(&b_buf).unwrap();
    let report = diff_irs(&a, &b, &DiffOptions::default());
    assert!(!report.is_clean());
    assert_eq!(report.timing_arcs.only_in_a.len(), 1);
    assert_eq!(report.timing_arcs.only_in_b.len(), 0);
    assert_eq!(report.timing_arcs.only_in_a[0].driver_pin, "B");
}

#[test]
fn arc_only_in_b_detected() {
    let a_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0)]);
    let b_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0), ("B", "Y", 100.0, 100.0)]);
    let a = root_as_timing_ir(&a_buf).unwrap();
    let b = root_as_timing_ir(&b_buf).unwrap();
    let report = diff_irs(&a, &b, &DiffOptions::default());
    assert!(!report.is_clean());
    assert_eq!(report.timing_arcs.only_in_a.len(), 0);
    assert_eq!(report.timing_arcs.only_in_b.len(), 1);
    assert_eq!(report.timing_arcs.only_in_b[0].driver_pin, "B");
}

#[test]
fn tolerance_options_respected() {
    let a_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0)]);
    let b_buf = build_ir_with_arcs(&[("A", "Y", 120.0, 100.0)]);
    let a = root_as_timing_ir(&a_buf).unwrap();
    let b = root_as_timing_ir(&b_buf).unwrap();

    // Tight tolerance: should flag.
    let tight = DiffOptions {
        absolute_ps: 1.0,
        relative: 0.001,
    };
    assert!(!diff_irs(&a, &b, &tight).is_clean());

    // Loose tolerance: should pass.
    let loose = DiffOptions {
        absolute_ps: 100.0,
        relative: 0.5,
    };
    assert!(diff_irs(&a, &b, &loose).is_clean());
}

#[test]
fn text_rendering_contains_status() {
    let a_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0)]);
    let b_buf = build_ir_with_arcs(&[("A", "Y", 200.0, 100.0)]);
    let a = root_as_timing_ir(&a_buf).unwrap();
    let b = root_as_timing_ir(&b_buf).unwrap();
    let report = diff_irs(&a, &b, &DiffOptions::default());
    let text = format!("{report}");
    assert!(text.contains("DIFFS"), "text: {text}");
    assert!(text.contains("timing arcs:"), "text: {text}");

    let clean_report = diff_irs(&a, &a, &DiffOptions::default());
    let clean_text = format!("{clean_report}");
    assert!(clean_text.contains("CLEAN"), "clean text: {clean_text}");
}

#[test]
fn json_serialisation_roundtrips_through_serde() {
    let a_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0)]);
    let b_buf = build_ir_with_arcs(&[("A", "Y", 120.0, 100.0)]);
    let a = root_as_timing_ir(&a_buf).unwrap();
    let b = root_as_timing_ir(&b_buf).unwrap();
    let report = diff_irs(&a, &b, &DiffOptions::default());
    let json = serde_json::to_string_pretty(&report).expect("serialises");
    assert!(json.contains("timing_arcs"));
    assert!(json.contains("value_mismatches"));
}

// -----------------------------------------------------------------------------
// Binary integration — run `timing-ir-diff` end-to-end
// -----------------------------------------------------------------------------

/// Writes a buffer to a temp file under the cargo target dir and returns the path.
fn write_temp_ir(name: &str, buf: &[u8]) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join("jacquard-timing-ir-diff-tests");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, buf).unwrap();
    path
}

fn bin_path() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_timing-ir-diff"))
}

#[test]
fn cli_exit_0_on_clean_diff() {
    let buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0)]);
    let p = write_temp_ir("clean_a.jtir", &buf);
    let q = write_temp_ir("clean_b.jtir", &buf);
    let out = std::process::Command::new(bin_path())
        .arg(&p)
        .arg(&q)
        .output()
        .expect("run binary");
    assert_eq!(
        out.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains("CLEAN"), "stdout: {stdout}");
}

#[test]
fn cli_exit_1_on_diff() {
    let a_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0)]);
    let b_buf = build_ir_with_arcs(&[("A", "Y", 200.0, 100.0)]);
    let p = write_temp_ir("diff_a.jtir", &a_buf);
    let q = write_temp_ir("diff_b.jtir", &b_buf);
    let out = std::process::Command::new(bin_path())
        .arg(&p)
        .arg(&q)
        .output()
        .expect("run binary");
    assert_eq!(
        out.status.code(),
        Some(1),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains("DIFFS"), "stdout: {stdout}");
}

#[test]
fn cli_exit_2_on_error() {
    let out = std::process::Command::new(bin_path())
        .arg("/nonexistent/path/a.jtir")
        .arg("/nonexistent/path/b.jtir")
        .output()
        .expect("run binary");
    assert_eq!(
        out.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
}

#[test]
fn cli_json_format_parses() {
    let a_buf = build_ir_with_arcs(&[("A", "Y", 100.0, 100.0)]);
    let b_buf = build_ir_with_arcs(&[("A", "Y", 200.0, 100.0)]);
    let p = write_temp_ir("json_a.jtir", &a_buf);
    let q = write_temp_ir("json_b.jtir", &b_buf);
    let out = std::process::Command::new(bin_path())
        .arg(&p)
        .arg(&q)
        .arg("--format")
        .arg("json")
        .output()
        .expect("run binary");
    assert_eq!(out.status.code(), Some(1));
    let stdout = std::str::from_utf8(&out.stdout).expect("utf8");
    let parsed: serde_json::Value = serde_json::from_str(stdout).expect("valid JSON");
    // Basic shape check.
    assert!(parsed.get("timing_arcs").is_some());
    assert!(parsed.get("options").is_some());
}
