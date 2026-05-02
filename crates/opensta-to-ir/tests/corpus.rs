//! WS4 corpus regression test.
//!
//! For each entry under `tests/timing_ir/corpus/`, re-runs `opensta-to-ir`
//! against the entry's `inputs/` and diffs the freshly produced IR
//! against the checked-in `expected.jtir` via `timing_ir::diff::diff_irs`.
//! Fails if any entry's diff exceeds the per-design tolerance from
//! `manifest.toml`.
//!
//! When the OpenSTA submodule is bumped (or the dump-format / Tcl driver
//! changes), regenerate goldens via `scripts/regenerate-corpus-goldens.sh`
//! and review the diff before committing — that's the load-bearing
//! workflow this test exists to gate.
//!
//! See `docs/plans/phase-0-ir-and-oracle.md` § WS4 and
//! `tests/timing_ir/corpus/README.md`.

use std::path::{Path, PathBuf};
use std::process::Command;

use opensta_to_ir::opensta::find_opensta;
use serde::Deserialize;
use tempfile::TempDir;
use timing_ir::diff::{diff_irs, DiffOptions};
use timing_ir::root_as_timing_ir;

#[derive(Debug, Deserialize)]
struct Manifest {
    #[allow(dead_code)]
    description: String,
    opensta_to_ir: OpenStaToIr,
    tolerance: Tolerance,
}

#[derive(Debug, Deserialize)]
struct OpenStaToIr {
    liberty: Vec<PathBuf>,
    verilog: Vec<PathBuf>,
    sdf: Option<PathBuf>,
    sdc: Option<PathBuf>,
    spef: Option<PathBuf>,
    top: String,
}

#[derive(Debug, Deserialize)]
struct Tolerance {
    absolute_ps: f64,
    relative: f64,
}

fn bin() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_opensta-to-ir"))
}

/// Walk up from this test's CARGO_MANIFEST_DIR to find the repo root —
/// the directory containing `tests/timing_ir/corpus/`.
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("repo root above crates/opensta-to-ir")
        .to_path_buf()
}

fn corpus_dir() -> PathBuf {
    repo_root().join("tests/timing_ir/corpus")
}

#[test]
fn corpus_designs_match_golden_ir() {
    let Some(_sta) = find_opensta(None) else {
        eprintln!("skipping: OpenSTA not built; run scripts/build-opensta.sh");
        return;
    };

    let entries = collect_corpus_entries();
    assert!(
        !entries.is_empty(),
        "no corpus entries found under {} — at least one is required",
        corpus_dir().display()
    );

    let mut failures: Vec<String> = Vec::new();
    for entry in &entries {
        if let Err(reason) = check_entry(entry) {
            failures.push(format!("{}: {}", entry.display(), reason));
        }
    }

    if !failures.is_empty() {
        panic!(
            "{} corpus entry/entries diverged from golden IR:\n  {}",
            failures.len(),
            failures.join("\n  ")
        );
    }
}

fn collect_corpus_entries() -> Vec<PathBuf> {
    let mut out = Vec::new();
    let dir = corpus_dir();
    let Ok(read) = std::fs::read_dir(&dir) else {
        return out;
    };
    for entry in read.flatten() {
        let path = entry.path();
        if path.is_dir() && path.join("manifest.toml").is_file() {
            out.push(path);
        }
    }
    out.sort();
    out
}

fn check_entry(entry_dir: &Path) -> Result<(), String> {
    let manifest_path = entry_dir.join("manifest.toml");
    let manifest_text =
        std::fs::read_to_string(&manifest_path).map_err(|e| format!("read manifest: {e}"))?;
    let manifest: Manifest =
        toml::from_str(&manifest_text).map_err(|e| format!("parse manifest: {e}"))?;

    let resolve = |p: &Path| -> PathBuf {
        if p.is_absolute() {
            p.to_path_buf()
        } else {
            entry_dir.join(p)
        }
    };

    let liberty: Vec<PathBuf> = manifest
        .opensta_to_ir
        .liberty
        .iter()
        .map(|p| resolve(p))
        .collect();
    let verilog: Vec<PathBuf> = manifest
        .opensta_to_ir
        .verilog
        .iter()
        .map(|p| resolve(p))
        .collect();
    let sdf = manifest.opensta_to_ir.sdf.as_deref().map(resolve);
    let sdc = manifest.opensta_to_ir.sdc.as_deref().map(resolve);
    let spef = manifest.opensta_to_ir.spef.as_deref().map(resolve);

    let tmp = TempDir::new().map_err(|e| format!("temp dir: {e}"))?;
    let fresh_path = tmp.path().join("fresh.jtir");

    let mut cmd = Command::new(bin());
    for lib in &liberty {
        cmd.arg("--liberty").arg(lib);
    }
    for v in &verilog {
        cmd.arg("--verilog").arg(v);
    }
    if let Some(p) = &sdf {
        cmd.arg("--sdf").arg(p);
    }
    if let Some(p) = &sdc {
        cmd.arg("--sdc").arg(p);
    }
    if let Some(p) = &spef {
        cmd.arg("--spef").arg(p);
    }
    cmd.arg("--top").arg(&manifest.opensta_to_ir.top);
    cmd.arg("--output").arg(&fresh_path);

    let output = cmd
        .output()
        .map_err(|e| format!("spawn opensta-to-ir: {e}"))?;
    if !output.status.success() {
        return Err(format!(
            "opensta-to-ir exit {:?}\nstderr: {}",
            output.status.code(),
            String::from_utf8_lossy(&output.stderr)
        ));
    }

    let fresh_buf = std::fs::read(&fresh_path).map_err(|e| format!("read fresh IR: {e}"))?;
    let golden_buf = std::fs::read(entry_dir.join("expected.jtir")).map_err(|e| {
        format!("read expected.jtir: {e} — regenerate via scripts/regenerate-corpus-goldens.sh")
    })?;

    let fresh_ir = root_as_timing_ir(&fresh_buf).map_err(|e| format!("parse fresh IR: {e}"))?;
    let golden_ir = root_as_timing_ir(&golden_buf).map_err(|e| format!("parse golden IR: {e}"))?;

    let opts = DiffOptions {
        absolute_ps: manifest.tolerance.absolute_ps,
        relative: manifest.tolerance.relative,
    };
    let report = diff_irs(&golden_ir, &fresh_ir, &opts);
    if report.is_clean() {
        Ok(())
    } else {
        Err(format!(
            "diff vs golden:\n{}\n(regenerate via scripts/regenerate-corpus-goldens.sh after \
             reviewing the diff)",
            report
        ))
    }
}
