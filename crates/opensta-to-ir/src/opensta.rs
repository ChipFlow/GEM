//! OpenSTA discovery and subprocess invocation.
//!
//! Phase 0 WS2 (Phase 2.1-followup-A): plumbing only. The Tcl driver
//! currently emits a stub dump (header + default corner + `# end`); real
//! timing extraction is the next slice.

use std::path::{Path, PathBuf};
use std::process::Command;

use tempfile::TempDir;

use crate::dump::{parse_dump, DumpDocument, ParseError};

/// The Tcl driver that runs inside OpenSTA, embedded at compile time so
/// the binary is self-contained.
const TCL_DRIVER: &str = include_str!("../tcl/dump_timing.tcl");

/// Inputs to a single OpenSTA invocation.
pub struct Invocation<'a> {
    pub liberty: &'a [PathBuf],
    pub verilog: &'a [PathBuf],
    pub sdf: Option<&'a Path>,
    pub spef: Option<&'a Path>,
    pub sdc: Option<&'a Path>,
    pub top: &'a str,
    pub generator_version: &'a str,
}

/// Failure modes from the OpenSTA subprocess path.
#[derive(Debug)]
pub enum InvokeError {
    BinaryNotFound,
    Spawn(std::io::Error),
    NonZero {
        exit_code: Option<i32>,
        stderr: String,
    },
    DumpRead(std::io::Error),
    DumpParse(ParseError),
}

impl std::fmt::Display for InvokeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BinaryNotFound => write!(
                f,
                "OpenSTA binary not found. Run scripts/build-opensta.sh, set \
                 JACQUARD_OPENSTA_BIN, or pass --opensta-bin"
            ),
            Self::Spawn(e) => write!(f, "spawning OpenSTA: {e}"),
            Self::NonZero { exit_code, stderr } => {
                let code = exit_code
                    .map(|c| c.to_string())
                    .unwrap_or_else(|| "<signal>".into());
                write!(f, "OpenSTA exited with status {code}\n{stderr}")
            }
            Self::DumpRead(e) => write!(f, "reading dump file: {e}"),
            Self::DumpParse(e) => write!(f, "parsing dump file: {e}"),
        }
    }
}

impl std::error::Error for InvokeError {}

/// Locate the OpenSTA binary.
///
/// Resolution order:
///
/// 1. `--opensta-bin <PATH>` (when supplied).
/// 2. `JACQUARD_OPENSTA_BIN` environment variable.
/// 3. `<repo-root>/scripts/build-opensta.sh --print-binary`, where
///    `<repo-root>` is derived from the build-time `CARGO_MANIFEST_DIR`.
///    This is the canonical install path during development.
/// 4. `sta` on `PATH`.
///
/// Returns `None` if all four fail.
pub fn find_opensta(explicit: Option<&Path>) -> Option<PathBuf> {
    if let Some(p) = explicit {
        if p.exists() {
            return Some(p.to_path_buf());
        }
        return None;
    }
    if let Ok(p) = std::env::var("JACQUARD_OPENSTA_BIN") {
        let pb = PathBuf::from(p);
        if pb.exists() {
            return Some(pb);
        }
    }
    if let Some(p) = probe_via_build_script() {
        return Some(p);
    }
    if let Ok(out) = Command::new("which").arg("sta").output() {
        if out.status.success() {
            let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
            if !s.is_empty() {
                return Some(PathBuf::from(s));
            }
        }
    }
    None
}

fn probe_via_build_script() -> Option<PathBuf> {
    // CARGO_MANIFEST_DIR points at crates/opensta-to-ir/ at build time;
    // the script lives at <repo-root>/scripts/build-opensta.sh.
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let script = manifest_dir
        .parent()?
        .parent()?
        .join("scripts/build-opensta.sh");
    if !script.exists() {
        return None;
    }
    let out = Command::new(&script).arg("--print-binary").output().ok()?;
    if !out.status.success() {
        return None;
    }
    let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
    if s.is_empty() {
        None
    } else {
        Some(PathBuf::from(s))
    }
}

/// Run OpenSTA with the embedded Tcl driver and return the parsed dump.
///
/// Pass `keep_tmp = true` to keep the temp directory after the call (the
/// path is logged to stderr) for debugging dump-format issues.
pub fn run(
    binary: &Path,
    inv: &Invocation<'_>,
    keep_tmp: bool,
    verbose: bool,
) -> Result<DumpDocument, InvokeError> {
    let dir = TempDir::new().map_err(InvokeError::Spawn)?;
    let script_path = dir.path().join("dump_timing.tcl");
    let dump_path = dir.path().join("dump.osd");
    std::fs::write(&script_path, TCL_DRIVER).map_err(InvokeError::Spawn)?;

    let liberty_arg = paths_to_lines(inv.liberty);
    let verilog_arg = paths_to_lines(inv.verilog);

    let mut cmd = Command::new(binary);
    cmd.arg("-no_init")
        .arg("-no_splash")
        .arg("-exit")
        .arg(&script_path);
    cmd.env("JACQUARD_DUMP_PATH", &dump_path);
    cmd.env("JACQUARD_LIBERTY_FILES", liberty_arg);
    cmd.env("JACQUARD_VERILOG_FILES", verilog_arg);
    cmd.env("JACQUARD_TOP", inv.top);
    cmd.env("JACQUARD_GENERATOR_VERSION", inv.generator_version);
    if let Some(p) = inv.sdf {
        cmd.env("JACQUARD_SDF_FILE", p);
    }
    if let Some(p) = inv.spef {
        cmd.env("JACQUARD_SPEF_FILE", p);
    }
    if let Some(p) = inv.sdc {
        cmd.env("JACQUARD_SDC_FILE", p);
    }

    let output = cmd.output().map_err(InvokeError::Spawn)?;

    if verbose {
        if !output.stderr.is_empty() {
            eprint!("{}", String::from_utf8_lossy(&output.stderr));
        }
        if !output.stdout.is_empty() {
            eprint!("{}", String::from_utf8_lossy(&output.stdout));
        }
    }

    if !output.status.success() {
        return Err(InvokeError::NonZero {
            exit_code: output.status.code(),
            stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
        });
    }

    let dump_text = std::fs::read_to_string(&dump_path).map_err(InvokeError::DumpRead)?;
    let doc = parse_dump(&dump_text).map_err(InvokeError::DumpParse)?;

    if keep_tmp {
        let kept = dir.keep();
        eprintln!("opensta-to-ir: kept tmp dir at {}", kept.display());
    }

    Ok(doc)
}

fn paths_to_lines(paths: &[PathBuf]) -> String {
    paths
        .iter()
        .map(|p| p.display().to_string())
        .collect::<Vec<_>>()
        .join("\n")
}
