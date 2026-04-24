//! `timing-ir-diff` — structured diff between two timing-IR documents.
//!
//! Used by CI to validate converter output against golden IR, and by
//! contributors to debug differences between two IRs produced by
//! different tools or tool versions. See WS4 in
//! `docs/plans/phase-0-ir-and-oracle.md`.

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, ValueEnum};
use timing_ir::diff::{diff_irs, DiffOptions};
use timing_ir::root_as_timing_ir;

/// Exit codes.
///
/// - `0` — diff clean.
/// - `1` — diffs detected past tolerance.
/// - `2` — tooling error (file not found, invalid IR, etc.).
const EXIT_CLEAN: u8 = 0;
const EXIT_DIFFS: u8 = 1;
const EXIT_ERROR: u8 = 2;

#[derive(Parser, Debug)]
#[command(
    name = "timing-ir-diff",
    about = "Diff two Jacquard timing-IR (.jtir) files",
    long_about = "Produces a structured diff between two timing-IR documents. \
                  Used by CI to validate converter output against golden IR."
)]
struct Args {
    /// Left-hand input. Typically the "expected" or golden IR in CI usage.
    a: PathBuf,
    /// Right-hand input. Typically the "actual" IR produced by a converter.
    b: PathBuf,

    /// Absolute delay tolerance in picoseconds. Either this OR `--tolerance-rel`
    /// being satisfied counts as a match.
    #[arg(long, default_value_t = 5.0)]
    tolerance_abs_ps: f64,

    /// Relative delay tolerance as a fraction (e.g. `0.02` for 2%). Either this
    /// OR `--tolerance-abs-ps` being satisfied counts as a match.
    #[arg(long, default_value_t = 0.02)]
    tolerance_rel: f64,

    /// Output format.
    #[arg(long, value_enum, default_value_t = Format::Text)]
    format: Format,
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum Format {
    /// Human-readable summary printed to stdout.
    Text,
    /// Structured JSON suitable for CI consumption.
    Json,
}

fn main() -> ExitCode {
    let args = Args::parse();
    match run(&args) {
        Ok(exit) => ExitCode::from(exit),
        Err(e) => {
            eprintln!("timing-ir-diff: {e}");
            ExitCode::from(EXIT_ERROR)
        }
    }
}

fn run(args: &Args) -> Result<u8, String> {
    let opts = DiffOptions {
        absolute_ps: args.tolerance_abs_ps,
        relative: args.tolerance_rel,
    };

    let a_bytes =
        std::fs::read(&args.a).map_err(|e| format!("reading {}: {e}", args.a.display()))?;
    let b_bytes =
        std::fs::read(&args.b).map_err(|e| format!("reading {}: {e}", args.b.display()))?;

    let a_ir = root_as_timing_ir(&a_bytes)
        .map_err(|e| format!("parsing {} as TimingIR: {e}", args.a.display()))?;
    let b_ir = root_as_timing_ir(&b_bytes)
        .map_err(|e| format!("parsing {} as TimingIR: {e}", args.b.display()))?;

    let report = diff_irs(&a_ir, &b_ir, &opts);

    match args.format {
        Format::Text => {
            print!("{report}");
        }
        Format::Json => {
            let json = serde_json::to_string_pretty(&report)
                .map_err(|e| format!("serialising report as JSON: {e}"))?;
            println!("{json}");
        }
    }

    Ok(if report.is_clean() {
        EXIT_CLEAN
    } else {
        EXIT_DIFFS
    })
}
