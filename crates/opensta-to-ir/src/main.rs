//! `opensta-to-ir` CLI binary.
//!
//! Phase 0 WS2 (Phase 2.1): the OpenSTA subprocess invocation is not yet
//! wired up. The `--from-dump` escape hatch reads a pre-generated dump
//! file (the format the Tcl driver will produce) and exercises the
//! dump → IR pipeline end-to-end. The escape hatch is genuinely useful
//! beyond Phase 2.1 — fixture-driven workflows can regenerate golden IR
//! from a saved dump without re-running OpenSTA.

use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;

const EXIT_OK: u8 = 0;
const EXIT_OPENSTA_FAILED: u8 = 1;
const EXIT_BUILD_FAILED: u8 = 2;
const EXIT_MIN_ARCS_FAILED: u8 = 3;
const EXIT_ARG_INVALID: u8 = 4;

#[derive(Parser, Debug)]
#[command(
    name = "opensta-to-ir",
    about = "Drive OpenSTA and emit Jacquard timing IR",
    long_about = "Phase 0 WS2 preprocessing tool. See \
                  docs/plans/ws2-opensta-to-ir.md."
)]
struct Args {
    /// Liberty cell library files. Repeatable.
    #[arg(long = "liberty", value_name = "PATH")]
    liberty: Vec<PathBuf>,

    /// Verilog netlists. Repeatable.
    #[arg(long = "verilog", value_name = "PATH")]
    verilog: Vec<PathBuf>,

    /// SDF back-annotated delays.
    #[arg(long = "sdf", value_name = "PATH")]
    sdf: Option<PathBuf>,

    /// SPEF parasitics.
    #[arg(long = "spef", value_name = "PATH")]
    spef: Option<PathBuf>,

    /// SDC constraint file.
    #[arg(long = "sdc", value_name = "PATH")]
    sdc: Option<PathBuf>,

    /// Top-level module name.
    #[arg(long = "top", value_name = "NAME")]
    top: Option<String>,

    /// Corner names (repeatable). Defaults to a single "default" corner.
    #[arg(long = "corner", value_name = "NAME")]
    corner: Vec<String>,

    /// IR output path (.jtir).
    #[arg(long = "output", value_name = "PATH")]
    output: PathBuf,

    /// Override the OpenSTA executable path. Default: probe via
    /// `scripts/build-opensta.sh --print-binary`, then PATH lookup.
    #[arg(long = "opensta-bin", value_name = "PATH")]
    opensta_bin: Option<PathBuf>,

    /// Read a pre-generated dump file instead of invoking OpenSTA.
    /// Phase 2.1 escape hatch; remains useful for regenerating IR from a
    /// saved dump without OpenSTA installed.
    #[arg(long = "from-dump", value_name = "PATH")]
    from_dump: Option<PathBuf>,

    /// Fail when fewer than this many timing arcs are emitted.
    #[arg(long = "min-arcs", default_value_t = 1, value_name = "N")]
    min_arcs: usize,

    /// Disable the --min-arcs check. For test fixtures only.
    #[arg(long = "allow-empty-parse", default_value_t = false)]
    allow_empty_parse: bool,
}

fn main() -> ExitCode {
    let args = Args::parse();
    match run(&args) {
        Ok(code) => ExitCode::from(code),
        Err((code, msg)) => {
            eprintln!("opensta-to-ir: {msg}");
            ExitCode::from(code)
        }
    }
}

fn run(args: &Args) -> Result<u8, (u8, String)> {
    let dump_text = match &args.from_dump {
        Some(path) => std::fs::read_to_string(path)
            .map_err(|e| (EXIT_ARG_INVALID, format!("reading {}: {e}", path.display())))?,
        None => {
            return Err((
                EXIT_OPENSTA_FAILED,
                "OpenSTA subprocess invocation is not yet implemented in Phase 2.1; \
                 pass --from-dump <PATH> to test the dump→IR pipeline."
                    .to_string(),
            ));
        }
    };

    let doc = opensta_to_ir::dump::parse_dump(&dump_text)
        .map_err(|e| (EXIT_BUILD_FAILED, format!("parsing dump: {e}")))?;

    let (buf, stats) = opensta_to_ir::builder::build_ir(&doc, env!("CARGO_PKG_VERSION"));

    if !args.allow_empty_parse && stats.arcs < args.min_arcs {
        return Err((
            EXIT_MIN_ARCS_FAILED,
            format!(
                "produced {} timing arcs (--min-arcs {}); pass --allow-empty-parse \
                 for empty-fixture tests",
                stats.arcs, args.min_arcs
            ),
        ));
    }

    std::fs::write(&args.output, &buf).map_err(|e| {
        (
            EXIT_BUILD_FAILED,
            format!("writing {}: {e}", args.output.display()),
        )
    })?;

    eprintln!(
        "opensta-to-ir: wrote {} ({} corners, {} arcs)",
        args.output.display(),
        stats.corners,
        stats.arcs
    );
    Ok(EXIT_OK)
}
