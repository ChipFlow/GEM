//! `opensta-to-ir` CLI binary.
//!
//! Phase 0 WS2 (Phase 2.1-followup-A): subprocess plumbing is wired but
//! the embedded Tcl driver currently emits a stub dump (header + default
//! corner + `# end`). Real timing-arc extraction is the next slice. The
//! `--from-dump` flag remains as a fixture-driven workflow that
//! regenerates IR from a saved dump without re-running OpenSTA.

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

    /// Keep the OpenSTA temp directory after the run for debugging.
    #[arg(long = "keep-tmp", default_value_t = false)]
    keep_tmp: bool,

    /// Echo OpenSTA's stdout/stderr to ours.
    #[arg(short = 'v', long = "verbose", default_value_t = false)]
    verbose: bool,
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
    let doc = match &args.from_dump {
        Some(path) => {
            let text = std::fs::read_to_string(path)
                .map_err(|e| (EXIT_ARG_INVALID, format!("reading {}: {e}", path.display())))?;
            opensta_to_ir::dump::parse_dump(&text)
                .map_err(|e| (EXIT_BUILD_FAILED, format!("parsing dump: {e}")))?
        }
        None => run_opensta(args)?,
    };

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

fn run_opensta(args: &Args) -> Result<opensta_to_ir::dump::DumpDocument, (u8, String)> {
    if args.liberty.is_empty() {
        return Err((
            EXIT_ARG_INVALID,
            "at least one --liberty is required".into(),
        ));
    }
    if args.verilog.is_empty() {
        return Err((
            EXIT_ARG_INVALID,
            "at least one --verilog is required".into(),
        ));
    }
    let top = args
        .top
        .as_deref()
        .ok_or_else(|| (EXIT_ARG_INVALID, "--top <NAME> is required".to_string()))?;

    let binary =
        opensta_to_ir::opensta::find_opensta(args.opensta_bin.as_deref()).ok_or_else(|| {
            (
                EXIT_OPENSTA_FAILED,
                opensta_to_ir::opensta::InvokeError::BinaryNotFound.to_string(),
            )
        })?;

    let invocation = opensta_to_ir::opensta::Invocation {
        liberty: &args.liberty,
        verilog: &args.verilog,
        sdf: args.sdf.as_deref(),
        spef: args.spef.as_deref(),
        sdc: args.sdc.as_deref(),
        top,
        generator_version: env!("CARGO_PKG_VERSION"),
    };
    opensta_to_ir::opensta::run(&binary, &invocation, args.keep_tmp, args.verbose)
        .map_err(|e| (EXIT_OPENSTA_FAILED, e.to_string()))
}
