//! OpenSTA discovery, version checking, and subprocess invocation.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::{Mutex, OnceLock};

use tempfile::TempDir;

use crate::dump::{parse_dump, DumpDocument, ParseError};

/// Minimum OpenSTA version Jacquard has been tested with. Matches the
/// commit pinned at `vendor/opensta/` (per ADR 0005). Older versions are
/// rejected with a hard error in `locate_and_check`.
pub const MIN_TESTED_OPENSTA_VERSION: OpenstaVersion = OpenstaVersion {
    major: 3,
    minor: 1,
    patch: 0,
};

/// Maximum OpenSTA version Jacquard has been tested with. Newer
/// versions are accepted with a `VersionStatus::NewerThanTested`
/// warning so that users see any timing discrepancies.
pub const MAX_TESTED_OPENSTA_VERSION: OpenstaVersion = OpenstaVersion {
    major: 3,
    minor: 1,
    patch: 0,
};

/// OpenSTA version triple as reported by `<binary> -version`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OpenstaVersion {
    pub major: u32,
    pub minor: u32,
    pub patch: u32,
}

impl std::fmt::Display for OpenstaVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

/// Output of `locate_and_check` when both discovery and version probe
/// succeed and the version is at least `MIN_TESTED_OPENSTA_VERSION`.
/// Callers compare `version` against `MAX_TESTED_OPENSTA_VERSION` to
/// decide whether to surface a "newer than tested" warning.
#[derive(Debug, Clone)]
pub struct LocatedOpensta {
    pub binary: PathBuf,
    pub version: OpenstaVersion,
}

/// Errors from locating and version-checking OpenSTA. Per WS-RH.1 these
/// are surfaced as hard errors when the shipped runtime needs OpenSTA
/// (`--sdf` was passed); they are not fall-through warnings.
#[derive(Debug)]
pub enum LocateError {
    /// `find_opensta` returned None.
    NotFound,
    /// Spawning `<binary> -version` failed.
    VersionProbeFailed {
        binary: PathBuf,
        err: std::io::Error,
    },
    /// `<binary> -version` exited non-zero.
    VersionProbeNonZero {
        binary: PathBuf,
        exit_code: Option<i32>,
        stderr: String,
    },
    /// `<binary> -version` produced output we couldn't parse.
    VersionUnparseable {
        binary: PathBuf,
        raw: String,
    },
    /// Detected version is older than `MIN_TESTED_OPENSTA_VERSION`.
    TooOld {
        binary: PathBuf,
        found: OpenstaVersion,
        required: OpenstaVersion,
    },
}

impl std::fmt::Display for LocateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotFound => write!(
                f,
                "OpenSTA binary not found. Set JACQUARD_OPENSTA_BIN, ensure `sta` is on PATH, \
                 or run scripts/build-opensta.sh to build the vendored copy."
            ),
            Self::VersionProbeFailed { binary, err } => write!(
                f,
                "failed to spawn `{} -version`: {err}",
                binary.display()
            ),
            Self::VersionProbeNonZero {
                binary,
                exit_code,
                stderr,
            } => {
                let code = exit_code
                    .map(|c| c.to_string())
                    .unwrap_or_else(|| "<signal>".into());
                write!(
                    f,
                    "`{} -version` exited with status {code}\n{stderr}",
                    binary.display()
                )
            }
            Self::VersionUnparseable { binary, raw } => write!(
                f,
                "could not parse OpenSTA version from `{} -version` output: {raw:?}",
                binary.display()
            ),
            Self::TooOld {
                binary,
                found,
                required,
            } => write!(
                f,
                "OpenSTA at `{}` is v{found}; Jacquard requires v{required} or newer. \
                 Rebuild via scripts/build-opensta.sh or upgrade your system OpenSTA.",
                binary.display()
            ),
        }
    }
}

impl std::error::Error for LocateError {}

/// Parse OpenSTA's `-version` output. Permissive: accepts an optional
/// leading `v`, missing patch component, and any trailing pre-release /
/// build-metadata suffix (`-rc1`, `+gitsha`).
pub fn parse_version(raw: &str) -> Option<OpenstaVersion> {
    let s = raw.trim().lines().next()?.trim();
    let s = s.strip_prefix('v').unwrap_or(s);
    // Strip pre-release / build-metadata suffixes.
    let core = s
        .split(|c: char| c == '-' || c == '+')
        .next()
        .unwrap_or(s);
    let mut parts = core.split('.');
    let major: u32 = parts.next()?.parse().ok()?;
    let minor: u32 = parts.next()?.parse().ok()?;
    let patch: u32 = parts.next().map(|p| p.parse().ok()).unwrap_or(Some(0))?;
    Some(OpenstaVersion {
        major,
        minor,
        patch,
    })
}

/// Probe a binary for its OpenSTA version. Caches results per binary
/// path so a long-running process doesn't pay the subprocess cost twice.
pub fn probe_version(binary: &Path) -> Result<OpenstaVersion, LocateError> {
    static CACHE: OnceLock<Mutex<HashMap<PathBuf, OpenstaVersion>>> = OnceLock::new();
    let cache = CACHE.get_or_init(|| Mutex::new(HashMap::new()));
    if let Some(v) = cache.lock().unwrap().get(binary) {
        return Ok(*v);
    }
    let out = Command::new(binary)
        .arg("-version")
        .output()
        .map_err(|err| LocateError::VersionProbeFailed {
            binary: binary.to_path_buf(),
            err,
        })?;
    if !out.status.success() {
        return Err(LocateError::VersionProbeNonZero {
            binary: binary.to_path_buf(),
            exit_code: out.status.code(),
            stderr: String::from_utf8_lossy(&out.stderr).into_owned(),
        });
    }
    let stdout = String::from_utf8_lossy(&out.stdout);
    let v = parse_version(&stdout).ok_or_else(|| LocateError::VersionUnparseable {
        binary: binary.to_path_buf(),
        raw: stdout.into_owned(),
    })?;
    cache.lock().unwrap().insert(binary.to_path_buf(), v);
    Ok(v)
}

/// Locate OpenSTA and verify its version is at least
/// `MIN_TESTED_OPENSTA_VERSION`. Caller is responsible for checking
/// `version > MAX_TESTED_OPENSTA_VERSION` and warning the user if so.
pub fn locate_and_check(explicit: Option<&Path>) -> Result<LocatedOpensta, LocateError> {
    let binary = find_opensta(explicit).ok_or(LocateError::NotFound)?;
    let version = probe_version(&binary)?;
    if version < MIN_TESTED_OPENSTA_VERSION {
        return Err(LocateError::TooOld {
            binary,
            found: version,
            required: MIN_TESTED_OPENSTA_VERSION,
        });
    }
    Ok(LocatedOpensta { binary, version })
}

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_version_basic() {
        assert_eq!(
            parse_version("3.1.0"),
            Some(OpenstaVersion {
                major: 3,
                minor: 1,
                patch: 0
            })
        );
    }

    #[test]
    fn parse_version_with_trailing_newline() {
        assert_eq!(
            parse_version("3.1.0\n"),
            Some(OpenstaVersion {
                major: 3,
                minor: 1,
                patch: 0
            })
        );
    }

    #[test]
    fn parse_version_first_line_only() {
        // OpenSTA's `-version` may print extra noise on later lines in
        // some builds; we only care about the first.
        assert_eq!(
            parse_version("3.2.5\nbuilt with gcc 11.4\n"),
            Some(OpenstaVersion {
                major: 3,
                minor: 2,
                patch: 5
            })
        );
    }

    #[test]
    fn parse_version_missing_patch_defaults_to_zero() {
        assert_eq!(
            parse_version("3.1"),
            Some(OpenstaVersion {
                major: 3,
                minor: 1,
                patch: 0
            })
        );
    }

    #[test]
    fn parse_version_strips_leading_v() {
        assert_eq!(
            parse_version("v3.1.0"),
            Some(OpenstaVersion {
                major: 3,
                minor: 1,
                patch: 0
            })
        );
    }

    #[test]
    fn parse_version_strips_prerelease_suffix() {
        assert_eq!(
            parse_version("3.1.0-rc1"),
            Some(OpenstaVersion {
                major: 3,
                minor: 1,
                patch: 0
            })
        );
        assert_eq!(
            parse_version("3.1.0+abcdef"),
            Some(OpenstaVersion {
                major: 3,
                minor: 1,
                patch: 0
            })
        );
    }

    #[test]
    fn parse_version_rejects_garbage() {
        assert_eq!(parse_version(""), None);
        assert_eq!(parse_version("not a version"), None);
        assert_eq!(parse_version("3"), None); // need at least major.minor
    }

    #[test]
    fn version_ordering() {
        let a = OpenstaVersion {
            major: 3,
            minor: 1,
            patch: 0,
        };
        let b = OpenstaVersion {
            major: 3,
            minor: 1,
            patch: 1,
        };
        let c = OpenstaVersion {
            major: 3,
            minor: 2,
            patch: 0,
        };
        let d = OpenstaVersion {
            major: 4,
            minor: 0,
            patch: 0,
        };
        assert!(a < b);
        assert!(b < c);
        assert!(c < d);
    }

    #[test]
    fn min_tested_matches_vendored_pin() {
        // `vendor/opensta/build/sta -version` reports 3.1.0 at commit
        // f361dd65 (the current submodule pin). If the submodule is
        // bumped, update this constant alongside the version-check
        // assumptions in `locate_and_check`.
        assert_eq!(
            MIN_TESTED_OPENSTA_VERSION,
            OpenstaVersion {
                major: 3,
                minor: 1,
                patch: 0
            }
        );
    }
}
