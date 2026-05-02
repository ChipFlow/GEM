#!/usr/bin/env bash
# Build the vendored OpenSTA submodule.
#
# Idempotent: if the binary already exists and `--force` is not set, this
# script prints its path and exits 0. Otherwise it initialises the
# submodule (if needed), checks build dependencies, and invokes CMake.
#
# Tests and developers should call this once after cloning to prepare the
# environment. CI runners that need OpenSTA also invoke it.
#
# Usage:
#   scripts/build-opensta.sh [--force] [--jobs N] [--build-dir PATH]
#   scripts/build-opensta.sh --print-binary    # print path of the existing binary, exit 1 if missing
#
# Exits non-zero on any failure with a clear diagnostic.

set -euo pipefail

# ---------------------------------------------------------------------------
# Locate repo root (script lives at <repo>/scripts/build-opensta.sh)
# ---------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SUBMODULE_PATH="${REPO_ROOT}/vendor/opensta"
DEFAULT_BUILD_DIR="${SUBMODULE_PATH}/build"

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

FORCE=0
JOBS=""
BUILD_DIR="${DEFAULT_BUILD_DIR}"
PRINT_ONLY=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --force)
            FORCE=1
            shift
            ;;
        --jobs)
            JOBS="$2"
            shift 2
            ;;
        --jobs=*)
            JOBS="${1#--jobs=}"
            shift
            ;;
        --build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        --build-dir=*)
            BUILD_DIR="${1#--build-dir=}"
            shift
            ;;
        --print-binary)
            PRINT_ONLY=1
            shift
            ;;
        -h|--help)
            sed -n '2,16p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
            exit 0
            ;;
        *)
            echo "build-opensta.sh: unknown argument: $1" >&2
            echo "Try --help for usage." >&2
            exit 64
            ;;
    esac
done

BINARY_PATH="${BUILD_DIR}/sta"

log() { printf '[build-opensta] %s\n' "$*"; }
err() { printf '[build-opensta] error: %s\n' "$*" >&2; }

# ---------------------------------------------------------------------------
# --print-binary fast path
# ---------------------------------------------------------------------------

if [[ "${PRINT_ONLY}" -eq 1 ]]; then
    if [[ -x "${BINARY_PATH}" ]]; then
        printf '%s\n' "${BINARY_PATH}"
        exit 0
    else
        err "OpenSTA binary not found at ${BINARY_PATH}; run scripts/build-opensta.sh first"
        exit 1
    fi
fi

# ---------------------------------------------------------------------------
# Submodule check
# ---------------------------------------------------------------------------

if [[ ! -f "${SUBMODULE_PATH}/CMakeLists.txt" ]]; then
    log "OpenSTA submodule not initialised; running git submodule update --init"
    git -C "${REPO_ROOT}" submodule update --init "${SUBMODULE_PATH}"
fi

if [[ ! -f "${SUBMODULE_PATH}/CMakeLists.txt" ]]; then
    err "submodule init failed: ${SUBMODULE_PATH}/CMakeLists.txt still missing"
    exit 2
fi

# ---------------------------------------------------------------------------
# Existing-binary fast path (idempotency)
# ---------------------------------------------------------------------------

if [[ "${FORCE}" -eq 0 && -x "${BINARY_PATH}" ]]; then
    log "OpenSTA binary already present at ${BINARY_PATH}"
    log "(pass --force to rebuild)"
    printf '%s\n' "${BINARY_PATH}"
    exit 0
fi

# ---------------------------------------------------------------------------
# Build prerequisites
# ---------------------------------------------------------------------------

# Required tools — emit a single combined error listing everything missing
# rather than failing one at a time.
MISSING=()
for tool in cmake make bison flex swig; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        MISSING+=("${tool}")
    fi
done

if [[ ${#MISSING[@]} -gt 0 ]]; then
    err "missing required build tools: ${MISSING[*]}"
    case "$(uname -s)" in
        Darwin)
            err "install via Homebrew: brew bundle --file ${SUBMODULE_PATH}/Brewfile"
            ;;
        Linux)
            err "install via apt: sudo apt install cmake make bison flex libfl-dev swig libtcl-dev libeigen3-dev libfmt-dev"
            err "(also need cudd; see vendor/opensta/Dockerfile.ubuntu22.04 for build steps)"
            err "note: libfl-dev is what carries FlexLexer.h on Ubuntu — without it cmake fails with FLEX_INCLUDE_DIR=NOTFOUND"
            ;;
        *)
            err "install build dependencies appropriate for $(uname -s)"
            ;;
    esac
    exit 3
fi

# ---------------------------------------------------------------------------
# Configure + build
# ---------------------------------------------------------------------------

# CMake failures usually mean a library (CUDD, Tcl, Eigen) is missing.
# The executables we checked above are necessary but not sufficient — the
# Brewfile in vendor/opensta/ enumerates the full set.
suggest_install() {
    case "$(uname -s)" in
        Darwin)
            err "this typically means a build library is missing. Install all"
            err "dependencies with:"
            err "    brew bundle --file ${SUBMODULE_PATH}/Brewfile"
            err "(includes cudd from the mht208/formal tap)"
            ;;
        Linux)
            err "this typically means a build library is missing. See"
            err "${SUBMODULE_PATH}/Dockerfile.ubuntu22.04 for the canonical"
            err "Linux dependency list (apt + cudd built from source)."
            ;;
        *)
            err "this typically means a build library is missing for $(uname -s)."
            ;;
    esac
}

log "configuring OpenSTA in ${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# On macOS, Apple ships an old flex without FlexLexer.h and an old bison.
# Homebrew's keg-only formulas live under their own prefixes; CMake's
# default search paths miss them. Hand-feed CMake the homebrew paths.
EXTRA_CMAKE_ARGS=()
if [[ "$(uname -s)" == "Darwin" ]] && command -v brew >/dev/null 2>&1; then
    if FLEX_PREFIX="$(brew --prefix flex 2>/dev/null)"; then
        EXTRA_CMAKE_ARGS+=("-DFLEX_INCLUDE_DIR=${FLEX_PREFIX}/include")
        EXTRA_CMAKE_ARGS+=("-DFLEX_EXECUTABLE=${FLEX_PREFIX}/bin/flex")
    fi
    if BISON_PREFIX="$(brew --prefix bison 2>/dev/null)"; then
        EXTRA_CMAKE_ARGS+=("-DBISON_EXECUTABLE=${BISON_PREFIX}/bin/bison")
    fi
fi

# CUDD is not packaged in apt and OpenSTA's FindCUDD only searches
# `${CUDD_DIR}` plus default system paths. CI builds CUDD from source
# into a non-default prefix and exports CUDD_DIR; honour it here so the
# script doesn't need a separate CI-only entry point. Brew's mht208/formal
# tap installs cudd in the default search path on macOS, so the env var
# is unset there and the find_library default behaviour suffices.
if [[ -n "${CUDD_DIR:-}" ]]; then
    EXTRA_CMAKE_ARGS+=("-DCUDD_DIR=${CUDD_DIR}")
fi

if ! cmake -S "${SUBMODULE_PATH}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release \
    "${EXTRA_CMAKE_ARGS[@]}"; then
    err "cmake configure failed"
    suggest_install
    exit 5
fi

if [[ -z "${JOBS}" ]]; then
    case "$(uname -s)" in
        Darwin) JOBS="$(sysctl -n hw.ncpu 2>/dev/null || echo 4)" ;;
        Linux)  JOBS="$(nproc 2>/dev/null || echo 4)" ;;
        *)      JOBS=4 ;;
    esac
fi

log "building OpenSTA (parallel jobs: ${JOBS})"
if ! cmake --build "${BUILD_DIR}" --parallel "${JOBS}"; then
    err "cmake build failed"
    suggest_install
    exit 6
fi

if [[ ! -x "${BINARY_PATH}" ]]; then
    err "build completed but binary not found at ${BINARY_PATH}"
    err "expected layout may have changed; check vendor/opensta/CMakeLists.txt"
    exit 4
fi

log "build complete: ${BINARY_PATH}"
printf '%s\n' "${BINARY_PATH}"
