#!/usr/bin/env bash
# Regenerate the WS4 corpus goldens (`expected.jtir` + `expected.json`)
# for every entry under tests/timing_ir/corpus/.
#
# Use this when:
#   - The OpenSTA submodule pin is bumped.
#   - The opensta-to-ir Tcl driver / dump format / builder changes.
#   - The timing_ir.fbs schema gets an additive extension that consumers
#     are expected to pick up.
#
# Workflow: run this script, then `git diff` the resulting changes to
# convince yourself the IR diffs are intentional, then commit the new
# goldens alongside whatever produced them. The regression test
# `corpus_designs_match_golden_ir` (in crates/opensta-to-ir) will catch
# unintentional drift on every PR.
#
# Usage:
#   scripts/regenerate-corpus-goldens.sh             # regenerate all
#   scripts/regenerate-corpus-goldens.sh <name>...   # regenerate named entries only
#
# Exits non-zero on any opensta-to-ir failure.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CORPUS_DIR="${REPO_ROOT}/tests/timing_ir/corpus"
SCHEMA="${REPO_ROOT}/crates/timing-ir/schemas/timing_ir.fbs"
OPENSTA_TO_IR="${REPO_ROOT}/crates/opensta-to-ir/target/release/opensta-to-ir"

if [[ ! -x "$OPENSTA_TO_IR" ]]; then
    echo "Building opensta-to-ir release binary..." >&2
    (cd "${REPO_ROOT}/crates/opensta-to-ir" && cargo build --release)
fi

if ! command -v flatc >/dev/null 2>&1; then
    echo "ERROR: flatc not on PATH. brew install flatbuffers (macOS) or apt install flatbuffers-compiler (Debian)." >&2
    exit 1
fi

# Pull a value out of an entry's manifest.toml. Uses Python's tomllib
# rather than adding a TOML dep to bash; tomllib has shipped in stdlib
# since Python 3.11 and the dev environment already has it.
read_manifest() {
    local manifest=$1
    local query=$2
    python3 - "$manifest" "$query" <<'PY'
import sys
import tomllib
with open(sys.argv[1], "rb") as f:
    data = tomllib.load(f)
keys = sys.argv[2].split(".")
node = data
for k in keys:
    if k.endswith("[]"):
        # list-valued — emit one line per item
        items = node[k[:-2]]
        for item in items:
            print(item)
        sys.exit(0)
    node = node[k]
print(node)
PY
}

regenerate_entry() {
    local entry_dir=$1
    local name
    name=$(basename "$entry_dir")
    local manifest="${entry_dir}/manifest.toml"

    if [[ ! -f "$manifest" ]]; then
        echo "skipping ${name}: no manifest.toml" >&2
        return
    fi

    echo "regenerating ${name}..." >&2

    local args=()
    while IFS= read -r lib; do
        [[ -n "$lib" ]] && args+=(--liberty "${entry_dir}/${lib}")
    done < <(read_manifest "$manifest" "opensta_to_ir.liberty[]")

    while IFS= read -r v; do
        [[ -n "$v" ]] && args+=(--verilog "${entry_dir}/${v}")
    done < <(read_manifest "$manifest" "opensta_to_ir.verilog[]")

    local sdf sdc spef top
    sdf=$(read_manifest "$manifest" "opensta_to_ir.sdf" 2>/dev/null || true)
    sdc=$(read_manifest "$manifest" "opensta_to_ir.sdc" 2>/dev/null || true)
    spef=$(read_manifest "$manifest" "opensta_to_ir.spef" 2>/dev/null || true)
    top=$(read_manifest "$manifest" "opensta_to_ir.top")

    [[ -n "$sdf"  ]] && args+=(--sdf  "${entry_dir}/${sdf}")
    [[ -n "$sdc"  ]] && args+=(--sdc  "${entry_dir}/${sdc}")
    [[ -n "$spef" ]] && args+=(--spef "${entry_dir}/${spef}")
    args+=(--top "$top" --output "${entry_dir}/expected.jtir")

    "$OPENSTA_TO_IR" "${args[@]}"

    # Refresh the JSON sidecar via flatc.
    flatc --json --strict-json --raw-binary --defaults-json \
        -o "$entry_dir" "$SCHEMA" -- "${entry_dir}/expected.jtir"
}

if [[ $# -gt 0 ]]; then
    for name in "$@"; do
        regenerate_entry "${CORPUS_DIR}/${name}"
    done
else
    for entry_dir in "$CORPUS_DIR"/*/; do
        regenerate_entry "${entry_dir%/}"
    done
fi

echo "done. Run 'git diff tests/timing_ir/corpus/' to review." >&2
