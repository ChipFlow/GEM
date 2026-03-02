"""Subprocess wrappers for `jacquard map` and `jacquard sim`."""

from __future__ import annotations

import logging
import shutil
import subprocess
import time
from dataclasses import replace
from pathlib import Path

from .config import JacquardConfig
from .errors import BinaryNotFoundError
from .result import MapResult, SimResult, parse_design_stats, parse_sim_summary

log = logging.getLogger(__name__)


def find_jacquard_binary(
    explicit_path: Path | None = None,
    cargo_target_dir: Path | None = None,
) -> Path:
    """Locate the jacquard binary.

    Search order:
    1. Explicit path (if provided)
    2. cargo target directory (release then debug)
    3. PATH lookup via shutil.which
    """
    if explicit_path is not None:
        p = Path(explicit_path)
        if p.is_file():
            return p
        raise BinaryNotFoundError(f"Explicit jacquard binary not found: {p}")

    if cargo_target_dir is not None:
        for profile in ("release", "debug"):
            candidate = Path(cargo_target_dir) / profile / "jacquard"
            if candidate.is_file():
                return candidate

    found = shutil.which("jacquard")
    if found is not None:
        return Path(found)

    raise BinaryNotFoundError(
        "jacquard binary not found. Install it or pass jacquard_bin explicitly."
    )


def _build_map_args(config: JacquardConfig) -> list[str]:
    """Build CLI arguments for `jacquard map`."""
    args: list[str] = ["map"]

    assert config.design.netlist is not None, "design.netlist is required for map"
    args.append(str(config.design.netlist))

    output = config.map.output
    assert output is not None, "map.output is required for map"
    args.append(str(output))

    if config.design.top_module:
        args.extend(["--top-module", config.design.top_module])

    if config.map.level_split:
        args.extend(["--level-split", ",".join(str(x) for x in config.map.level_split)])

    if config.map.max_stage_degrad != 0:
        args.extend(["--max-stage-degrad", str(config.map.max_stage_degrad)])

    if config.map.xprop:
        args.append("--xprop")

    return args


def _build_sim_args(config: JacquardConfig) -> list[str]:
    """Build CLI arguments for `jacquard sim`."""
    args: list[str] = ["sim"]

    assert config.design.netlist is not None, "design.netlist is required for sim"
    args.append(str(config.design.netlist))

    gemparts = config.effective_gemparts()
    assert gemparts is not None, "gemparts (or map.output) is required for sim"
    args.append(str(gemparts))

    assert config.sim.input_vcd is not None, "sim.input_vcd is required for sim"
    args.append(str(config.sim.input_vcd))

    assert config.sim.output_vcd is not None, "sim.output_vcd is required for sim"
    args.append(str(config.sim.output_vcd))

    if config.sim.num_blocks is not None:
        args.extend(["--num-blocks", str(config.sim.num_blocks)])

    if config.design.top_module:
        args.extend(["--top-module", config.design.top_module])

    if config.map.level_split:
        args.extend(["--level-split", ",".join(str(x) for x in config.map.level_split)])

    if config.sim.input_vcd_scope:
        args.extend(["--input-vcd-scope", config.sim.input_vcd_scope])

    if config.sim.output_vcd_scope:
        args.extend(["--output-vcd-scope", config.sim.output_vcd_scope])

    if config.sim.check_with_cpu:
        args.append("--check-with-cpu")

    if config.sim.max_cycles is not None:
        args.extend(["--max-cycles", str(config.sim.max_cycles)])

    if config.sim.xprop:
        args.append("--xprop")

    for pattern in config.sim.dump_signals:
        args.extend(["--dump-signals", pattern])

    if config.sim.dump_scope:
        args.extend(["--dump-scope", config.sim.dump_scope])

    if config.sim.dump_depth is not None:
        args.extend(["--dump-depth", str(config.sim.dump_depth)])

    for k, v in config.sim.plusargs.items():
        args.extend(["--plusarg", f"{k}={v}"])

    if config.sim.sdf is not None and config.sim.sdf.file is not None:
        args.extend(["--sdf", str(config.sim.sdf.file)])
        if config.sim.sdf.corner != "typ":
            args.extend(["--sdf-corner", config.sim.sdf.corner])
        if config.sim.sdf.debug:
            args.append("--sdf-debug")

    if config.sim.timing is not None and config.sim.timing.enabled:
        args.append("--enable-timing")
        if config.sim.timing.clock_period is not None:
            args.extend(["--timing-clock-period", str(config.sim.timing.clock_period)])
        if config.sim.timing.report_violations:
            args.append("--timing-report-violations")

    if config.design.liberty:
        args.extend(["--liberty", str(config.design.liberty)])

    return args


def _apply_overrides(config: JacquardConfig, **overrides: object) -> JacquardConfig:
    """Apply keyword overrides to a config, returning a new copy.

    Supports flat keys that map to nested fields:
      - netlist, top_module, liberty -> design.*
      - output, level_split, max_stage_degrad -> map.*
      - gemparts, input_vcd, output_vcd, num_blocks, max_cycles, xprop,
        dump_signals, dump_scope, dump_depth, plusargs, check_with_cpu -> sim.*
    """
    if not overrides:
        return config

    # Deep copy via replace
    design = replace(config.design)
    map_cfg = replace(config.map)
    sim_cfg = replace(config.sim)

    design_keys = {"netlist", "top_module", "liberty"}
    map_keys = {"output", "level_split", "max_stage_degrad"}
    sim_keys = {
        "gemparts", "input_vcd", "output_vcd", "num_blocks", "max_cycles",
        "xprop", "dump_signals", "dump_scope", "dump_depth", "plusargs",
        "check_with_cpu", "input_vcd_scope", "output_vcd_scope",
    }

    for key, val in overrides.items():
        if key in design_keys:
            if key == "netlist" and val is not None:
                val = Path(val)  # type: ignore[arg-type]
            elif key == "liberty" and val is not None:
                val = Path(val)  # type: ignore[arg-type]
            setattr(design, key, val)
        elif key in map_keys:
            if key == "output" and val is not None:
                val = Path(val)  # type: ignore[arg-type]
            setattr(map_cfg, key, val)
        elif key in sim_keys:
            if key in ("gemparts", "input_vcd", "output_vcd") and val is not None:
                val = Path(val)  # type: ignore[arg-type]
            setattr(sim_cfg, key, val)
        else:
            log.warning("Unknown override key: %s", key)

    return JacquardConfig(design=design, map=map_cfg, sim=sim_cfg)


def map(
    config: JacquardConfig,
    *,
    jacquard_bin: Path | None = None,
    cargo_target_dir: Path | None = None,
    verbose: int = 0,
    quiet: int = 0,
    timeout: float | None = None,
    **overrides: object,
) -> MapResult:
    """Run `jacquard map` as a subprocess.

    Args:
        config: Project configuration.
        jacquard_bin: Explicit path to jacquard binary.
        cargo_target_dir: Cargo target directory for binary lookup.
        verbose: Verbosity level (number of -v flags).
        quiet: Quiet level (number of -q flags).
        timeout: Subprocess timeout in seconds.
        **overrides: Override any config field for this run.

    Returns:
        MapResult with success status and parsed output.

    Raises:
        BinaryNotFoundError: If jacquard binary cannot be found.
        MappingError: If map fails and caller wants exception on failure.
    """
    binary = find_jacquard_binary(jacquard_bin, cargo_target_dir)
    effective = _apply_overrides(config, **overrides)

    cmd = [str(binary)]
    cmd.extend(["-v"] * verbose)
    cmd.extend(["-q"] * quiet)
    cmd.extend(_build_map_args(effective))

    log.info("Running: %s", " ".join(cmd))

    start = time.monotonic()
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as e:
        elapsed = time.monotonic() - start
        return MapResult(
            success=False,
            returncode=-1,
            stdout=e.stdout or "",
            stderr=f"Timeout after {elapsed:.1f}s",
            elapsed=elapsed,
        )

    elapsed = time.monotonic() - start

    return MapResult(
        success=proc.returncode == 0,
        returncode=proc.returncode,
        stdout=proc.stdout,
        stderr=proc.stderr,
        elapsed=elapsed,
        gemparts_path=effective.map.output if proc.returncode == 0 else None,
    )


def sim(
    config: JacquardConfig,
    *,
    jacquard_bin: Path | None = None,
    cargo_target_dir: Path | None = None,
    verbose: int = 0,
    quiet: int = 0,
    timeout: float | None = None,
    **overrides: object,
) -> SimResult:
    """Run `jacquard sim` as a subprocess.

    Args:
        config: Project configuration.
        jacquard_bin: Explicit path to jacquard binary.
        cargo_target_dir: Cargo target directory for binary lookup.
        verbose: Verbosity level (number of -v flags).
        quiet: Quiet level (number of -q flags).
        timeout: Subprocess timeout in seconds.
        **overrides: Override any config field for this run.

    Returns:
        SimResult with success status and parsed output.

    Raises:
        BinaryNotFoundError: If jacquard binary cannot be found.
    """
    binary = find_jacquard_binary(jacquard_bin, cargo_target_dir)
    effective = _apply_overrides(config, **overrides)

    cmd = [str(binary)]
    cmd.extend(["-v"] * verbose)
    cmd.extend(["-q"] * quiet)
    cmd.extend(_build_sim_args(effective))

    log.info("Running: %s", " ".join(cmd))

    start = time.monotonic()
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as e:
        elapsed = time.monotonic() - start
        return SimResult(
            success=False,
            returncode=-1,
            stdout=e.stdout or "",
            stderr=f"Timeout after {elapsed:.1f}s",
            elapsed=elapsed,
        )

    elapsed = time.monotonic() - start

    # Combine stdout+stderr for parsing (Rust may log to either)
    combined = proc.stdout + "\n" + proc.stderr

    design_stats = parse_design_stats(combined)
    summary = parse_sim_summary(combined)

    return SimResult(
        success=proc.returncode == 0,
        returncode=proc.returncode,
        stdout=proc.stdout,
        stderr=proc.stderr,
        elapsed=elapsed,
        design_stats=design_stats,
        gpu_backend=summary.get("gpu_backend", ""),
        num_cycles=summary.get("num_cycles", 0),
        wall_clock_secs=summary.get("wall_clock_secs", 0.0),
        throughput=summary.get("throughput", 0.0),
        state_memory_kb=summary.get("state_memory_kb", 0.0),
        output_signals=summary.get("output_signals", 0),
        output_vcd=summary.get("output_vcd", effective.sim.output_vcd),
    )
