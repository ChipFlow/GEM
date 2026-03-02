"""Structured result types for Jacquard CLI output parsing."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path


@dataclass
class DesignStats:
    """Design-level statistics parsed from `=== Design Statistics ===` block."""

    num_aigpins: int = 0
    num_and_gates: int = 0
    num_dffs: int = 0
    num_srams: int = 0
    num_inputs: int = 0
    num_outputs: int = 0
    total_partitions: int = 0
    num_major_stages: int = 0
    num_blocks: int = 0
    reg_io_state_words: int = 0
    sram_storage_words: int = 0
    script_data_words: int = 0
    min_partitions_per_block: int = 0
    max_partitions_per_block: int = 0
    min_script_size: int = 0
    max_script_size: int = 0
    load_balance_ratio: float = 1.0
    xprop_enabled: bool = False
    num_x_capable_partitions: int = 0


@dataclass
class MapResult:
    """Result of a `jacquard map` invocation."""

    success: bool
    returncode: int
    stdout: str
    stderr: str
    elapsed: float  # wall-clock seconds
    gemparts_path: Path | None = None


@dataclass
class SimResult:
    """Result of a `jacquard sim` invocation."""

    success: bool
    returncode: int
    stdout: str
    stderr: str
    elapsed: float  # wall-clock seconds (total process time)
    design_stats: DesignStats | None = None
    gpu_backend: str = ""
    num_cycles: int = 0
    wall_clock_secs: float = 0.0  # GPU sim time reported by Jacquard
    throughput: float = 0.0  # cycles/sec
    state_memory_kb: float = 0.0
    output_signals: int = 0
    output_vcd: Path | None = None


# --- Parsing functions ---

# Regex patterns matching the Rust Display impl in src/sim/report.rs

_RE_AIG = re.compile(
    r"AIG:\s+(\d+)\s+pins,\s+(\d+)\s+AND gates,\s+(\d+)\s+DFFs,\s+(\d+)\s+SRAMs"
)
_RE_IO = re.compile(r"I/O:\s+(\d+)\s+inputs,\s+(\d+)\s+outputs")
_RE_STATE = re.compile(r"State:\s+(\d+)\s+words\s+.*?\+\s+(\d+)\s+words")
_RE_SCRIPT = re.compile(r"Script:\s+(\d+)\s+words")
_RE_PARTITIONS = re.compile(
    r"Partitions:\s+(\d+)\s+total\s+across\s+(\d+)\s+stage\(s\)\s+.*?(\d+)\s+block\(s\)"
)
_RE_PER_BLOCK = re.compile(
    r"Per block:\s+(\d+)-(\d+)\s+partitions,\s+script\s+(\d+)-(\d+)\s+words\s+"
    r"\(balance ratio:\s+([\d.]+)x\)"
)
_RE_XPROP = re.compile(r"X-propagation:\s+(\d+)/(\d+)\s+partitions\s+X-capable")

_RE_GPU_BACKEND = re.compile(r"GPU backend:\s+(\S+)")
_RE_CYCLES = re.compile(r"Cycles simulated:\s+(\d+)")
_RE_WALL_CLOCK = re.compile(r"Wall-clock time:\s+([\d.]+)s")
_RE_THROUGHPUT = re.compile(r"Throughput:\s+([\d.eE+]+)\s+cycles/sec")
_RE_STATE_MEM = re.compile(r"State memory:\s+([\d.]+)\s+KB/cycle")
_RE_OUTPUT = re.compile(r"Output:\s+(\d+)\s+signals\s+.*?→\s+(.+)")


def parse_design_stats(text: str) -> DesignStats | None:
    """Parse a `=== Design Statistics ===` block from CLI output.

    Returns None if the block is not found.
    """
    if "=== Design Statistics ===" not in text:
        return None

    stats = DesignStats()

    m = _RE_AIG.search(text)
    if m:
        stats.num_aigpins = int(m.group(1))
        stats.num_and_gates = int(m.group(2))
        stats.num_dffs = int(m.group(3))
        stats.num_srams = int(m.group(4))

    m = _RE_IO.search(text)
    if m:
        stats.num_inputs = int(m.group(1))
        stats.num_outputs = int(m.group(2))

    m = _RE_STATE.search(text)
    if m:
        stats.reg_io_state_words = int(m.group(1))
        stats.sram_storage_words = int(m.group(2))

    m = _RE_SCRIPT.search(text)
    if m:
        stats.script_data_words = int(m.group(1))

    m = _RE_PARTITIONS.search(text)
    if m:
        stats.total_partitions = int(m.group(1))
        stats.num_major_stages = int(m.group(2))
        stats.num_blocks = int(m.group(3))

    m = _RE_PER_BLOCK.search(text)
    if m:
        stats.min_partitions_per_block = int(m.group(1))
        stats.max_partitions_per_block = int(m.group(2))
        stats.min_script_size = int(m.group(3))
        stats.max_script_size = int(m.group(4))
        stats.load_balance_ratio = float(m.group(5))

    m = _RE_XPROP.search(text)
    if m:
        stats.xprop_enabled = True
        stats.num_x_capable_partitions = int(m.group(1))

    return stats


def parse_sim_summary(text: str) -> dict:
    """Parse a `=== Simulation Summary ===` block from CLI output.

    Returns a dict with parsed fields. Missing fields have default values.
    """
    result: dict = {}

    m = _RE_GPU_BACKEND.search(text)
    if m:
        result["gpu_backend"] = m.group(1)

    m = _RE_CYCLES.search(text)
    if m:
        result["num_cycles"] = int(m.group(1))

    m = _RE_WALL_CLOCK.search(text)
    if m:
        result["wall_clock_secs"] = float(m.group(1))

    m = _RE_THROUGHPUT.search(text)
    if m:
        result["throughput"] = float(m.group(1))

    m = _RE_STATE_MEM.search(text)
    if m:
        result["state_memory_kb"] = float(m.group(1))

    m = _RE_OUTPUT.search(text)
    if m:
        result["output_signals"] = int(m.group(1))
        result["output_vcd"] = Path(m.group(2).strip())

    return result
