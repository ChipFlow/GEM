# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""Compare timing results between CVC (event-driven) and Loom (GPU) simulators.

Parses CVC's RESULT: lines and Loom's timed VCD to compare signal arrival
times. The primary comparison is Loom's Q arrival (full path from CLK edge
through combo logic to output register Q) vs CVC's total_delay RESULT
(CLK -> dff_out.D path measured by the CVC testbench).

Timing model differences:
  CVC VCD:  Q transitions at CLK + dff_out_CLK_to_Q (~360ps) because CVC is
            event-driven and models DFF capture/output separately.
  Loom VCD: Q transitions at CLK + full_combo_path (~1323ps) because Loom
            propagates arrival times through the entire combinational path.

So the meaningful comparison is Loom Q arrival vs CVC total_delay, NOT
Loom Q arrival vs CVC Q arrival from VCD.

Usage:
    uv run scripts/compare_timing.py \
        --cvc-log cvc_output.log \
        --cvc-vcd cvc_inv_chain_output.vcd \
        --loom-vcd loom_timed_output.vcd \
        --clock-period-ps 10000 \
        --clock-signal CLK \
        --output-signal Q
"""
from __future__ import annotations

import argparse
import json
import logging
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# CVC log parser
# ---------------------------------------------------------------------------

def parse_cvc_results(log_path: Path) -> dict[str, int]:
    """Parse RESULT: key=value lines from CVC stdout log.

    Returns dict like {"clk_to_q": 350, "chain_delay": 973, "total_delay": 1323}.
    """
    results: dict[str, int] = {}
    text = log_path.read_text()
    for m in re.finditer(r"^RESULT:\s*(\w+)=(\d+)", text, re.MULTILINE):
        results[m.group(1)] = int(m.group(2))
    return results


# ---------------------------------------------------------------------------
# Minimal VCD parser
# ---------------------------------------------------------------------------

@dataclass
class VCDSignal:
    name: str
    id_code: str
    size: int = 1
    scope: str = ""


@dataclass
class VCDData:
    timescale_ps: float = 1.0
    signals: dict[str, VCDSignal] = field(default_factory=dict)  # id_code -> signal
    transitions: dict[str, list[tuple[int, str]]] = field(default_factory=dict)  # signal_name -> [(time_ps, value)]


def _parse_timescale(line: str) -> float:
    """Convert timescale string like '1ps' or '1 ns' to picoseconds."""
    line = line.strip()
    m = re.match(r"(\d+)\s*(ps|ns|us|ms|s)", line)
    if not m:
        return 1.0
    ratio = int(m.group(1))
    unit = m.group(2)
    multiplier = {"ps": 1, "ns": 1000, "us": 1_000_000, "ms": 1_000_000_000, "s": 1_000_000_000_000}
    return ratio * multiplier[unit]


def parse_vcd(vcd_path: Path) -> VCDData:
    """Parse a VCD file, returning signal definitions and transitions.

    Handles $timescale, $scope/$upscope, $var, $enddefinitions,
    $dumpvars, #timestamp, and scalar value changes (0x, 1x, xx).
    """
    data = VCDData()
    text = vcd_path.read_text()
    lines = text.splitlines()

    in_header = True
    in_dumpvars = False
    current_scope: list[str] = []
    current_time = 0
    timescale_buf: list[str] = []
    in_timescale = False

    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue

        # --- Header parsing ---
        if in_header:
            if in_timescale:
                if "$end" in stripped:
                    ts_text = " ".join(timescale_buf)
                    ts_text = ts_text.replace("$timescale", "").replace("$end", "").strip()
                    data.timescale_ps = _parse_timescale(ts_text)
                    in_timescale = False
                else:
                    timescale_buf.append(stripped)
                continue

            if stripped.startswith("$timescale"):
                if "$end" in stripped:
                    ts_text = stripped.replace("$timescale", "").replace("$end", "").strip()
                    data.timescale_ps = _parse_timescale(ts_text)
                else:
                    in_timescale = True
                    timescale_buf = [stripped]
                continue

            if stripped.startswith("$scope"):
                m = re.match(r"\$scope\s+\w+\s+(\S+)\s+\$end", stripped)
                if m:
                    current_scope.append(m.group(1))
                continue

            if stripped.startswith("$upscope"):
                if current_scope:
                    current_scope.pop()
                continue

            if stripped.startswith("$var"):
                m = re.match(r"\$var\s+\w+\s+(\d+)\s+(\S+)\s+(\S+)(?:\s+\[.*?\])?\s+\$end", stripped)
                if m:
                    size = int(m.group(1))
                    id_code = m.group(2)
                    name = m.group(3)
                    scope = ".".join(current_scope)
                    full_name = f"{scope}.{name}" if scope else name
                    sig = VCDSignal(name=full_name, id_code=id_code, size=size, scope=scope)
                    data.signals[id_code] = sig
                    data.transitions[full_name] = []
                continue

            if stripped.startswith("$enddefinitions"):
                in_header = False
                continue
            continue

        # --- Body parsing ---
        if stripped.startswith("$dumpvars"):
            in_dumpvars = True
            continue

        if stripped == "$end" and in_dumpvars:
            in_dumpvars = False
            continue

        if stripped.startswith("#"):
            try:
                current_time = int(stripped[1:])
            except ValueError:
                pass
            continue

        # Scalar value change: 0!, 1!, x!, etc.
        m = re.match(r"([01xXzZ])(\S+)", stripped)
        if m:
            val = m.group(1).lower()
            id_code = m.group(2)
            if id_code in data.signals:
                sig = data.signals[id_code]
                time_ps = int(current_time * data.timescale_ps)
                data.transitions[sig.name].append((time_ps, val))
            continue

    return data


# ---------------------------------------------------------------------------
# Arrival measurement
# ---------------------------------------------------------------------------

def find_signal(vcd: VCDData, name: str) -> str | None:
    """Find a signal by short name (case-insensitive, searches all scopes)."""
    name_lower = name.lower()
    for sig in vcd.signals.values():
        leaf = sig.name.rsplit(".", 1)[-1]
        if leaf.lower() == name_lower or sig.name.lower() == name_lower:
            return sig.name
    return None


def measure_arrivals(
    vcd: VCDData,
    clock_signal: str,
    output_signal: str,
    clock_period_ps: int,
) -> list[tuple[int, int, str]]:
    """Measure output signal arrival times relative to preceding clock rising edges.

    Returns list of (cycle_number, arrival_ps, value) tuples.
    """
    clock_name = find_signal(vcd, clock_signal)
    output_name = find_signal(vcd, output_signal)

    if clock_name is None:
        log.warning("Clock signal '%s' not found in VCD. Available: %s",
                     clock_signal, [s.name for s in vcd.signals.values()])
        return []
    if output_name is None:
        log.warning("Output signal '%s' not found in VCD. Available: %s",
                     output_signal, [s.name for s in vcd.signals.values()])
        return []

    # Collect clock rising edges
    clock_edges: list[int] = []
    prev_clk = "0"
    for time_ps, val in vcd.transitions[clock_name]:
        if prev_clk == "0" and val == "1":
            clock_edges.append(time_ps)
        prev_clk = val

    if not clock_edges:
        log.warning("No clock rising edges found for '%s'", clock_name)
        return []

    # For each output transition, find the preceding clock edge
    arrivals: list[tuple[int, int, str]] = []
    for time_ps, val in vcd.transitions[output_name]:
        if val == "x":
            continue

        preceding_edge = None
        for edge in reversed(clock_edges):
            if edge <= time_ps:
                preceding_edge = edge
                break

        if preceding_edge is not None:
            arrival = time_ps - preceding_edge
            cycle = clock_edges.index(preceding_edge)
            arrivals.append((cycle, arrival, val))

    return arrivals


def measure_arrivals_no_clock(
    vcd: VCDData,
    output_signal: str,
    clock_period_ps: int,
) -> list[tuple[int, int, str]]:
    """Measure output signal arrival times when VCD has no clock signal.

    Infers clock edges from clock period. Assumes first rising edge at
    clock_period_ps / 2 (matching stimulus pattern: CLK=0 at t=0).
    """
    output_name = find_signal(vcd, output_signal)
    if output_name is None:
        log.warning("Output signal '%s' not found in VCD. Available: %s",
                     output_signal, [s.name for s in vcd.signals.values()])
        return []

    half_period = clock_period_ps // 2

    arrivals: list[tuple[int, int, str]] = []
    for time_ps, val in vcd.transitions[output_name]:
        if val == "x":
            continue

        if time_ps < half_period:
            # Before first clock edge — use virtual edge at 0
            cycle = 0
            arrival = time_ps
        else:
            adjusted = time_ps - half_period
            cycle = adjusted // clock_period_ps + 1
            edge_time = half_period + (cycle - 1) * clock_period_ps
            arrival = time_ps - edge_time

        arrivals.append((cycle, arrival, val))

    return arrivals


# ---------------------------------------------------------------------------
# Comparison and reporting
# ---------------------------------------------------------------------------

@dataclass
class MetricComparison:
    """One comparison between a CVC reference value and Loom measured value."""
    name: str
    cvc_ps: int
    loom_ps: int
    diff_ps: int
    diff_pct: float
    status: str  # PASS, WARN, FAIL


def compute_status(
    diff_ps: int,
    diff_pct: float,
    max_conservative_pct: float,
    max_optimistic_pct: float,
) -> str:
    """Determine PASS/WARN/FAIL status for a timing comparison."""
    if diff_ps < 0 and abs(diff_pct) > max_optimistic_pct:
        return "FAIL"  # Loom is optimistic (underestimates delay)
    if diff_pct > max_conservative_pct:
        return "WARN"  # Loom is too conservative
    return "PASS"


def format_report(
    cvc_results: dict[str, int],
    metric_comparisons: list[MetricComparison],
    cvc_vcd_arrivals: list[tuple[int, int, str]],
    loom_vcd_arrivals: list[tuple[int, int, str]],
    clock_signal: str,
    output_signal: str,
) -> str:
    """Format the comparison report."""
    lines: list[str] = []
    lines.append("=== Timing Comparison: CVC vs Loom (inv_chain_pnr) ===")
    lines.append("")

    # CVC reference values
    lines.append("CVC Reference (from RESULT lines):")
    if cvc_results:
        for key in ["clk_to_q", "chain_delay", "total_delay"]:
            if key in cvc_results:
                lines.append(f"  {key}: {cvc_results[key]} ps")
    else:
        lines.append("  (no RESULT lines found)")
    lines.append("")

    # Primary comparison: structural metrics
    lines.append("Primary Comparison (Loom Q arrival vs CVC total_delay):")
    lines.append("  Loom's timed VCD propagates the full combo path delay to Q,")
    lines.append("  so Loom's Q arrival corresponds to CVC's total_delay metric.")
    lines.append("")
    if metric_comparisons:
        lines.append(f"  | {'Metric':>20} | {'CVC (ps)':>8} | {'Loom (ps)':>9} | {'Diff (ps)':>9} | {'Diff (%)':>8} | {'Status':>6} |")
        lines.append(f"  |{'-'*22}|{'-'*10}|{'-'*11}|{'-'*11}|{'-'*10}|{'-'*8}|")
        for mc in metric_comparisons:
            lines.append(
                f"  | {mc.name:>20} | {mc.cvc_ps:>8} | {mc.loom_ps:>9} "
                f"| {mc.diff_ps:>+9d} | {mc.diff_pct:>+7.1f}% | {mc.status:>6} |"
            )
    else:
        lines.append("  (no comparable metrics)")
    lines.append("")

    # Informational: raw VCD arrivals
    lines.append(f"Raw VCD {output_signal} Arrivals (informational):")
    if cvc_vcd_arrivals:
        # Deduplicate to show unique arrival values
        cvc_unique = sorted(set(a for _, a, _ in cvc_vcd_arrivals))
        lines.append(f"  CVC (dff_out CLK->Q only): {cvc_unique} ps")
    else:
        lines.append("  CVC: (no arrivals measured)")
    if loom_vcd_arrivals:
        loom_unique = sorted(set(a for _, a, _ in loom_vcd_arrivals))
        lines.append(f"  Loom (full combo path):     {loom_unique} ps")
    else:
        lines.append("  Loom: (no arrivals measured)")
    lines.append("")

    # Not yet comparable
    lines.append("Not Yet Comparable (internal paths -- requires future --timing-report):")
    clk_to_q = cvc_results.get("clk_to_q")
    chain = cvc_results.get("chain_delay")
    if clk_to_q is not None:
        lines.append(f"  clk_to_q (dff_in CLK->Q): CVC={clk_to_q}ps, Loom=N/A")
    if chain is not None:
        lines.append(f"  chain_delay (q1->c[15]):   CVC={chain}ps, Loom=N/A")
    lines.append("")

    # Overall result
    pass_count = sum(1 for mc in metric_comparisons if mc.status == "PASS")
    warn_count = sum(1 for mc in metric_comparisons if mc.status == "WARN")
    fail_count = sum(1 for mc in metric_comparisons if mc.status == "FAIL")
    total_count = len(metric_comparisons)

    if fail_count > 0:
        overall = "FAIL"
    elif warn_count > 0:
        overall = "WARN"
    elif total_count > 0:
        overall = "PASS"
    else:
        overall = "SKIP"

    lines.append(f"RESULT: {overall} ({pass_count}/{total_count} comparable metrics within threshold)")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description="Compare CVC vs Loom timing results")
    parser.add_argument("--cvc-log", type=Path, required=True,
                        help="CVC stdout log with RESULT: lines")
    parser.add_argument("--cvc-vcd", type=Path, required=True,
                        help="CVC output VCD")
    parser.add_argument("--loom-vcd", type=Path, required=True,
                        help="Loom --timing-vcd output")
    parser.add_argument("--clock-period-ps", type=int, required=True,
                        help="Clock period in picoseconds")
    parser.add_argument("--clock-signal", default="CLK",
                        help="Clock signal name (default: CLK)")
    parser.add_argument("--output-signal", default="Q",
                        help="Output signal to compare (default: Q)")
    parser.add_argument("--max-conservative-pct", type=float, default=15.0,
                        help="Max acceptable overestimate %% (default: 15)")
    parser.add_argument("--max-optimistic-pct", type=float, default=5.0,
                        help="Max acceptable underestimate %% (default: 5)")
    parser.add_argument("--output-json", type=Path, default=None,
                        help="Write machine-readable results to JSON")
    args = parser.parse_args()

    # Parse CVC log
    cvc_results = parse_cvc_results(args.cvc_log)
    log.info("CVC RESULT values: %s", cvc_results)

    # Parse VCDs
    cvc_vcd = parse_vcd(args.cvc_vcd)
    loom_vcd = parse_vcd(args.loom_vcd)

    log.info("CVC VCD: %d signals, timescale=%.0f ps",
             len(cvc_vcd.signals), cvc_vcd.timescale_ps)
    log.info("Loom VCD: %d signals, timescale=%.0f ps",
             len(loom_vcd.signals), loom_vcd.timescale_ps)

    # Measure arrivals from VCDs
    cvc_clock = find_signal(cvc_vcd, args.clock_signal)
    if cvc_clock:
        cvc_arrivals = measure_arrivals(
            cvc_vcd, args.clock_signal, args.output_signal, args.clock_period_ps)
    else:
        log.info("CVC VCD has no clock signal; inferring edges from period")
        cvc_arrivals = measure_arrivals_no_clock(
            cvc_vcd, args.output_signal, args.clock_period_ps)

    loom_clock = find_signal(loom_vcd, args.clock_signal)
    if loom_clock:
        loom_arrivals = measure_arrivals(
            loom_vcd, args.clock_signal, args.output_signal, args.clock_period_ps)
    else:
        log.info("Loom VCD has no clock signal; inferring edges from period")
        loom_arrivals = measure_arrivals_no_clock(
            loom_vcd, args.output_signal, args.clock_period_ps)

    log.info("CVC VCD Q arrivals: %s", cvc_arrivals)
    log.info("Loom VCD Q arrivals: %s", loom_arrivals)

    # Primary comparison: Loom's steady-state Q arrival vs CVC total_delay
    # Loom's Q arrival includes the full combo path, matching CVC's total_delay.
    metric_comparisons: list[MetricComparison] = []

    # Get Loom's steady-state arrival (skip cycle 0 which may be initial state)
    loom_steady = [a for c, a, _ in loom_arrivals if c > 0]
    if not loom_steady:
        loom_steady = [a for _, a, _ in loom_arrivals]

    if loom_steady and "total_delay" in cvc_results:
        # Use the most common (modal) Loom arrival as the representative value
        loom_representative = max(set(loom_steady), key=loom_steady.count)
        cvc_total = cvc_results["total_delay"]
        diff_ps = loom_representative - cvc_total
        diff_pct = (diff_ps / cvc_total * 100) if cvc_total > 0 else 0.0

        metric_comparisons.append(MetricComparison(
            name="Q arrival (full path)",
            cvc_ps=cvc_total,
            loom_ps=loom_representative,
            diff_ps=diff_ps,
            diff_pct=diff_pct,
            status=compute_status(diff_ps, diff_pct,
                                  args.max_conservative_pct, args.max_optimistic_pct),
        ))

    # Report
    report = format_report(
        cvc_results, metric_comparisons, cvc_arrivals, loom_arrivals,
        args.clock_signal, args.output_signal)
    print(report)

    # JSON output
    if args.output_json:
        json_data = {
            "cvc_results": cvc_results,
            "loom_vcd_arrivals": [
                {"cycle": c, "arrival_ps": a, "value": v}
                for c, a, v in loom_arrivals
            ],
            "cvc_vcd_arrivals": [
                {"cycle": c, "arrival_ps": a, "value": v}
                for c, a, v in cvc_arrivals
            ],
            "metric_comparisons": [
                {
                    "name": mc.name,
                    "cvc_ps": mc.cvc_ps,
                    "loom_ps": mc.loom_ps,
                    "diff_ps": mc.diff_ps,
                    "diff_pct": mc.diff_pct,
                    "status": mc.status,
                }
                for mc in metric_comparisons
            ],
            "overall": "FAIL" if any(mc.status == "FAIL" for mc in metric_comparisons)
                       else "WARN" if any(mc.status == "WARN" for mc in metric_comparisons)
                       else "PASS" if metric_comparisons
                       else "SKIP",
        }
        args.output_json.write_text(json.dumps(json_data, indent=2))
        log.info("JSON results written to %s", args.output_json)

    # Exit code
    if any(mc.status == "FAIL" for mc in metric_comparisons):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
