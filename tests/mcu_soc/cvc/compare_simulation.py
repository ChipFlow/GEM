# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""Compare Jacquard and CVC simulation outputs for MCU SoC validation.

Functional comparison: Reads both VCDs, extracts gpio_out signals at each
clock edge, and reports differences. Handles format differences:
  - Jacquard: individual 1-bit signals with multi-char VCD codes (e.g., %-)
  - CVC: 44-bit bus gpio_out[43:0] with potential X values

Timing comparison (optional): When --jacquard-timing-vcd is provided, compares
sub-cycle arrival times between Jacquard and CVC, reporting statistics and
identifying timing-critical transitions near clock edges.

When --config is provided, the port_mapping.outputs from the JSON config is
used to translate Jacquard internal port names (e.g. io$soc_gpio_0_gpio$o[0])
back to gpio_out[N] indices.

Usage:
    python3 compare_simulation.py <jacquard.vcd> <cvc.vcd> [--skip-cycles N] [--config <config.json>] [--skip-bits 0-5]
    python3 compare_simulation.py <jacquard.vcd> <cvc.vcd> --jacquard-timing-vcd <timing.vcd> [...]
"""

import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, median


class VCDResult:
    """Parsed gpio_out changes and metadata from a VCD file."""
    def __init__(self, changes: list[tuple[int, int]], last_time: int):
        self.changes = changes  # [(timestamp, value), ...]
        self.last_time = last_time  # last timestamp in VCD


def build_output_port_mapping(config_path: Path) -> dict[str, int]:
    """Build internal_name → gpio_number mapping from sim config outputs.

    The config's port_mapping.outputs maps gpio_number → internal_name.
    We invert it so internal_name → gpio_number.
    """
    with open(config_path) as f:
        config = json.load(f)

    mapping: dict[str, int] = {}
    pm = config.get("port_mapping", {})
    for gpio_num_str, internal_name in pm.get("outputs", {}).items():
        mapping[internal_name] = int(gpio_num_str)
    return mapping


def parse_jacquard_vcd(
    path: Path,
    width: int = 44,
    output_port_map: dict[str, int] | None = None,
) -> VCDResult:
    """Parse Jacquard VCD and return gpio_out value changes + last timestamp.

    Jacquard VCD has individual 1-bit signals. In traditional mode:
        $var wire 1 %- gpio_out[1] $end
    In port-mapped mode (Jacquard internal names):
        $var wire 1 %- io$soc_gpio_0_gpio$o [0] $end

    When output_port_map is provided, internal names are mapped to GPIO indices.
    """
    code_to_bit: dict[str, int] = {}
    in_header = True
    results: list[tuple[int, int]] = []
    current_time = 0
    last_time = 0
    state = [0] * width
    matched_signals = 0

    with open(path) as f:
        for line in f:
            stripped = line.strip()
            if in_header:
                if stripped == "$enddefinitions $end":
                    in_header = False
                    continue
                # Try traditional gpio_out[N] format first
                m = re.match(
                    r'\$var\s+wire\s+1\s+(\S+)\s+gpio_out\s*\[(\d+)\]\s+\$end',
                    stripped,
                )
                if m:
                    code_to_bit[m.group(1)] = int(m.group(2))
                    matched_signals += 1
                    continue
                # Try port-mapped format if mapping provided
                if output_port_map:
                    m = re.match(
                        r'\$var\s+wire\s+1\s+(\S+)\s+(.+?)\s*\$end',
                        stripped,
                    )
                    if m:
                        vid = m.group(1)
                        name = m.group(2).strip()
                        # Normalize "name [N]" → "name[N]"
                        name = re.sub(r'\s+\[(\d+)\]$', r'[\1]', name)
                        if name in output_port_map:
                            code_to_bit[vid] = output_port_map[name]
                            matched_signals += 1
                continue

            if stripped.startswith('#'):
                current_time = int(stripped[1:])
                last_time = current_time
                continue

            if stripped and stripped[0] in '01xXzZ':
                val_char = stripped[0]
                code = stripped[1:]
                if code in code_to_bit:
                    bit = code_to_bit[code]
                    new_val = 1 if val_char == '1' else 0
                    if state[bit] != new_val:
                        state[bit] = new_val
                        val = sum(state[i] << i for i in range(width))
                        results.append((current_time, val))

    print(f"  Matched {matched_signals} output signals to gpio_out bits")
    return VCDResult(results, last_time)


def parse_cvc_vcd(path: Path) -> VCDResult:
    """Parse CVC VCD and return gpio_out value changes + last timestamp.

    CVC VCD has 44-bit bus:
        $var wire 44 ! gpio_out [43:0] $end
    X values are treated as 0.
    """
    gpio_out_code = None
    in_header = True
    results: list[tuple[int, int]] = []
    current_time = 0
    last_time = 0

    with open(path) as f:
        for line in f:
            stripped = line.strip()
            if in_header:
                if stripped == "$enddefinitions $end":
                    in_header = False
                    continue
                m = re.match(
                    r'\$var\s+wire\s+44\s+(\S+)\s+gpio_out\s',
                    stripped,
                )
                if m:
                    gpio_out_code = m.group(1)
                continue

            if stripped.startswith('#'):
                current_time = int(stripped[1:])
                last_time = current_time
                continue

            if stripped.startswith('b') and gpio_out_code:
                parts = stripped.split()
                if len(parts) == 2 and parts[1] == gpio_out_code:
                    binary = parts[0][1:]
                    val = 0
                    for ch in binary:
                        val <<= 1
                        if ch == '1':
                            val |= 1
                    results.append((current_time, val))

    return VCDResult(results, last_time)


@dataclass
class BitTransition:
    """A single bit-level transition with timing info."""
    timestamp: int   # ps
    bit: int         # gpio_out bit index
    new_value: int   # 0 or 1
    cycle: int       # clock cycle this falls in
    arrival_ps: int  # offset from preceding rising clock edge


def parse_jacquard_timing_transitions(
    path: Path,
    clock_period: int,
    width: int = 44,
    output_port_map: dict[str, int] | None = None,
) -> list[BitTransition]:
    """Parse Jacquard timing VCD into per-bit transitions with arrival offsets."""
    code_to_bit: dict[str, int] = {}
    in_header = True
    transitions: list[BitTransition] = []
    current_time = 0
    state = [0] * width
    matched_signals = 0

    with open(path) as f:
        for line in f:
            stripped = line.strip()
            if in_header:
                if stripped == "$enddefinitions $end":
                    in_header = False
                    continue
                m = re.match(
                    r'\$var\s+wire\s+1\s+(\S+)\s+gpio_out\s*\[(\d+)\]\s+\$end',
                    stripped,
                )
                if m:
                    code_to_bit[m.group(1)] = int(m.group(2))
                    matched_signals += 1
                    continue
                if output_port_map:
                    m = re.match(
                        r'\$var\s+wire\s+1\s+(\S+)\s+(.+?)\s*\$end',
                        stripped,
                    )
                    if m:
                        vid = m.group(1)
                        name = m.group(2).strip()
                        name = re.sub(r'\s+\[(\d+)\]$', r'[\1]', name)
                        if name in output_port_map:
                            code_to_bit[vid] = output_port_map[name]
                            matched_signals += 1
                continue

            if stripped.startswith('#'):
                current_time = int(stripped[1:])
                continue

            if stripped and stripped[0] in '01xXzZ':
                val_char = stripped[0]
                code = stripped[1:]
                if code in code_to_bit:
                    bit = code_to_bit[code]
                    new_val = 1 if val_char == '1' else 0
                    if state[bit] != new_val:
                        state[bit] = new_val
                        cycle = current_time // clock_period
                        arrival = current_time - cycle * clock_period
                        transitions.append(BitTransition(
                            timestamp=current_time,
                            bit=bit,
                            new_value=new_val,
                            cycle=cycle,
                            arrival_ps=arrival,
                        ))

    print(f"  Matched {matched_signals} signals, {len(transitions)} bit transitions")
    return transitions


def parse_cvc_timing_transitions(
    path: Path,
    clock_period: int,
) -> list[BitTransition]:
    """Parse CVC timing VCD into per-bit transitions with arrival offsets.

    Decomposes 44-bit bus changes into individual bit transitions.
    """
    gpio_out_code = None
    in_header = True
    transitions: list[BitTransition] = []
    current_time = 0
    prev_val = 0
    width = 44

    with open(path) as f:
        for line in f:
            stripped = line.strip()
            if in_header:
                if stripped == "$enddefinitions $end":
                    in_header = False
                    continue
                m = re.match(
                    r'\$var\s+wire\s+44\s+(\S+)\s+gpio_out\s',
                    stripped,
                )
                if m:
                    gpio_out_code = m.group(1)
                continue

            if stripped.startswith('#'):
                current_time = int(stripped[1:])
                continue

            if stripped.startswith('b') and gpio_out_code:
                parts = stripped.split()
                if len(parts) == 2 and parts[1] == gpio_out_code:
                    binary = parts[0][1:]
                    val = 0
                    for ch in binary:
                        val <<= 1
                        if ch == '1':
                            val |= 1
                    diff = val ^ prev_val
                    for bit in range(width):
                        if diff & (1 << bit):
                            new_bit_val = (val >> bit) & 1
                            cycle = current_time // clock_period
                            arrival = current_time - cycle * clock_period
                            transitions.append(BitTransition(
                                timestamp=current_time,
                                bit=bit,
                                new_value=new_bit_val,
                                cycle=cycle,
                                arrival_ps=arrival,
                            ))
                    prev_val = val

    print(f"  {len(transitions)} bit transitions")
    return transitions


def run_timing_comparison(
    jacquard_transitions: list[BitTransition],
    cvc_transitions: list[BitTransition],
    clock_period: int,
    skip_cycles: int,
    num_cycles: int,
    skip_bits: set[int],
) -> None:
    """Compare timing between Jacquard and CVC transitions, print report."""

    def group_by_cycle_bit(
        transitions: list[BitTransition],
    ) -> dict[tuple[int, int], int]:
        """Map (cycle, bit) → first arrival_ps in that cycle."""
        result: dict[tuple[int, int], int] = {}
        for t in transitions:
            if t.bit in skip_bits:
                continue
            if t.cycle < skip_cycles or t.cycle >= num_cycles:
                continue
            key = (t.cycle, t.bit)
            if key not in result:
                result[key] = t.arrival_ps
        return result

    jq_grouped = group_by_cycle_bit(jacquard_transitions)
    cvc_grouped = group_by_cycle_bit(cvc_transitions)

    common_keys = set(jq_grouped.keys()) & set(cvc_grouped.keys())
    jq_only = set(jq_grouped.keys()) - set(cvc_grouped.keys())
    cvc_only = set(cvc_grouped.keys()) - set(jq_grouped.keys())

    print(f"\n{'='*60}")
    print("Timing Comparison Report")
    print(f"{'='*60}")
    print(f"  Jacquard transitions (filtered): {len(jq_grouped)}")
    print(f"  CVC transitions (filtered):      {len(cvc_grouped)}")
    print(f"  Common (cycle,bit) pairs:        {len(common_keys)}")
    print(f"  Jacquard-only transitions:       {len(jq_only)}")
    print(f"  CVC-only transitions:        {len(cvc_only)}")

    if not common_keys:
        print("  No common transitions to compare timing.")
        print(f"{'='*60}")
        return

    # Compute differences: (cycle, bit, cvc_arrival, jq_arrival, diff)
    diffs: list[tuple[int, int, int, int, int]] = []
    for key in sorted(common_keys):
        cycle, bit = key
        cvc_arr = cvc_grouped[key]
        jq_arr = jq_grouped[key]
        diff = jq_arr - cvc_arr
        diffs.append((cycle, bit, cvc_arr, jq_arr, diff))

    abs_diffs = [abs(d[4]) for d in diffs]

    print(f"\n  Arrival Time Differences (Jacquard - CVC):")
    print(f"    Mean:   {mean(abs_diffs):,.0f} ps")
    print(f"    Median: {median(abs_diffs):,.0f} ps")
    print(f"    Max:    {max(abs_diffs):,.0f} ps")

    pct_diffs = [100.0 * abs(d[4]) / clock_period for d in diffs]
    print(f"    Mean %: {mean(pct_diffs):.1f}% of clock period")
    print(f"    Max  %: {max(pct_diffs):.1f}% of clock period")

    # Histogram
    bucket_edges = [
        (100, "<100ps"),
        (500, "<500ps"),
        (1000, "<1ns"),
        (5000, "<5ns"),
    ]
    print(f"\n  Difference Distribution:")
    prev_edge = 0
    for threshold, label in bucket_edges:
        count = sum(1 for d in abs_diffs if prev_edge <= d < threshold)
        pct = 100 * count / len(abs_diffs)
        print(f"    {label:>8}: {count:>5} ({pct:.1f}%)")
        prev_edge = threshold
    count = sum(1 for d in abs_diffs if d >= prev_edge)
    pct = 100 * count / len(abs_diffs)
    print(f"    {'>=5ns':>8}: {count:>5} ({pct:.1f}%)")

    # Top discrepancies
    diffs_sorted = sorted(diffs, key=lambda d: abs(d[4]), reverse=True)
    print(f"\n  Top 10 Largest Discrepancies:")
    print(f"    {'Cycle':>7} {'Bit':>4} {'CVC(ps)':>10} {'Jqrd(ps)':>10} {'Diff(ps)':>10} {'%clk':>6}")
    for cycle, bit, cvc_arr, jq_arr, diff in diffs_sorted[:10]:
        pct = 100.0 * abs(diff) / clock_period
        print(f"    {cycle:>7} {bit:>4} {cvc_arr:>10,} {jq_arr:>10,} {diff:>+10,} {pct:>5.1f}%")

    # Timing-critical analysis
    near_edge_threshold = 0.8 * clock_period
    timing_critical: list[tuple[int, int, int, int, str]] = []

    for cycle, bit, cvc_arr, jq_arr, diff in diffs:
        cvc_near = cvc_arr > near_edge_threshold
        jq_near = jq_arr > near_edge_threshold
        if cvc_near != jq_near:
            if jq_near and not cvc_near:
                reason = "Jacquard late (near next edge), CVC safe"
            else:
                reason = "CVC late (near next edge), Jacquard safe"
            timing_critical.append((cycle, bit, cvc_arr, jq_arr, reason))

    # Check for transitions captured in different cycles
    edge_crossings: list[tuple[int, int, int, int]] = []
    for cycle, bit, cvc_arr, jq_arr, diff in diffs:
        cvc_same = cvc_arr < clock_period
        jq_same = jq_arr < clock_period
        if cvc_same != jq_same:
            edge_crossings.append((cycle, bit, cvc_arr, jq_arr))

    print(f"\n  Timing-Critical Analysis:")
    print(f"    Near-edge threshold: {near_edge_threshold:,.0f} ps "
          f"(>{100*0.8:.0f}% of clock period)")
    print(f"    Transitions where simulators disagree on near-edge: "
          f"{len(timing_critical)}")

    if timing_critical:
        print(f"\n    {'Cycle':>7} {'Bit':>4} {'CVC(ps)':>10} {'Jqrd(ps)':>10} Issue")
        for cycle, bit, cvc_arr, jq_arr, reason in timing_critical[:20]:
            print(f"    {cycle:>7} {bit:>4} {cvc_arr:>10,} {jq_arr:>10,} "
                  f"{reason}")
        if len(timing_critical) > 20:
            print(f"    ... ({len(timing_critical) - 20} more)")

    if edge_crossings:
        print(f"\n    Clock edge crossings (different cycle capture): "
              f"{len(edge_crossings)}")
        print(f"    {'Cycle':>7} {'Bit':>4} {'CVC(ps)':>10} {'Jqrd(ps)':>10}")
        for cycle, bit, cvc_arr, jq_arr in edge_crossings[:10]:
            print(f"    {cycle:>7} {bit:>4} {cvc_arr:>10,} {jq_arr:>10,}")
        if len(edge_crossings) > 10:
            print(f"    ... ({len(edge_crossings) - 10} more)")
    else:
        print("    Clock edge crossings: 0 "
              "(all transitions captured in same cycle)")

    print(f"{'='*60}")


def main() -> None:
    # Parse arguments
    args = sys.argv[1:]
    skip_cycles = 0
    config_path: Path | None = None
    num_cycles_override: int | None = None
    skip_bits: set[int] = set()
    jacquard_timing_vcd: Path | None = None
    cvc_timing_vcd: Path | None = None
    positional: list[str] = []

    i = 0
    while i < len(args):
        if args[i] == "--skip-cycles" and i + 1 < len(args):
            skip_cycles = int(args[i + 1])
            i += 2
        elif args[i] == "--config" and i + 1 < len(args):
            config_path = Path(args[i + 1])
            i += 2
        elif args[i] == "--num-cycles" and i + 1 < len(args):
            num_cycles_override = int(args[i + 1])
            i += 2
        elif args[i] == "--skip-bits" and i + 1 < len(args):
            # Parse "0-5" or "0,1,2" or "0-5,10"
            for part in args[i + 1].split(","):
                if "-" in part:
                    lo, hi = part.split("-", 1)
                    skip_bits.update(range(int(lo), int(hi) + 1))
                else:
                    skip_bits.add(int(part))
            i += 2
        elif args[i] in ("--jacquard-timing-vcd", "--loom-timing-vcd") and i + 1 < len(args):
            jacquard_timing_vcd = Path(args[i + 1])
            i += 2
        elif args[i] == "--cvc-timing-vcd" and i + 1 < len(args):
            cvc_timing_vcd = Path(args[i + 1])
            i += 2
        else:
            positional.append(args[i])
            i += 1

    if len(positional) < 2:
        print(
            f"Usage: {sys.argv[0]} <jacquard.vcd> <cvc.vcd> [--skip-cycles N] "
            f"[--config <config.json>] [--num-cycles N] [--skip-bits 0-5]",
            file=sys.stderr,
        )
        sys.exit(1)

    jacquard_path = Path(positional[0])
    cvc_path = Path(positional[1])

    output_port_map: dict[str, int] | None = None
    if config_path:
        output_port_map = build_output_port_mapping(config_path)
        print(f"Loaded output port mapping from {config_path}: {len(output_port_map)} outputs")

    clock_period = 40000  # ps

    print(f"Parsing Jacquard VCD: {jacquard_path}")
    jq_result = parse_jacquard_vcd(jacquard_path, output_port_map=output_port_map)
    jq_changes = jq_result.changes
    print(f"  {len(jq_changes)} gpio_out value changes, last_time={jq_result.last_time}ps")

    print(f"Parsing CVC VCD: {cvc_path}")
    cvc_result = parse_cvc_vcd(cvc_path)
    cvc_changes = cvc_result.changes
    print(f"  {len(cvc_changes)} gpio_out value changes, last_time={cvc_result.last_time}ps")

    # Determine comparison range
    if num_cycles_override:
        num_cycles = num_cycles_override
        max_time = num_cycles * clock_period
        print(f"  Using explicit cycle count: {num_cycles}")
    else:
        # Use the minimum of both last timestamps as the comparison range
        max_time = min(jq_result.last_time, cvc_result.last_time)
        num_cycles = max_time // clock_period + 1

    # Build per-cycle values by interpolating from change lists
    def build_cycle_values(
        changes: list[tuple[int, int]], num_cycles: int
    ) -> list[int]:
        """Build value at each rising clock edge from a list of (time, value)."""
        values = [0] * num_cycles
        change_idx = 0
        current_val = 0
        for cycle in range(num_cycles):
            t = cycle * clock_period
            while change_idx < len(changes) and changes[change_idx][0] <= t:
                current_val = changes[change_idx][1]
                change_idx += 1
            values[cycle] = current_val
        return values

    jq_values = build_cycle_values(jq_changes, num_cycles)
    cvc_values = build_cycle_values(cvc_changes, num_cycles)

    # Build comparison mask (exclude skip_bits)
    compare_mask = sum(1 << i for i in range(44) if i not in skip_bits)
    if skip_bits:
        print(f"  Skipping bits: {sorted(skip_bits)}")

    print(f"\nComparing {num_cycles} cycles (skip first {skip_cycles}):")
    print(f"  Clock period: {clock_period} ps")

    matches = 0
    mismatches = 0
    first_mismatch = None

    for cycle in range(skip_cycles, num_cycles):
        lv = jq_values[cycle] & compare_mask
        cv = cvc_values[cycle] & compare_mask
        if lv == cv:
            matches += 1
        else:
            mismatches += 1
            if mismatches <= 10:
                diff = lv ^ cv
                diff_bits = [i for i in range(44) if diff & (1 << i) and i not in skip_bits]
                t = cycle * clock_period
                print(f"\n  MISMATCH at cycle {cycle} (t={t}ps):")
                print(f"    Jacquard: 0x{lv:011x}")
                print(f"    CVC:  0x{cv:011x}")
                print(f"    Diff bits: {diff_bits}")
                if first_mismatch is None:
                    first_mismatch = cycle
            elif mismatches == 11:
                print(f"\n  ... (suppressing further mismatches)")

    total = matches + mismatches
    print(f"\n{'='*60}")
    bits_label = f"gpio_out excluding bits {sorted(skip_bits)}" if skip_bits else "gpio_out"
    print(f"Comparison Summary ({bits_label}, cycles {skip_cycles}-{num_cycles-1}):")
    print(f"  Cycles compared: {total}")
    if total > 0:
        print(f"  Matches:    {matches} ({100*matches/total:.1f}%)")
        print(f"  Mismatches: {mismatches} ({100*mismatches/total:.1f}%)")
    else:
        print("  No cycles to compare")
    if first_mismatch is not None:
        print(f"  First mismatch at cycle: {first_mismatch}")
    print(f"{'='*60}")

    # --- Timing comparison (optional, informational only) ---
    if jacquard_timing_vcd:
        # Default CVC timing VCD to the same functional CVC VCD
        cvc_timing_path = cvc_timing_vcd if cvc_timing_vcd else cvc_path

        print(f"\nParsing Jacquard timing VCD: {jacquard_timing_vcd}")
        jq_timing = parse_jacquard_timing_transitions(
            jacquard_timing_vcd, clock_period, output_port_map=output_port_map,
        )

        print(f"Parsing CVC timing VCD: {cvc_timing_path}")
        cvc_timing = parse_cvc_timing_transitions(
            cvc_timing_path, clock_period,
        )

        run_timing_comparison(
            jq_timing, cvc_timing, clock_period,
            skip_cycles, num_cycles, skip_bits,
        )

    if mismatches > 0:
        sys.exit(1)


if __name__ == '__main__':
    main()
