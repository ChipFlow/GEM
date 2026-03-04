# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""SDF timing path tracer.

Parses an SDF file and traces timing paths through INTERCONNECT and IOPATH
entries. Useful for understanding clock tree delays, data path timing, and
debugging timing discrepancies between simulators.

Usage:
    uv run scripts/sdf_trace.py <sdf_file> clock-tree <dff_instance>
    uv run scripts/sdf_trace.py <sdf_file> trace-back <instance.pin> [-d N]
    uv run scripts/sdf_trace.py <sdf_file> trace-fwd <instance.pin> [-d N]
    uv run scripts/sdf_trace.py <sdf_file> cell-info <instance>
    uv run scripts/sdf_trace.py <sdf_file> output-path <dff_instance> <output_port>

Examples:
    # Trace clock tree to a specific DFF
    uv run scripts/sdf_trace.py tests/mcu_soc/data/6_final.sdf clock-tree _58172_

    # Trace backwards from a pin
    uv run scripts/sdf_trace.py tests/mcu_soc/data/6_final.sdf trace-back _58172_.CLK -d 10

    # Show all IOPATH/INTERCONNECT info for a cell
    uv run scripts/sdf_trace.py tests/mcu_soc/data/6_final.sdf cell-info _58172_

    # Trace from DFF Q output to an output port
    uv run scripts/sdf_trace.py tests/mcu_soc/data/6_final.sdf output-path _58172_ io\\$soc_flash_clk\\$o
"""

import re
import sys
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class SdfDelay:
    rise_ps: int
    fall_ps: int

    def max_ps(self) -> int:
        return max(self.rise_ps, self.fall_ps)

    def __str__(self) -> str:
        if self.rise_ps == self.fall_ps:
            return f"{self.rise_ps}ps"
        return f"{self.rise_ps}/{self.fall_ps}ps"


@dataclass
class SdfInterconnect:
    source: str  # "instance.pin" or bare port name
    dest: str    # "instance.pin"
    delay: SdfDelay

    @property
    def source_instance(self) -> str:
        """Extract instance name from source (empty if it's a port)."""
        if '.' in self.source:
            return self.source[:self.source.rfind('.')]
        return ""

    @property
    def source_pin(self) -> str:
        if '.' in self.source:
            return self.source[self.source.rfind('.') + 1:]
        return self.source

    @property
    def dest_instance(self) -> str:
        if '.' in self.dest:
            return self.dest[:self.dest.rfind('.')]
        return ""

    @property
    def dest_pin(self) -> str:
        if '.' in self.dest:
            return self.dest[self.dest.rfind('.') + 1:]
        return self.dest


@dataclass
class SdfIopath:
    input_pin: str
    output_pin: str
    delay: SdfDelay


@dataclass
class SdfCellInfo:
    celltype: str
    instance: str
    iopaths: list[SdfIopath] = field(default_factory=list)


@dataclass
class SdfData:
    timescale_ns: float = 1.0
    cells: dict[str, SdfCellInfo] = field(default_factory=dict)
    interconnects: list[SdfInterconnect] = field(default_factory=list)
    # Indexes built after parsing
    ic_by_dest_inst: dict[str, list[SdfInterconnect]] = field(default_factory=dict)
    ic_by_source_inst: dict[str, list[SdfInterconnect]] = field(default_factory=dict)

    def build_indexes(self) -> None:
        for ic in self.interconnects:
            di = ic.dest_instance
            si = ic.source_instance if ic.source_instance else ic.source
            self.ic_by_dest_inst.setdefault(di, []).append(ic)
            self.ic_by_source_inst.setdefault(si, []).append(ic)


def parse_delay_triple(s: str, timescale_ns: float) -> int:
    """Parse '(min:typ:max)' or '(val)' → picoseconds (typ corner)."""
    s = s.strip().strip('()')
    if ':' in s:
        parts = s.split(':')
        val = float(parts[1])  # typ
    elif s:
        val = float(s)
    else:
        return 0
    return int(val * timescale_ns * 1000)  # ns → ps


def parse_sdf(path: Path, corner: str = "typ") -> SdfData:
    """Parse an SDF file into structured data."""
    sdf = SdfData()
    corner_idx = {"min": 0, "typ": 1, "max": 2}.get(corner, 1)

    current_celltype = ""
    current_instance = ""
    in_cell = False
    ic_count = 0
    cell_count = 0

    with open(path) as f:
        for line in f:
            stripped = line.strip()

            # Timescale
            m = re.match(r'\(TIMESCALE\s+(\S+)\)', stripped)
            if m:
                ts = m.group(1).lower()
                if ts.endswith("ns"):
                    sdf.timescale_ns = float(ts[:-2]) if ts[:-2] else 1.0
                elif ts.endswith("ps"):
                    sdf.timescale_ns = float(ts[:-2]) / 1000.0
                else:
                    sdf.timescale_ns = float(ts)
                continue

            # Cell start
            m = re.match(r'\(CELLTYPE\s+"([^"]+)"\)', stripped)
            if m:
                current_celltype = m.group(1)
                in_cell = True
                continue

            # Instance
            m = re.match(r'\(INSTANCE\s*(.*?)\)', stripped)
            if m and in_cell:
                current_instance = m.group(1).strip().replace('\\', '')
                if current_instance:
                    cell_count += 1
                    sdf.cells[current_instance] = SdfCellInfo(
                        celltype=current_celltype,
                        instance=current_instance,
                    )
                continue

            # IOPATH
            m = re.match(
                r'\(IOPATH\s+(\S+)\s+(\S+)\s+(\([^)]*\))\s+(\([^)]*\))\)',
                stripped,
            )
            if m and in_cell and current_instance:
                input_pin = m.group(1)
                output_pin = m.group(2)
                rise = parse_delay_triple(m.group(3), sdf.timescale_ns)
                fall = parse_delay_triple(m.group(4), sdf.timescale_ns)
                cell = sdf.cells.get(current_instance)
                if cell:
                    cell.iopaths.append(SdfIopath(
                        input_pin=input_pin,
                        output_pin=output_pin,
                        delay=SdfDelay(rise, fall),
                    ))
                continue

            # INTERCONNECT (top-level cell only, instance is empty)
            m = re.match(
                r'\(INTERCONNECT\s+(\S+)\s+(\S+)\s+(\([^)]*\))\s+(\([^)]*\))\)',
                stripped,
            )
            if m:
                source = m.group(1).replace('\\', '')
                dest = m.group(2).replace('\\', '')
                rise = parse_delay_triple(m.group(3), sdf.timescale_ns)
                fall = parse_delay_triple(m.group(4), sdf.timescale_ns)
                sdf.interconnects.append(SdfInterconnect(
                    source=source, dest=dest,
                    delay=SdfDelay(rise, fall),
                ))
                ic_count += 1
                continue

    sdf.build_indexes()
    print(f"Parsed SDF: {cell_count} cells, {ic_count} interconnects, "
          f"timescale={sdf.timescale_ns}ns", file=sys.stderr)
    return sdf


def cmd_cell_info(sdf: SdfData, instance: str) -> None:
    """Show all SDF information for a cell instance."""
    cell = sdf.cells.get(instance)
    if cell:
        print(f"Cell: {instance}  (type: {cell.celltype})")
        if cell.iopaths:
            print(f"  IOPATH entries:")
            for io in cell.iopaths:
                print(f"    {io.input_pin} → {io.output_pin}: {io.delay}")
    else:
        print(f"Cell {instance} not found in SDF")

    # Incoming interconnects
    incoming = sdf.ic_by_dest_inst.get(instance, [])
    if incoming:
        print(f"  Incoming INTERCONNECT ({len(incoming)}):")
        for ic in incoming:
            print(f"    {ic.source} → {ic.dest}: {ic.delay}")

    # Outgoing interconnects
    outgoing = sdf.ic_by_source_inst.get(instance, [])
    if outgoing:
        print(f"  Outgoing INTERCONNECT ({len(outgoing)}):")
        for ic in outgoing:
            print(f"    {ic.source} → {ic.dest}: {ic.delay}")


def cmd_trace_back(sdf: SdfData, target: str, max_depth: int) -> None:
    """Trace backwards from a destination pin through INTERCONNECT entries."""
    # target is "instance.pin" or bare port/instance name
    if '.' in target:
        inst = target[:target.rfind('.')]
        pin = target[target.rfind('.') + 1:]
    else:
        # Could be an instance name or a bare port name
        # Check if it's a known cell instance first
        if target in sdf.cells:
            inst = target
            pin = None
        else:
            # Treat as bare port — search for INTERCONNECT entries targeting it
            inst = ""
            pin = target

    print(f"Tracing backwards from: {target}")
    print()

    # Find matching INTERCONNECT entries
    _trace_back_recursive(sdf, inst, pin, 0, max_depth, 0)


def _trace_back_recursive(
    sdf: SdfData, inst: str, pin: str | None,
    depth: int, max_depth: int, cumulative_ps: int,
) -> None:
    indent = "  " * depth

    # Show cell IOPATH if we have the instance
    cell = sdf.cells.get(inst)
    if cell and pin:
        # Find IOPATH entries that OUTPUT to this pin
        for io in cell.iopaths:
            if io.output_pin == pin:
                print(f"{indent}[{cell.celltype}] {inst}")
                print(f"{indent}  IOPATH {io.input_pin} → {io.output_pin}: "
                      f"{io.delay}  (cumul: {cumulative_ps + io.delay.max_ps()}ps)")
    elif cell:
        print(f"{indent}[{cell.celltype}] {inst}")

    if depth >= max_depth:
        if depth > 0:
            print(f"{indent}  (max depth reached)")
        return

    # Find incoming INTERCONNECT entries to this instance
    incoming = sdf.ic_by_dest_inst.get(inst, [])
    if pin:
        # Filter to specific pin
        target_full = f"{inst}.{pin}" if inst else pin
        incoming = [ic for ic in incoming if ic.dest == target_full]

    if not incoming:
        if not inst:
            print(f"{indent}  (top-level port: {pin or inst})")
        return

    for ic in incoming:
        wire_ps = ic.delay.max_ps()
        new_cumul = cumulative_ps + wire_ps
        src_inst = ic.source_instance
        src_pin = ic.source_pin

        print(f"{indent}  ← wire {ic.delay} from {ic.source}  "
              f"(cumul: {new_cumul}ps)")

        if src_inst:
            # Check if source instance has an IOPATH to this pin
            src_cell = sdf.cells.get(src_inst)
            if src_cell:
                for io in src_cell.iopaths:
                    if io.output_pin == src_pin:
                        io_cumul = new_cumul + io.delay.max_ps()
                        print(f"{indent}  [IOPATH {io.input_pin}→{io.output_pin}: "
                              f"{io.delay}]  (cumul: {io_cumul}ps)")
                        # Continue tracing from the input pin
                        _trace_back_recursive(
                            sdf, src_inst, io.input_pin,
                            depth + 1, max_depth, io_cumul,
                        )
                        break
                else:
                    # No IOPATH found — might be a register output
                    print(f"{indent}  [{src_cell.celltype}] {src_inst} (no matching IOPATH)")
        else:
            # Source is a port (no instance)
            print(f"{indent}  (input port: {ic.source})")


def cmd_trace_fwd(sdf: SdfData, source: str, max_depth: int) -> None:
    """Trace forwards from a source pin through INTERCONNECT entries."""
    if '.' in source:
        inst = source[:source.rfind('.')]
        pin = source[source.rfind('.') + 1:]
    else:
        inst = source
        pin = None

    print(f"Tracing forwards from: {source}")
    print()
    _trace_fwd_recursive(sdf, inst, pin, 0, max_depth, 0)


def _trace_fwd_recursive(
    sdf: SdfData, inst: str, pin: str | None,
    depth: int, max_depth: int, cumulative_ps: int,
) -> None:
    indent = "  " * depth

    if depth >= max_depth:
        print(f"{indent}(max depth reached)")
        return

    # Find outgoing INTERCONNECT from this instance
    outgoing = sdf.ic_by_source_inst.get(inst, [])
    if pin:
        source_full = f"{inst}.{pin}"
        outgoing = [ic for ic in outgoing if ic.source == source_full]

    if not outgoing:
        print(f"{indent}(no outgoing interconnects)")
        return

    for ic in outgoing:
        wire_ps = ic.delay.max_ps()
        new_cumul = cumulative_ps + wire_ps
        dest_inst = ic.dest_instance
        dest_pin = ic.dest_pin

        print(f"{indent}→ wire {ic.delay} to {ic.dest}  (cumul: {new_cumul}ps)")

        dest_cell = sdf.cells.get(dest_inst)
        if dest_cell:
            # Show what this cell does with the input
            for io in dest_cell.iopaths:
                if io.input_pin == dest_pin:
                    io_cumul = new_cumul + io.delay.max_ps()
                    print(f"{indent}  [{dest_cell.celltype}] {dest_inst}")
                    print(f"{indent}  IOPATH {io.input_pin}→{io.output_pin}: "
                          f"{io.delay}  (cumul: {io_cumul}ps)")
                    if depth + 1 < max_depth:
                        _trace_fwd_recursive(
                            sdf, dest_inst, io.output_pin,
                            depth + 1, max_depth, io_cumul,
                        )
                    break
            else:
                print(f"{indent}  [{dest_cell.celltype}] {dest_inst} "
                      f"(pin {dest_pin}, no matching IOPATH)")


def cmd_clock_tree(sdf: SdfData, dff_instance: str) -> None:
    """Trace the clock tree path from pad to a specific DFF."""
    cell = sdf.cells.get(dff_instance)
    if not cell:
        print(f"ERROR: Instance {dff_instance} not found in SDF")
        sys.exit(1)

    print(f"Clock tree to DFF: {dff_instance} ({cell.celltype})")
    print()

    # Find CLK IOPATH
    clk_iopath = None
    for io in cell.iopaths:
        if io.input_pin == "CLK":
            clk_iopath = io
            break

    if clk_iopath:
        print(f"  DFF IOPATH CLK → {clk_iopath.output_pin}: {clk_iopath.delay}")
    else:
        print("  (no CLK IOPATH found)")

    print()
    print("Clock path (traced backwards from DFF CLK pin):")
    print(f"  {'Stage':<45} {'Delay':>12} {'Cumulative':>12}")
    print(f"  {'-'*45} {'-'*12} {'-'*12}")

    # Trace backwards from DFF CLK
    path: list[tuple[str, str, SdfDelay]] = []  # (description, type, delay)
    current_inst = dff_instance
    current_pin = "CLK"
    total_ps = 0

    for _ in range(20):  # max depth safety
        # Find INTERCONNECT to current_inst.current_pin
        incoming = sdf.ic_by_dest_inst.get(current_inst, [])
        target = f"{current_inst}.{current_pin}"
        matching = [ic for ic in incoming if ic.dest == target]

        if not matching:
            # Try without pin (direct port connection)
            if not current_inst:
                break
            # No interconnect found
            break

        ic = matching[0]  # Take first (should be unique for clock)
        wire_delay = ic.delay

        # Add wire to path
        total_ps += wire_delay.max_ps()
        path.append((
            f"wire: {ic.source} → {ic.dest}",
            "wire",
            wire_delay,
        ))
        print(f"  wire: {ic.source:<39} {wire_delay!s:>12} {total_ps:>11}ps")

        src_inst = ic.source_instance
        src_pin = ic.source_pin

        if not src_inst:
            # Reached an input port
            print(f"  PORT: {ic.source:<39} {'':>12} {total_ps:>11}ps")
            break

        # Find IOPATH in source cell
        src_cell = sdf.cells.get(src_inst)
        if src_cell:
            for io in src_cell.iopaths:
                if io.output_pin == src_pin:
                    total_ps += io.delay.max_ps()
                    path.append((
                        f"[{src_cell.celltype}] {src_inst}",
                        "cell",
                        io.delay,
                    ))
                    print(f"  [{src_cell.celltype}] {src_inst:<25} "
                          f"{io.delay!s:>12} {total_ps:>11}ps")
                    current_pin = io.input_pin
                    current_inst = src_inst
                    break
            else:
                print(f"  [{src_cell.celltype}] {src_inst} (no matching IOPATH)")
                break
        else:
            print(f"  {src_inst} (not found in SDF)")
            break

    print()
    print(f"  Total clock insertion delay: {total_ps:,}ps")
    if clk_iopath:
        total_with_clk2q = total_ps + clk_iopath.delay.max_ps()
        print(f"  + DFF clk-to-Q ({clk_iopath.delay}): {total_with_clk2q:,}ps")
    print(f"  Clock tree depth: {len([p for p in path if p[1] == 'cell'])} buffers")


def cmd_output_path(
    sdf: SdfData, dff_instance: str, output_port: str,
) -> None:
    """Trace from DFF Q output to an output port."""
    cell = sdf.cells.get(dff_instance)
    if not cell:
        print(f"ERROR: Instance {dff_instance} not found in SDF")
        sys.exit(1)

    # Find CLK→Q IOPATH
    clk_iopath = None
    for io in cell.iopaths:
        if io.input_pin == "CLK" and io.output_pin == "Q":
            clk_iopath = io
            break

    print(f"Output path: {dff_instance}.Q → {output_port}")
    print()
    if clk_iopath:
        print(f"  DFF clk-to-Q: {clk_iopath.delay}")
    print()
    print("Forward path from DFF Q:")
    cmd_trace_fwd(sdf, f"{dff_instance}.Q", max_depth=10)


def main() -> None:
    args = sys.argv[1:]
    if len(args) < 2:
        print(__doc__, file=sys.stderr)
        sys.exit(1)

    sdf_path = Path(args[0])
    command = args[1]

    if not sdf_path.exists():
        print(f"ERROR: SDF file not found: {sdf_path}", file=sys.stderr)
        sys.exit(1)

    sdf = parse_sdf(sdf_path)

    if command == "clock-tree" and len(args) >= 3:
        cmd_clock_tree(sdf, args[2])
    elif command == "trace-back" and len(args) >= 3:
        depth = 5
        if "-d" in args:
            idx = args.index("-d")
            if idx + 1 < len(args):
                depth = int(args[idx + 1])
        cmd_trace_back(sdf, args[2], depth)
    elif command == "trace-fwd" and len(args) >= 3:
        depth = 5
        if "-d" in args:
            idx = args.index("-d")
            if idx + 1 < len(args):
                depth = int(args[idx + 1])
        cmd_trace_fwd(sdf, args[2], depth)
    elif command == "cell-info" and len(args) >= 3:
        cmd_cell_info(sdf, args[2])
    elif command == "output-path" and len(args) >= 4:
        cmd_output_path(sdf, args[2], args[3])
    else:
        print(__doc__, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
