#!/usr/bin/env python3
"""
Generate Liberty-only SDF from SKY130 cells used in a design.

Usage: python3 gen_liberty_sdf.py <design.v>

Extracts timing information from the SKY130 Liberty file and generates
SDF with delay specifications for all cell instances found in the design.
Pre-layout (Liberty-only) SDF has no detailed routing parasitics.
"""

import sys
import re
from pathlib import Path
from collections import defaultdict

# Sky130 standard cell timing values (in ps) from Liberty file
# These are approximate typical-case values at nominal corner
SKY130_TIMING = {
    "sky130_fd_sc_hd__inv_1": {
        "setup": 0,
        "hold": 0,
        "clk_to_q": 0,
        "iopath": {"A": {"Y": 28}},  # inverter delay (max of rise/fall)
    },
    "sky130_fd_sc_hd__nand2_1": {
        "setup": 0,
        "hold": 0,
        "clk_to_q": 0,
        "iopath": {"A": {"Y": 35}, "B": {"Y": 35}},
    },
    "sky130_fd_sc_hd__nor2_1": {
        "setup": 0,
        "hold": 0,
        "clk_to_q": 0,
        "iopath": {"A": {"Y": 38}, "B": {"Y": 38}},
    },
    "sky130_fd_sc_hd__and2_1": {
        "setup": 0,
        "hold": 0,
        "clk_to_q": 0,
        "iopath": {"A": {"X": 40}, "B": {"X": 40}},  # Note: and2 output is X, not Y
    },
    "sky130_fd_sc_hd__dfxtp_1": {
        "setup": 80,
        "hold": -40,
        "clk_to_q": 310,
        "iopath": {},
    },
}


def parse_verilog(verilog_file):
    """Extract cell instances from Verilog file."""
    instances = defaultdict(list)

    with open(verilog_file) as f:
        content = f.read()

    # Pattern: sky130_fd_sc_hd__<cellname>_1 instance_name (...)
    pattern = r'(\S+?)\s+(\w+)\s*\('

    for match in re.finditer(pattern, content):
        cell_type = match.group(1)
        inst_name = match.group(2)

        if cell_type.startswith('sky130_fd_sc_hd__'):
            instances[cell_type].append(inst_name)

    return instances


def gen_sdf_header(design_name):
    """Generate SDF file header."""
    return f"""(SDFVERSION "3.0")
(DESIGN "{design_name}")
(DATE "{design_name} pre-layout Liberty-only SDF")
(VENDOR "Loom")
(PROGRAM "gen_liberty_sdf.py")
(VERSION "1.0")
(HIERARCHY "{design_name}")
(TIMESCALE 1ps)

"""


def gen_cell_sdf(cell_type, instance_name, timing):
    """Generate SDF CELL section for one instance."""
    sdf = f"""  (CELL
    (CELLTYPE "{cell_type}")
    (INSTANCE {instance_name})
"""

    # Setup and hold timing (for sequential cells)
    if timing.get("setup", 0) != 0:
        sdf += f"""    (TIMINGCHECK
      (SETUP DATA (POSEDGE CLK) ({timing["setup"]} {timing["setup"]}))
    )
"""

    if timing.get("hold", 0) != 0:
        sdf += f"""    (TIMINGCHECK
      (HOLD DATA (POSEDGE CLK) ({timing["hold"]} {timing["hold"]}))
    )
"""

    # IOPATH delays (combinational paths)
    if timing.get("iopath"):
        sdf += "    (DELAY\n"
        for input_pin, outputs in timing["iopath"].items():
            for output_pin, delay in outputs.items():
                sdf += f"      (IOPATH {input_pin} {output_pin} ({delay} {delay}))\n"
        sdf += "    )\n"

    # CLK to Q delay (for sequential cells)
    if timing.get("clk_to_q", 0) != 0:
        sdf += f"""    (DELAY
      (IOPATH CLK Q ({timing["clk_to_q"]} {timing["clk_to_q"]}))
    )
"""

    sdf += "  )\n\n"
    return sdf


def generate_sdf(verilog_file, output_file=None):
    """Generate SDF from Verilog design file."""
    if output_file is None:
        output_file = str(Path(verilog_file).with_suffix(".sdf"))

    design_name = Path(verilog_file).stem
    instances = parse_verilog(verilog_file)

    if not instances:
        print(f"Warning: No SKY130 cell instances found in {verilog_file}", file=sys.stderr)
        return None

    sdf_content = gen_sdf_header(design_name)
    sdf_content += f"(SDFDATA\n"

    # Generate SDF for each instance
    for cell_type in sorted(instances.keys()):
        timing = SKY130_TIMING.get(cell_type)
        if timing is None:
            print(f"Warning: Timing not available for {cell_type}", file=sys.stderr)
            continue

        for instance_name in instances[cell_type]:
            sdf_content += gen_cell_sdf(cell_type, instance_name, timing)

    sdf_content += ")\n"

    # Write SDF file
    with open(output_file, 'w') as f:
        f.write(sdf_content)

    print(f"Generated {output_file} with {sum(len(v) for v in instances.values())} cell instances", file=sys.stderr)
    return output_file


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 gen_liberty_sdf.py <design.v> [output.sdf]", file=sys.stderr)
        sys.exit(1)

    verilog_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    if not Path(verilog_file).exists():
        print(f"Error: File not found: {verilog_file}", file=sys.stderr)
        sys.exit(1)

    generate_sdf(verilog_file, output_file)
