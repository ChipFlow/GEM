#!/usr/bin/env python3
"""Generate sim_config.json for gpu_sim from a ChipFlow pins.lock file.

Supports both SKY130 openframe and IHP pin formats.

Usage:
    python scripts/gen_sim_config.py <pins_lock> <output_dir> [options]

Examples:
    # Generate from SKY130 openframe pins.lock
    python scripts/gen_sim_config.py tests/timing_test/minimal_build/api_artifacts/config.json \
        tests/timing_test/ --netlist tests/timing_test/minimal_build/6_final.v \
        --firmware tests/timing_test/software.bin

    # Generate from chipflow-examples mcu_soc
    python scripts/gen_sim_config.py chipflow-examples/mcu_soc/pins.lock \
        tests/mcu_soc/ --firmware chipflow-examples/mcu_soc/build/software/software.bin
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
log = logging.getLogger(__name__)


def extract_pin_index(pin_entry, process: str) -> int:
    """Extract the GPIO/pad index from a pin entry.

    SKY130 openframe: [pad_number, "gpio", gpio_index, null, null] -> gpio_index
    IHP:              integer pad number -> pad number
    """
    if isinstance(pin_entry, list):
        # SKY130 openframe format: [pad, "gpio", gpio_index, ...]
        assert len(pin_entry) >= 3, f"Unexpected pin tuple format: {pin_entry}"
        return pin_entry[2]
    elif isinstance(pin_entry, int):
        # IHP format: direct pad/pin number
        return pin_entry
    else:
        raise ValueError(f"Unknown pin format: {pin_entry} (type={type(pin_entry)})")


def gen_sim_config(
    pins_lock: dict,
    *,
    netlist_path: str | None = None,
    liberty_path: str | None = None,
    firmware_path: str | None = None,
    firmware_offset: int = 1048576,
    clock_period_ps: int = 40000,
    num_cycles: int = 100000,
    reset_cycles: int = 10,
    output_events: str | None = None,
    events_reference: str | None = None,
) -> dict:
    """Generate sim_config.json content from pins.lock data."""
    process = pins_lock.get("process", "unknown")
    ports = pins_lock["port_map"]["ports"]
    soc = ports["soc"]
    core = ports["_core"]["bringup_pins"]

    # Clock
    clk_pins = core["clk"]["pins"]
    assert len(clk_pins) == 1, f"Expected 1 clock pin, got {len(clk_pins)}"
    clock_gpio = extract_pin_index(clk_pins[0], process)

    # Reset
    rst_pins = core["rst_n"]["pins"]
    assert len(rst_pins) == 1, f"Expected 1 reset pin, got {len(rst_pins)}"
    reset_gpio = extract_pin_index(rst_pins[0], process)
    rst_invert = core["rst_n"]["iomodel"].get("invert", False)
    # invert=True in pins.lock means the reset signal is active low (_n suffix)
    reset_active_high = not rst_invert

    # Flash
    flash_config = None
    if "flash" in soc:
        flash = soc["flash"]
        flash_clk = extract_pin_index(flash["clk"]["pins"][0], process)
        flash_csn = extract_pin_index(flash["csn"]["pins"][0], process)
        # d0 is the first data pin (MOSI/MISO in SPI mode)
        flash_d0 = extract_pin_index(flash["d"]["pins"][0], process)

        flash_config = {
            "clk_gpio": flash_clk,
            "csn_gpio": flash_csn,
            "d0_gpio": flash_d0,
            "firmware": firmware_path or "software.bin",
            "firmware_offset": firmware_offset,
        }

    # UART (find first uart peripheral)
    uart_config = None
    for key in sorted(soc.keys()):
        if key.startswith("uart_"):
            uart = soc[key]
            uart_tx = extract_pin_index(uart["tx"]["pins"][0], process)
            uart_rx = extract_pin_index(uart["rx"]["pins"][0], process)
            uart_config = {
                "tx_gpio": uart_tx,
                "rx_gpio": uart_rx,
                "baud_rate": 115200,
            }
            log.info(f"UART: {key} tx=gpio {uart_tx}, rx=gpio {uart_rx}")
            break

    # Build config
    config: dict = {
        "clock_gpio": clock_gpio,
        "reset_gpio": reset_gpio,
        "reset_active_high": reset_active_high,
        "reset_cycles": reset_cycles,
        "num_cycles": num_cycles,
        "clock_period_ps": clock_period_ps,
    }

    if netlist_path:
        config["netlist_path"] = netlist_path

    if liberty_path:
        config["liberty_path"] = liberty_path

    if flash_config:
        config["flash"] = flash_config

    if uart_config:
        config["uart"] = uart_config

    if output_events:
        config["output_events"] = output_events

    if events_reference:
        config["events_reference"] = events_reference

    log.info(f"Process: {process}")
    log.info(f"Clock: gpio {clock_gpio}")
    log.info(f"Reset: gpio {reset_gpio} (active_high={reset_active_high})")
    if flash_config:
        log.info(f"Flash: clk=gpio {flash_config['clk_gpio']}, "
                 f"csn=gpio {flash_config['csn_gpio']}, "
                 f"d0=gpio {flash_config['d0_gpio']}")

    return config


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate sim_config.json from ChipFlow pins.lock")
    parser.add_argument("pins_lock", type=Path,
                        help="Path to pins.lock (or config.json)")
    parser.add_argument("output_dir", type=Path,
                        help="Output directory for sim_config.json")
    parser.add_argument("--netlist", type=str, default=None,
                        help="Path to gate-level netlist (6_final.v)")
    parser.add_argument("--liberty", type=str, default=None,
                        help="Path to liberty timing file (.lib)")
    parser.add_argument("--firmware", type=str, default=None,
                        help="Path to firmware binary (software.bin)")
    parser.add_argument("--firmware-offset", type=int, default=1048576,
                        help="Firmware offset in flash (default: 1048576 = 1MB)")
    parser.add_argument("--clock-period-ps", type=int, default=40000,
                        help="Clock period in picoseconds (default: 40000 = 25MHz)")
    parser.add_argument("--num-cycles", type=int, default=100000,
                        help="Number of simulation cycles (default: 100000)")
    parser.add_argument("--output-events", type=str, default=None,
                        help="Path to write output events JSON")
    parser.add_argument("--events-reference", type=str, default=None,
                        help="Path to reference events JSON for verification")

    args = parser.parse_args()

    with open(args.pins_lock) as f:
        pins_lock = json.load(f)

    config = gen_sim_config(
        pins_lock,
        netlist_path=args.netlist,
        liberty_path=args.liberty,
        firmware_path=args.firmware,
        firmware_offset=args.firmware_offset,
        clock_period_ps=args.clock_period_ps,
        num_cycles=args.num_cycles,
        output_events=args.output_events,
        events_reference=args.events_reference,
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_path = args.output_dir / "sim_config.json"
    with open(output_path, "w") as f:
        json.dump(config, f, indent=4)
        f.write("\n")

    log.info(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
