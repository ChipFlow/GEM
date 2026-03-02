"""Jacquard configuration management — mirrors the Rust JacquardConfig / jacquard.toml."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from .errors import ConfigError


@dataclass
class SdfConfig:
    """SDF timing back-annotation configuration."""

    file: Path | None = None
    corner: str = "typ"
    debug: bool = False


@dataclass
class TimingConfig:
    """Post-simulation timing analysis configuration."""

    enabled: bool = False
    clock_period: int | None = None  # picoseconds
    report_violations: bool = False


@dataclass
class DesignConfig:
    """Shared design parameters used across subcommands."""

    netlist: Path | None = None
    top_module: str | None = None
    liberty: Path | None = None


@dataclass
class MapConfig:
    """Partition mapping settings for `jacquard map`."""

    output: Path | None = None
    level_split: list[int] = field(default_factory=list)
    max_stage_degrad: int = 0
    xprop: bool = False


@dataclass
class SimConfig:
    """Simulation settings for `jacquard sim`."""

    gemparts: Path | None = None
    input_vcd: Path | None = None
    output_vcd: Path | None = None
    num_blocks: int | None = None
    input_vcd_scope: str | None = None
    output_vcd_scope: str | None = None
    max_cycles: int | None = None
    check_with_cpu: bool = False
    xprop: bool = False
    dump_signals: list[str] = field(default_factory=list)
    dump_scope: str | None = None
    dump_depth: int | None = None
    plusargs: dict[str, str] = field(default_factory=dict)
    sdf: SdfConfig | None = None
    timing: TimingConfig | None = None


@dataclass
class JacquardConfig:
    """Top-level project configuration, mirrors `jacquard.toml`."""

    design: DesignConfig = field(default_factory=DesignConfig)
    map: MapConfig = field(default_factory=MapConfig)
    sim: SimConfig = field(default_factory=SimConfig)

    @classmethod
    def from_toml(cls, path: Path) -> JacquardConfig:
        """Load configuration from a TOML file.

        Relative paths are resolved against the config file's parent directory.
        """
        path = Path(path)
        if not path.exists():
            raise ConfigError(f"Config file not found: {path}")
        try:
            with open(path, "rb") as f:
                raw = tomllib.load(f)
        except tomllib.TOMLDecodeError as e:
            raise ConfigError(f"Failed to parse {path}: {e}") from e

        config_dir = path.parent
        config = cls._from_dict(raw)
        config._resolve_paths(config_dir)
        return config

    @classmethod
    def _from_dict(cls, d: dict) -> JacquardConfig:
        """Build config from a parsed TOML dict."""
        design_d = d.get("design", {})
        map_d = d.get("map", {})
        sim_d = d.get("sim", {})

        design = DesignConfig(
            netlist=Path(design_d["netlist"]) if "netlist" in design_d else None,
            top_module=design_d.get("top_module"),
            liberty=Path(design_d["liberty"]) if "liberty" in design_d else None,
        )

        map_cfg = MapConfig(
            output=Path(map_d["output"]) if "output" in map_d else None,
            level_split=map_d.get("level_split", []),
            max_stage_degrad=map_d.get("max_stage_degrad", 0),
            xprop=map_d.get("xprop", False),
        )

        # Parse SDF sub-config
        sdf_cfg = None
        if "sdf" in sim_d:
            sdf_d = sim_d["sdf"]
            sdf_cfg = SdfConfig(
                file=Path(sdf_d["file"]) if "file" in sdf_d else None,
                corner=sdf_d.get("corner", "typ"),
                debug=sdf_d.get("debug", False),
            )

        # Parse timing sub-config
        timing_cfg = None
        if "timing" in sim_d:
            timing_d = sim_d["timing"]
            timing_cfg = TimingConfig(
                enabled=timing_d.get("enabled", False),
                clock_period=timing_d.get("clock_period"),
                report_violations=timing_d.get("report_violations", False),
            )

        sim_cfg = SimConfig(
            gemparts=Path(sim_d["gemparts"]) if "gemparts" in sim_d else None,
            input_vcd=Path(sim_d["input_vcd"]) if "input_vcd" in sim_d else None,
            output_vcd=Path(sim_d["output_vcd"]) if "output_vcd" in sim_d else None,
            num_blocks=sim_d.get("num_blocks"),
            input_vcd_scope=sim_d.get("input_vcd_scope"),
            output_vcd_scope=sim_d.get("output_vcd_scope"),
            max_cycles=sim_d.get("max_cycles"),
            check_with_cpu=sim_d.get("check_with_cpu", False),
            xprop=sim_d.get("xprop", False),
            dump_signals=sim_d.get("dump_signals", []),
            dump_scope=sim_d.get("dump_scope"),
            dump_depth=sim_d.get("dump_depth"),
            plusargs=sim_d.get("plusargs", {}),
            sdf=sdf_cfg,
            timing=timing_cfg,
        )

        return cls(design=design, map=map_cfg, sim=sim_cfg)

    def to_toml(self, path: Path) -> None:
        """Serialize configuration to a TOML file."""
        lines: list[str] = []

        # [design]
        lines.append("[design]")
        if self.design.netlist is not None:
            lines.append(f'netlist = "{self.design.netlist}"')
        if self.design.top_module is not None:
            lines.append(f'top_module = "{self.design.top_module}"')
        if self.design.liberty is not None:
            lines.append(f'liberty = "{self.design.liberty}"')

        # [map]
        lines.append("")
        lines.append("[map]")
        if self.map.output is not None:
            lines.append(f'output = "{self.map.output}"')
        if self.map.level_split:
            lines.append(f"level_split = {self.map.level_split}")
        if self.map.max_stage_degrad != 0:
            lines.append(f"max_stage_degrad = {self.map.max_stage_degrad}")
        if self.map.xprop:
            lines.append("xprop = true")

        # [sim]
        lines.append("")
        lines.append("[sim]")
        if self.sim.gemparts is not None:
            lines.append(f'gemparts = "{self.sim.gemparts}"')
        if self.sim.input_vcd is not None:
            lines.append(f'input_vcd = "{self.sim.input_vcd}"')
        if self.sim.output_vcd is not None:
            lines.append(f'output_vcd = "{self.sim.output_vcd}"')
        if self.sim.num_blocks is not None:
            lines.append(f"num_blocks = {self.sim.num_blocks}")
        if self.sim.input_vcd_scope is not None:
            lines.append(f'input_vcd_scope = "{self.sim.input_vcd_scope}"')
        if self.sim.output_vcd_scope is not None:
            lines.append(f'output_vcd_scope = "{self.sim.output_vcd_scope}"')
        if self.sim.max_cycles is not None:
            lines.append(f"max_cycles = {self.sim.max_cycles}")
        if self.sim.check_with_cpu:
            lines.append("check_with_cpu = true")
        if self.sim.xprop:
            lines.append("xprop = true")
        if self.sim.dump_signals:
            items = ", ".join(f'"{s}"' for s in self.sim.dump_signals)
            lines.append(f"dump_signals = [{items}]")
        if self.sim.dump_scope is not None:
            lines.append(f'dump_scope = "{self.sim.dump_scope}"')
        if self.sim.dump_depth is not None:
            lines.append(f"dump_depth = {self.sim.dump_depth}")
        if self.sim.plusargs:
            lines.append("")
            lines.append("[sim.plusargs]")
            for k, v in sorted(self.sim.plusargs.items()):
                # Quote keys that contain dots (TOML requirement)
                key = f'"{k}"' if "." in k else k
                lines.append(f'{key} = "{v}"')

        # [sim.sdf]
        if self.sim.sdf is not None:
            lines.append("")
            lines.append("[sim.sdf]")
            if self.sim.sdf.file is not None:
                lines.append(f'file = "{self.sim.sdf.file}"')
            if self.sim.sdf.corner != "typ":
                lines.append(f'corner = "{self.sim.sdf.corner}"')
            if self.sim.sdf.debug:
                lines.append("debug = true")

        # [sim.timing]
        if self.sim.timing is not None:
            lines.append("")
            lines.append("[sim.timing]")
            if self.sim.timing.enabled:
                lines.append("enabled = true")
            if self.sim.timing.clock_period is not None:
                lines.append(f"clock_period = {self.sim.timing.clock_period}")
            if self.sim.timing.report_violations:
                lines.append("report_violations = true")

        lines.append("")  # trailing newline
        Path(path).write_text("\n".join(lines))

    def effective_gemparts(self) -> Path | None:
        """Get effective gemparts path, falling back to map.output."""
        return self.sim.gemparts or self.map.output

    def _resolve_paths(self, config_dir: Path) -> None:
        """Resolve relative paths against the config file's directory."""
        self.design.netlist = _resolve(self.design.netlist, config_dir)
        self.design.liberty = _resolve(self.design.liberty, config_dir)
        self.map.output = _resolve(self.map.output, config_dir)
        self.sim.gemparts = _resolve(self.sim.gemparts, config_dir)
        self.sim.input_vcd = _resolve(self.sim.input_vcd, config_dir)
        self.sim.output_vcd = _resolve(self.sim.output_vcd, config_dir)
        if self.sim.sdf is not None:
            self.sim.sdf.file = _resolve(self.sim.sdf.file, config_dir)


def _resolve(path: Path | None, base: Path) -> Path | None:
    """Resolve a relative path against a base directory. Absolute paths unchanged."""
    if path is None:
        return None
    if path.is_absolute():
        return path
    return base / path
