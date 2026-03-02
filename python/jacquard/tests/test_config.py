"""Tests for JacquardConfig TOML I/O and field handling."""

from pathlib import Path

import pytest

from jacquard.config import JacquardConfig, SdfConfig, TimingConfig
from jacquard.errors import ConfigError


@pytest.fixture
def tmp_toml(tmp_path):
    """Helper to write TOML content and return path."""
    def _write(content: str) -> Path:
        p = tmp_path / "jacquard.toml"
        p.write_text(content)
        return p
    return _write


class TestFromToml:
    def test_empty_config(self, tmp_toml):
        config = JacquardConfig.from_toml(tmp_toml(""))
        assert config.design.netlist is None
        assert config.map.level_split == []
        assert config.sim.num_blocks is None

    def test_minimal_config(self, tmp_toml):
        config = JacquardConfig.from_toml(tmp_toml("""
[design]
netlist = "gatelevel.gv"

[map]
output = "result.gemparts"

[sim]
input_vcd = "input.vcd"
output_vcd = "output.vcd"
"""))
        # Paths are resolved against tmp_path
        assert config.design.netlist is not None
        assert config.design.netlist.name == "gatelevel.gv"
        assert config.map.output is not None
        assert config.map.output.name == "result.gemparts"

    def test_full_config(self, tmp_toml):
        config = JacquardConfig.from_toml(tmp_toml("""
[design]
netlist = "build/gatelevel.gv"
top_module = "my_top"
liberty = "/opt/pdk/lib.lib"

[map]
output = "build/result.gemparts"
level_split = [20, 40]
max_stage_degrad = 1
xprop = true

[sim]
input_vcd = "test/input.vcd"
output_vcd = "test/output.vcd"
num_blocks = 128
max_cycles = 1000
check_with_cpu = true
xprop = true
dump_signals = ["*clk*", "data_out*"]
dump_scope = "cpu/core0"
dump_depth = 2

[sim.plusargs]
run_id = "nightly_042"
seed = "12345"
"force.reset" = "0"

[sim.sdf]
file = "build/design.sdf"
corner = "typ"
debug = false

[sim.timing]
enabled = true
clock_period = 1000
report_violations = true
"""))
        assert config.map.level_split == [20, 40]
        assert config.sim.num_blocks == 128
        assert config.sim.dump_signals == ["*clk*", "data_out*"]
        assert config.sim.dump_scope == "cpu/core0"
        assert config.sim.dump_depth == 2
        assert config.sim.plusargs["run_id"] == "nightly_042"
        assert config.sim.plusargs["force.reset"] == "0"
        assert config.sim.sdf is not None
        assert config.sim.sdf.corner == "typ"
        assert config.sim.timing is not None
        assert config.sim.timing.clock_period == 1000

    def test_path_resolution_relative(self, tmp_toml):
        config = JacquardConfig.from_toml(tmp_toml("""
[design]
netlist = "build/gatelevel.gv"

[map]
output = "build/result.gemparts"
"""))
        # Relative paths resolved against config dir (tmp_path)
        assert config.design.netlist is not None
        assert config.design.netlist.is_absolute()
        assert "build" in str(config.design.netlist)

    def test_path_resolution_absolute(self, tmp_toml):
        config = JacquardConfig.from_toml(tmp_toml("""
[design]
netlist = "/absolute/path/gatelevel.gv"
liberty = "/opt/pdk/lib.lib"
"""))
        assert config.design.netlist == Path("/absolute/path/gatelevel.gv")
        assert config.design.liberty == Path("/opt/pdk/lib.lib")

    def test_missing_file(self, tmp_path):
        with pytest.raises(ConfigError, match="not found"):
            JacquardConfig.from_toml(tmp_path / "nonexistent.toml")

    def test_invalid_toml(self, tmp_toml):
        with pytest.raises(ConfigError, match="Failed to parse"):
            JacquardConfig.from_toml(tmp_toml("this is not [valid toml"))

    def test_effective_gemparts_direct(self, tmp_toml):
        config = JacquardConfig.from_toml(tmp_toml("""
[sim]
gemparts = "sim.gemparts"
"""))
        assert config.effective_gemparts() is not None
        assert config.effective_gemparts().name == "sim.gemparts"  # type: ignore[union-attr]

    def test_effective_gemparts_fallback(self, tmp_toml):
        config = JacquardConfig.from_toml(tmp_toml("""
[map]
output = "map.gemparts"
"""))
        assert config.effective_gemparts() is not None
        assert config.effective_gemparts().name == "map.gemparts"  # type: ignore[union-attr]

    def test_plusargs_default_empty(self, tmp_toml):
        config = JacquardConfig.from_toml(tmp_toml(""))
        assert config.sim.plusargs == {}

    def test_dump_signals_default_empty(self, tmp_toml):
        config = JacquardConfig.from_toml(tmp_toml(""))
        assert config.sim.dump_signals == []
        assert config.sim.dump_scope is None
        assert config.sim.dump_depth is None


class TestRoundTrip:
    def test_minimal_round_trip(self, tmp_path):
        original = JacquardConfig()
        original.design.netlist = Path("gatelevel.gv")
        original.map.output = Path("result.gemparts")
        original.sim.input_vcd = Path("input.vcd")
        original.sim.output_vcd = Path("output.vcd")

        toml_path = tmp_path / "jacquard.toml"
        original.to_toml(toml_path)

        loaded = JacquardConfig.from_toml(toml_path)
        assert loaded.design.netlist is not None
        assert loaded.design.netlist.name == "gatelevel.gv"
        assert loaded.map.output is not None
        assert loaded.map.output.name == "result.gemparts"

    def test_full_round_trip(self, tmp_path):
        original = JacquardConfig()
        original.design.netlist = Path("build/design.gv")
        original.design.top_module = "top"
        original.map.output = Path("build/result.gemparts")
        original.map.level_split = [20, 40]
        original.map.max_stage_degrad = 1
        original.map.xprop = True
        original.sim.input_vcd = Path("test/input.vcd")
        original.sim.output_vcd = Path("test/output.vcd")
        original.sim.num_blocks = 64
        original.sim.max_cycles = 5000
        original.sim.xprop = True
        original.sim.check_with_cpu = True
        original.sim.dump_signals = ["*clk*", "data*"]
        original.sim.dump_scope = "cpu"
        original.sim.dump_depth = 3
        original.sim.plusargs = {"run_id": "test1", "force.reset": "0"}
        original.sim.sdf = SdfConfig(file=Path("design.sdf"), corner="max", debug=True)
        original.sim.timing = TimingConfig(enabled=True, clock_period=2000, report_violations=True)

        toml_path = tmp_path / "jacquard.toml"
        original.to_toml(toml_path)

        loaded = JacquardConfig.from_toml(toml_path)

        assert loaded.design.top_module == "top"
        assert loaded.map.level_split == [20, 40]
        assert loaded.map.max_stage_degrad == 1
        assert loaded.map.xprop is True
        assert loaded.sim.num_blocks == 64
        assert loaded.sim.max_cycles == 5000
        assert loaded.sim.xprop is True
        assert loaded.sim.check_with_cpu is True
        assert loaded.sim.dump_signals == ["*clk*", "data*"]
        assert loaded.sim.dump_scope == "cpu"
        assert loaded.sim.dump_depth == 3
        assert loaded.sim.plusargs == {"run_id": "test1", "force.reset": "0"}
        assert loaded.sim.sdf is not None
        assert loaded.sim.sdf.corner == "max"
        assert loaded.sim.sdf.debug is True
        assert loaded.sim.timing is not None
        assert loaded.sim.timing.enabled is True
        assert loaded.sim.timing.clock_period == 2000
        assert loaded.sim.timing.report_violations is True
