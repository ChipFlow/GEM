"""Tests for CLI argument building and result parsing."""

from pathlib import Path
from unittest.mock import patch

import pytest

from jacquard.config import (
    DesignConfig,
    JacquardConfig,
    MapConfig,
    SdfConfig,
    SimConfig,
    TimingConfig,
)
from jacquard.errors import BinaryNotFoundError
from jacquard.result import parse_design_stats, parse_sim_summary
from jacquard.runner import (
    _apply_overrides,
    _build_map_args,
    _build_sim_args,
    find_jacquard_binary,
)


class TestFindBinary:
    def test_explicit_path(self, tmp_path):
        binary = tmp_path / "jacquard"
        binary.touch()
        result = find_jacquard_binary(explicit_path=binary)
        assert result == binary

    def test_explicit_path_missing(self, tmp_path):
        with pytest.raises(BinaryNotFoundError):
            find_jacquard_binary(explicit_path=tmp_path / "nonexistent")

    def test_cargo_target_release(self, tmp_path):
        release_dir = tmp_path / "release"
        release_dir.mkdir()
        binary = release_dir / "jacquard"
        binary.touch()
        result = find_jacquard_binary(cargo_target_dir=tmp_path)
        assert result == binary

    def test_cargo_target_debug_fallback(self, tmp_path):
        debug_dir = tmp_path / "debug"
        debug_dir.mkdir()
        binary = debug_dir / "jacquard"
        binary.touch()
        result = find_jacquard_binary(cargo_target_dir=tmp_path)
        assert result == binary

    def test_path_lookup(self):
        with patch("jacquard.runner.shutil.which", return_value="/usr/local/bin/jacquard"):
            result = find_jacquard_binary()
            assert result == Path("/usr/local/bin/jacquard")

    def test_not_found(self):
        with patch("jacquard.runner.shutil.which", return_value=None):
            with pytest.raises(BinaryNotFoundError):
                find_jacquard_binary()


class TestBuildMapArgs:
    def test_minimal(self):
        config = JacquardConfig(
            design=DesignConfig(netlist=Path("design.gv")),
            map=MapConfig(output=Path("result.gemparts")),
        )
        args = _build_map_args(config)
        assert args == ["map", "design.gv", "result.gemparts"]

    def test_with_options(self):
        config = JacquardConfig(
            design=DesignConfig(netlist=Path("design.gv"), top_module="top"),
            map=MapConfig(
                output=Path("result.gemparts"),
                level_split=[20, 40],
                max_stage_degrad=1,
                xprop=True,
            ),
        )
        args = _build_map_args(config)
        assert "map" in args
        assert "--top-module" in args
        assert args[args.index("--top-module") + 1] == "top"
        assert "--level-split" in args
        assert args[args.index("--level-split") + 1] == "20,40"
        assert "--max-stage-degrad" in args
        assert args[args.index("--max-stage-degrad") + 1] == "1"
        assert "--xprop" in args

    def test_missing_netlist_asserts(self):
        config = JacquardConfig(map=MapConfig(output=Path("out.gemparts")))
        with pytest.raises(AssertionError, match="netlist"):
            _build_map_args(config)

    def test_missing_output_asserts(self):
        config = JacquardConfig(design=DesignConfig(netlist=Path("design.gv")))
        with pytest.raises(AssertionError, match="output"):
            _build_map_args(config)


class TestBuildSimArgs:
    def test_minimal(self):
        config = JacquardConfig(
            design=DesignConfig(netlist=Path("design.gv")),
            map=MapConfig(output=Path("result.gemparts")),
            sim=SimConfig(
                input_vcd=Path("input.vcd"),
                output_vcd=Path("output.vcd"),
            ),
        )
        args = _build_sim_args(config)
        assert args[:5] == ["sim", "design.gv", "result.gemparts", "input.vcd", "output.vcd"]

    def test_with_all_options(self):
        config = JacquardConfig(
            design=DesignConfig(
                netlist=Path("design.gv"),
                top_module="top",
                liberty=Path("timing.lib"),
            ),
            map=MapConfig(level_split=[30]),
            sim=SimConfig(
                gemparts=Path("parts.gemparts"),
                input_vcd=Path("in.vcd"),
                output_vcd=Path("out.vcd"),
                num_blocks=128,
                input_vcd_scope="tb.dut",
                output_vcd_scope="dut",
                check_with_cpu=True,
                max_cycles=1000,
                xprop=True,
                dump_signals=["*clk*", "data*"],
                dump_scope="cpu",
                dump_depth=2,
                plusargs={"seed": "42", "force.reset": "0"},
                sdf=SdfConfig(file=Path("design.sdf"), corner="max", debug=True),
                timing=TimingConfig(enabled=True, clock_period=2000, report_violations=True),
            ),
        )
        args = _build_sim_args(config)

        assert "--num-blocks" in args
        assert args[args.index("--num-blocks") + 1] == "128"
        assert "--top-module" in args
        assert "--level-split" in args
        assert "--input-vcd-scope" in args
        assert "--output-vcd-scope" in args
        assert "--check-with-cpu" in args
        assert "--max-cycles" in args
        assert "--xprop" in args
        assert "--dump-scope" in args
        assert args[args.index("--dump-scope") + 1] == "cpu"
        assert "--dump-depth" in args
        assert args[args.index("--dump-depth") + 1] == "2"
        assert "--sdf" in args
        assert "--sdf-corner" in args
        assert args[args.index("--sdf-corner") + 1] == "max"
        assert "--sdf-debug" in args
        assert "--enable-timing" in args
        assert "--timing-clock-period" in args
        assert "--timing-report-violations" in args
        assert "--liberty" in args

        # Check dump_signals appear twice (two patterns)
        dump_idxs = [i for i, a in enumerate(args) if a == "--dump-signals"]
        assert len(dump_idxs) == 2

        # Check plusargs appear
        plusarg_idxs = [i for i, a in enumerate(args) if a == "--plusarg"]
        assert len(plusarg_idxs) == 2
        plusarg_vals = {args[i + 1] for i in plusarg_idxs}
        assert "seed=42" in plusarg_vals
        assert "force.reset=0" in plusarg_vals


class TestApplyOverrides:
    def test_no_overrides(self):
        config = JacquardConfig()
        result = _apply_overrides(config)
        assert result is config  # returns same object when no overrides

    def test_design_overrides(self):
        config = JacquardConfig(design=DesignConfig(netlist=Path("old.gv")))
        result = _apply_overrides(config, netlist="new.gv", top_module="top")
        assert result.design.netlist == Path("new.gv")
        assert result.design.top_module == "top"
        # Original unchanged
        assert config.design.netlist == Path("old.gv")

    def test_sim_overrides(self):
        config = JacquardConfig(sim=SimConfig(max_cycles=1000))
        result = _apply_overrides(config, max_cycles=500, num_blocks=64, xprop=True)
        assert result.sim.max_cycles == 500
        assert result.sim.num_blocks == 64
        assert result.sim.xprop is True

    def test_map_overrides(self):
        config = JacquardConfig()
        result = _apply_overrides(config, level_split=[20, 40])
        assert result.map.level_split == [20, 40]


class TestParseDesignStats:
    SAMPLE_OUTPUT = """\
=== Design Statistics ===
AIG: 5000 pins, 2000 AND gates, 500 DFFs, 4 SRAMs
I/O: 64 inputs, 128 outputs
State: 256 words (1.0 KB reg/io) + 1024 words (4.0 KB SRAM)
Script: 65536 words (256.0 KB)
Partitions: 120 total across 2 stage(s) → 32 block(s)
  Per stage: [60, 60]
  Per block: 3-5 partitions, script 1800-2200 words (balance ratio: 1.15x)
"""

    SAMPLE_XPROP_OUTPUT = SAMPLE_OUTPUT + \
        "X-propagation: 80/120 partitions X-capable\n"

    def test_parse_basic(self):
        stats = parse_design_stats(self.SAMPLE_OUTPUT)
        assert stats is not None
        assert stats.num_aigpins == 5000
        assert stats.num_and_gates == 2000
        assert stats.num_dffs == 500
        assert stats.num_srams == 4
        assert stats.num_inputs == 64
        assert stats.num_outputs == 128
        assert stats.reg_io_state_words == 256
        assert stats.sram_storage_words == 1024
        assert stats.script_data_words == 65536
        assert stats.total_partitions == 120
        assert stats.num_major_stages == 2
        assert stats.num_blocks == 32
        assert stats.min_partitions_per_block == 3
        assert stats.max_partitions_per_block == 5
        assert stats.min_script_size == 1800
        assert stats.max_script_size == 2200
        assert abs(stats.load_balance_ratio - 1.15) < 0.01
        assert stats.xprop_enabled is False

    def test_parse_xprop(self):
        stats = parse_design_stats(self.SAMPLE_XPROP_OUTPUT)
        assert stats is not None
        assert stats.xprop_enabled is True
        assert stats.num_x_capable_partitions == 80

    def test_parse_no_stats(self):
        assert parse_design_stats("some random output") is None


class TestParseSimSummary:
    SAMPLE_OUTPUT = """\

=== Simulation Summary ===
GPU backend: Metal
Cycles simulated: 10000
Wall-clock time: 1.234s
Throughput: 8.10e+03 cycles/sec
State memory: 2.5 KB/cycle
Output: 128 signals → /tmp/output.vcd
"""

    def test_parse_basic(self):
        result = parse_sim_summary(self.SAMPLE_OUTPUT)
        assert result["gpu_backend"] == "Metal"
        assert result["num_cycles"] == 10000
        assert abs(result["wall_clock_secs"] - 1.234) < 0.001
        assert abs(result["throughput"] - 8100.0) < 1.0
        assert abs(result["state_memory_kb"] - 2.5) < 0.01
        assert result["output_signals"] == 128
        assert result["output_vcd"] == Path("/tmp/output.vcd")

    def test_parse_empty(self):
        result = parse_sim_summary("no summary here")
        assert result == {}

    def test_parse_with_plusargs(self):
        output = self.SAMPLE_OUTPUT + "Plusargs: force.reset=0, seed=42\n"
        result = parse_sim_summary(output)
        # Plusargs line doesn't have a dedicated parser field; just check
        # other fields still parse correctly
        assert result["gpu_backend"] == "Metal"

    def test_scientific_notation_throughput(self):
        output = """\
=== Simulation Summary ===
GPU backend: CUDA
Cycles simulated: 500000
Wall-clock time: 0.500s
Throughput: 1.00e+06 cycles/sec
State memory: 10.0 KB/cycle
Output: 256 signals → out.vcd
"""
        result = parse_sim_summary(output)
        assert abs(result["throughput"] - 1_000_000.0) < 1.0
