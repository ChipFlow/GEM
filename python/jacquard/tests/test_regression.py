"""Tests for batch regression runner."""

from pathlib import Path
from unittest.mock import patch

from jacquard.config import DesignConfig, JacquardConfig, MapConfig, SimConfig
from jacquard.regression import (
    RegressionReport,
    TestCase,
    TestResult,
    _run_single_test,
    run_regression,
)
from jacquard.result import SimResult


def _make_config() -> JacquardConfig:
    """Build a minimal config for testing."""
    return JacquardConfig(
        design=DesignConfig(netlist=Path("design.gv")),
        map=MapConfig(output=Path("result.gemparts")),
        sim=SimConfig(
            input_vcd=Path("input.vcd"),
            output_vcd=Path("output.vcd"),
        ),
    )


def _mock_sim_result(returncode: int = 0, cycles: int = 100) -> SimResult:
    return SimResult(
        success=returncode == 0,
        returncode=returncode,
        stdout="",
        stderr="" if returncode == 0 else "Error: something failed",
        elapsed=1.5,
        num_cycles=cycles,
    )


class TestRegressionReport:
    def test_empty_report(self):
        report = RegressionReport()
        assert report.passed == 0
        assert report.failed == 0
        assert report.errors == 0
        assert report.total == 0

    def test_all_passed(self):
        report = RegressionReport(results={
            "test1": TestResult("test1", _mock_sim_result(), passed=True),
            "test2": TestResult("test2", _mock_sim_result(), passed=True),
        })
        assert report.passed == 2
        assert report.failed == 0
        assert report.total == 2

    def test_mixed_results(self):
        report = RegressionReport(results={
            "pass": TestResult("pass", _mock_sim_result(), passed=True),
            "fail": TestResult("fail", _mock_sim_result(returncode=1), passed=False),
            "error": TestResult("error", _mock_sim_result(returncode=-1), passed=False),
        })
        assert report.passed == 1
        assert report.failed == 1
        assert report.errors == 1

    def test_summary_output(self):
        report = RegressionReport(
            results={
                "nvdla": TestResult("nvdla", _mock_sim_result(cycles=10000), passed=True),
                "rocket": TestResult(
                    "rocket",
                    _mock_sim_result(returncode=1),
                    passed=False,
                    error_message="Expected exit 0, got 1",
                ),
            },
            total_wall_clock=5.0,
        )
        summary = report.summary()
        assert "1/2 passed" in summary
        assert "PASS: nvdla" in summary
        assert "FAIL: rocket" in summary
        assert "5.0s" in summary


class TestRunSingleTest:
    @patch("jacquard.regression.sim")
    def test_passing_test(self, mock_sim):
        mock_sim.return_value = _mock_sim_result()
        test = TestCase(name="basic", config=_make_config())
        result = _run_single_test(test, None, None)
        assert result.passed is True
        assert result.error_message == ""

    @patch("jacquard.regression.sim")
    def test_failing_test(self, mock_sim):
        mock_sim.return_value = _mock_sim_result(returncode=1)
        test = TestCase(name="failing", config=_make_config())
        result = _run_single_test(test, None, None)
        assert result.passed is False
        assert "Expected exit 0, got 1" in result.error_message

    @patch("jacquard.regression.sim")
    def test_expected_nonzero_exit(self, mock_sim):
        mock_sim.return_value = _mock_sim_result(returncode=1)
        test = TestCase(name="expected_fail", config=_make_config(), expected_exit=1)
        result = _run_single_test(test, None, None)
        assert result.passed is True

    @patch("jacquard.regression.sim")
    def test_max_cycles_override(self, mock_sim):
        mock_sim.return_value = _mock_sim_result()
        test = TestCase(name="limited", config=_make_config(), max_cycles=50)
        _run_single_test(test, None, None)
        # Verify max_cycles was passed as override
        _, kwargs = mock_sim.call_args
        assert kwargs.get("max_cycles") == 50

    @patch("jacquard.regression.sim")
    def test_exception_handling(self, mock_sim):
        mock_sim.side_effect = RuntimeError("GPU exploded")
        test = TestCase(name="crash", config=_make_config())
        result = _run_single_test(test, None, None)
        assert result.passed is False
        assert result.sim_result.returncode == -1
        assert "GPU exploded" in result.error_message


class TestRunRegression:
    @patch("jacquard.regression.sim")
    def test_sequential_execution(self, mock_sim):
        mock_sim.return_value = _mock_sim_result()
        tests = [
            TestCase(name="test1", config=_make_config()),
            TestCase(name="test2", config=_make_config()),
        ]
        report = run_regression(tests, parallel=1)
        assert report.total == 2
        assert report.passed == 2
        assert report.total_wall_clock > 0

    @patch("jacquard.regression.sim")
    def test_mixed_pass_fail(self, mock_sim):
        def side_effect(config, **kwargs):
            # Second call fails
            if mock_sim.call_count == 2:
                return _mock_sim_result(returncode=1)
            return _mock_sim_result()

        mock_sim.side_effect = side_effect
        tests = [
            TestCase(name="pass", config=_make_config()),
            TestCase(name="fail", config=_make_config()),
        ]
        report = run_regression(tests, parallel=1)
        assert report.passed == 1
        assert report.failed == 1
