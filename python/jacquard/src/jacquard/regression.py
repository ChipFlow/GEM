"""Batch regression runner for multi-design testing."""

from __future__ import annotations

import logging
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path

from .config import JacquardConfig
from .result import SimResult
from .runner import sim

log = logging.getLogger(__name__)


@dataclass
class TestCase:
    """A single regression test case."""

    name: str
    config: JacquardConfig
    expected_exit: int = 0
    max_cycles: int | None = None
    golden_vcd: Path | None = None


@dataclass
class TestResult:
    """Result of a single test case within a regression run."""

    name: str
    sim_result: SimResult
    passed: bool
    error_message: str = ""


@dataclass
class RegressionReport:
    """Aggregated results of a regression run."""

    results: dict[str, TestResult] = field(default_factory=dict)
    total_wall_clock: float = 0.0

    @property
    def passed(self) -> int:
        return sum(1 for r in self.results.values() if r.passed)

    @property
    def failed(self) -> int:
        return sum(
            1 for r in self.results.values()
            if not r.passed and r.sim_result.returncode != -1
        )

    @property
    def errors(self) -> int:
        return sum(1 for r in self.results.values() if r.sim_result.returncode == -1)

    @property
    def total(self) -> int:
        return len(self.results)

    def summary(self) -> str:
        """Human-readable summary of regression results."""
        lines = [
            f"Regression: {self.passed}/{self.total} passed"
            f" ({self.failed} failed, {self.errors} errors)",
            f"Total wall-clock: {self.total_wall_clock:.1f}s",
            "",
        ]
        for name, result in sorted(self.results.items()):
            status = "PASS" if result.passed else "FAIL"
            detail = ""
            if result.sim_result.num_cycles > 0:
                cycles = result.sim_result.num_cycles
                elapsed = result.sim_result.elapsed
                detail = f" ({cycles} cycles, {elapsed:.1f}s)"
            if result.error_message:
                detail += f" [{result.error_message}]"
            lines.append(f"  {status}: {name}{detail}")

        return "\n".join(lines)


def _run_single_test(
    test: TestCase,
    jacquard_bin: Path | None,
    cargo_target_dir: Path | None,
) -> TestResult:
    """Execute a single test case and evaluate pass/fail."""
    overrides: dict = {}
    if test.max_cycles is not None:
        overrides["max_cycles"] = test.max_cycles

    try:
        result = sim(
            test.config,
            jacquard_bin=jacquard_bin,
            cargo_target_dir=cargo_target_dir,
            **overrides,
        )
    except Exception as e:
        return TestResult(
            name=test.name,
            sim_result=SimResult(
                success=False,
                returncode=-1,
                stdout="",
                stderr=str(e),
                elapsed=0.0,
            ),
            passed=False,
            error_message=str(e),
        )

    passed = result.returncode == test.expected_exit
    error_msg = ""
    if not passed:
        error_msg = f"Expected exit {test.expected_exit}, got {result.returncode}"
        if result.stderr:
            # Grab last line of stderr for context
            last_line = result.stderr.strip().splitlines()[-1] if result.stderr.strip() else ""
            if last_line:
                error_msg += f": {last_line[:200]}"

    return TestResult(
        name=test.name,
        sim_result=result,
        passed=passed,
        error_message=error_msg,
    )


def run_regression(
    tests: list[TestCase],
    *,
    jacquard_bin: Path | None = None,
    cargo_target_dir: Path | None = None,
    parallel: int = 1,
) -> RegressionReport:
    """Run a batch of test cases, optionally in parallel.

    Args:
        tests: List of test cases to execute.
        jacquard_bin: Explicit path to jacquard binary.
        cargo_target_dir: Cargo target directory for binary lookup.
        parallel: Maximum number of concurrent tests. Each sim gets its own
                  GPU context, so parallelism is limited by GPU memory.

    Returns:
        RegressionReport with pass/fail results for each test.
    """
    report = RegressionReport()
    start = time.monotonic()

    if parallel <= 1:
        # Sequential execution
        for test in tests:
            log.info("Running test: %s", test.name)
            result = _run_single_test(test, jacquard_bin, cargo_target_dir)
            report.results[test.name] = result
            status = "PASS" if result.passed else "FAIL"
            log.info("  %s: %s (%.1fs)", test.name, status, result.sim_result.elapsed)
    else:
        # Parallel execution
        with ProcessPoolExecutor(max_workers=parallel) as executor:
            futures = {
                executor.submit(
                    _run_single_test, test, jacquard_bin, cargo_target_dir
                ): test.name
                for test in tests
            }
            for future in as_completed(futures):
                name = futures[future]
                try:
                    result = future.result()
                except Exception as e:
                    result = TestResult(
                        name=name,
                        sim_result=SimResult(
                            success=False,
                            returncode=-1,
                            stdout="",
                            stderr=str(e),
                            elapsed=0.0,
                        ),
                        passed=False,
                        error_message=f"Executor error: {e}",
                    )
                report.results[name] = result
                status = "PASS" if result.passed else "FAIL"
                log.info("  %s: %s (%.1fs)", name, status, result.sim_result.elapsed)

    report.total_wall_clock = time.monotonic() - start
    return report
