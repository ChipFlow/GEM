"""Jacquard — Python API for GPU-accelerated RTL simulation.

Subprocess wrapper around the `jacquard` CLI for batch automation,
regression testing, and result analysis.

Example:
    >>> from jacquard import JacquardConfig, sim
    >>> config = JacquardConfig.from_toml(Path("jacquard.toml"))
    >>> result = sim(config, max_cycles=100)
    >>> assert result.success
"""

from .config import (
    DesignConfig,
    JacquardConfig,
    MapConfig,
    SdfConfig,
    SimConfig,
    TimingConfig,
)
from .errors import (
    BinaryNotFoundError,
    ConfigError,
    JacquardError,
    MappingError,
    SimulationError,
)
from .regression import RegressionReport, TestCase, TestResult, run_regression
from .result import DesignStats, MapResult, SimResult
from .runner import find_jacquard_binary, map, sim

__all__ = [
    # Config
    "JacquardConfig",
    "DesignConfig",
    "MapConfig",
    "SimConfig",
    "SdfConfig",
    "TimingConfig",
    # Runner
    "map",
    "sim",
    "find_jacquard_binary",
    # Results
    "MapResult",
    "SimResult",
    "DesignStats",
    # Regression
    "TestCase",
    "TestResult",
    "RegressionReport",
    "run_regression",
    # Errors
    "JacquardError",
    "ConfigError",
    "MappingError",
    "SimulationError",
    "BinaryNotFoundError",
]
