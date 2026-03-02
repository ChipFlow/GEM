"""Jacquard error hierarchy for Python API."""


class JacquardError(Exception):
    """Base error for all Jacquard operations."""


class ConfigError(JacquardError):
    """Error in configuration parsing or validation."""


class MappingError(JacquardError):
    """Error during partition mapping (jacquard map)."""

    def __init__(self, message: str, returncode: int = 1, stderr: str = ""):
        super().__init__(message)
        self.returncode = returncode
        self.stderr = stderr


class SimulationError(JacquardError):
    """Error during GPU simulation (jacquard sim)."""

    def __init__(self, message: str, returncode: int = 1, stderr: str = ""):
        super().__init__(message)
        self.returncode = returncode
        self.stderr = stderr


class BinaryNotFoundError(JacquardError):
    """Jacquard binary not found in PATH or cargo target directory."""
