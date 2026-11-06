"""Composable model plugins. Importing this package loads no ML/device backends."""
from .policy import ModelPolicy

__all__ = ["ModelPolicy"]
