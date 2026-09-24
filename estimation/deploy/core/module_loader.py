#!/usr/bin/env python3
"""Unified module loader for production deployment components.

Centralizes dynamic loading of deploy algorithms while ensuring:
1. Strict locality (only loads from deploy/ directory, no external paths or fallbacks);
2. Caching to prevent redundant imports;
3. Dependency injection support for unit testing and mocking;
4. Clear error diagnostics if any required algorithm module fails to load.
"""
import importlib.util
from pathlib import Path
import sys
from typing import Any, Dict, Optional

_DEPLOY_DIR = Path(__file__).resolve().parent.parent

# Registry of expected deploy algorithm filenames
DEPLOY_MODULE_FILES: Dict[str, str] = {
    'fit_bottle_axis': 'algorithms/fit_bottle_axis.py',
    'fit_estee_box_top_surface': 'algorithms/fit_estee_box_top_surface.py',
    'fit_tube_top_edge': 'algorithms/fit_tube_top_edge.py',
    'fit_front_panel_plane': 'algorithms/fit_front_panel_plane.py',
    'box_selection': 'pipeline/box_selection.py',
    'box_pipeline': 'algorithms/box_pipeline.py',
    'box_geometric_center': 'algorithms/box_geometric_center.py',
}

_CACHE: Dict[str, Any] = {}


def load_deploy_module(name: str, filename: Optional[str] = None, reload: bool = False) -> Any:
    """Loads a deploy module by name strictly from the deploy directory."""
    if not reload and name in _CACHE:
        return _CACHE[name]

    fn = filename or DEPLOY_MODULE_FILES.get(name, f"{name}.py")
    path = _DEPLOY_DIR / fn
    if not path.is_file():
        raise FileNotFoundError(f"Deploy algorithm module '{name}' not found at {path}")

    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load module spec for '{name}' from {path}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    _CACHE[name] = module
    return module


class DeployModules:
    """Encapsulates all 7 core algorithm modules for SKU localization."""

    def __init__(self, overrides: Optional[Dict[str, Any]] = None):
        self._overrides = overrides or {}
        self._modules: Dict[str, Any] = {}

    def get(self, name: str) -> Any:
        if name in self._overrides:
            return self._overrides[name]
        if name not in self._modules:
            self._modules[name] = load_deploy_module(name)
        return self._modules[name]

    @property
    def fit_bottle_axis(self) -> Any:
        return self.get('fit_bottle_axis')

    @property
    def fit_estee_box_top_surface(self) -> Any:
        return self.get('fit_estee_box_top_surface')

    @property
    def fit_tube_top_edge(self) -> Any:
        return self.get('fit_tube_top_edge')

    @property
    def fit_front_panel_plane(self) -> Any:
        return self.get('fit_front_panel_plane')

    @property
    def box_selection(self) -> Any:
        return self.get('box_selection')

    @property
    def box_pipeline(self) -> Any:
        return self.get('box_pipeline')

    @property
    def box_geometric_center(self) -> Any:
        return self.get('box_geometric_center')


_DEFAULT_DEPLOY_MODULES: Optional[DeployModules] = None


def get_deploy_modules() -> DeployModules:
    """Returns the singleton DeployModules container."""
    global _DEFAULT_DEPLOY_MODULES
    if _DEFAULT_DEPLOY_MODULES is None:
        _DEFAULT_DEPLOY_MODULES = DeployModules()
    return _DEFAULT_DEPLOY_MODULES


def set_deploy_module_override(name: str, mock_module: Any) -> None:
    """Injects a mock or replacement module for testing."""
    _CACHE[name] = mock_module
    dm = get_deploy_modules()
    dm._overrides[name] = mock_module


def clear_deploy_module_overrides() -> None:
    """Clears all mock overrides from the global cache."""
    dm = get_deploy_modules()
    dm._overrides.clear()
