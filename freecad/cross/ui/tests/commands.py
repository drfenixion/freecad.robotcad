"""Import helper for tests of code that lives in command modules.

Command modules (`ui/command_*.py`) import `FreeCADGui` and register their
FreeCAD command when imported, so they can only be imported in a FreeCAD with
the GUI (and the whole workbench importable). Tests of functions defined in a
command module are skipped when the module cannot be imported, e.g. under
`freecadcmd`.
"""
from __future__ import annotations

import importlib
import types
import unittest


def import_command(module_name: str) -> types.ModuleType:
    """Return the module `freecad.cross.ui.<module_name>` or skip the calling test."""
    try:
        return importlib.import_module(f'..{module_name}', package=__package__)
    except ImportError as e:  # pragma: no cover - depends on the environment.
        raise unittest.SkipTest(f'{module_name} cannot be imported (needs the FreeCAD GUI): {e}')
