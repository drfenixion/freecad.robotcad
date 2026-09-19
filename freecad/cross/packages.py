"""Pure-Python helpers to manage the workbench's pip-installed packages.

This module must NOT import FreeCAD at module level: it is loaded both by the
workbench (``freecad/cross/__init__.py``, inside FreeCAD) and, standalone,
by the stdio MCP bridge (``freecad/cross/mcp/stdio_server.py``), which runs
in a separate plain-Python process where ``freecad.cross`` must not be
imported (importing it would run the heavy workbench ``__init__``).

All packages are installed into FreeCAD's ``AdditionalPythonPackages``
directory, which is added to ``sys.path`` by :func:`add_packages_path`.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys


def get_python_exe() -> str:
    """Return the Python executable to use for ``pip`` calls.

    Inside FreeCAD always prefer the bundled interpreter (``get_python_exe``
    from the FreeCAD utilities), which is correct for AppImage builds where
    ``sys.executable`` points at the FreeCAD binary, not at a Python. In a
    plain process (stdio bridge) those imports fail and the interpreter that
    runs the process is used.
    """
    try:  # For v0.21:
        from addonmanager_utilities import get_python_exe
        return get_python_exe()
    except (ModuleNotFoundError, ImportError, AttributeError):
        try:  # For v0.22/v1.0:
            from freecad.utils import get_python_exe
            return get_python_exe()
        except (ModuleNotFoundError, ImportError, AttributeError):
            return sys.executable


def add_packages_path():
    """Dynamically add FreeCAD's AdditionalPythonPackages directory to sys.path.

    The path is PREPENDED (inserted at position 0) so that packages installed
    there take precedence over the base Python environment. This is required
    for ``pip install --target`` installs to actually win over conflicting
    versions already present in the FreeCAD/conda environment (e.g. an older
    ``pydantic`` that the ``mcp`` SDK needs at >= 2.12).
    """
    major = sys.version_info.major
    minor = sys.version_info.minor
    pythonPackagesPath = (
        f'~/.local/share/FreeCAD/AdditionalPythonPackages/py{major}{minor}'
    )
    path = Path(pythonPackagesPath).expanduser().absolute()
    if not path.exists():
        path.mkdir(parents=True)
    if path.exists() and (str(path) not in sys.path):
        sys.path.insert(0, str(path))

    return path


def pip_install(pkg_name):
    """Install a Python package into FreeCAD's AdditionalPythonPackages."""
    packages_path = add_packages_path()

    python_exe = get_python_exe()

    p = subprocess.Popen(
        [
            python_exe, '-m', 'pip', 'install',
            '--disable-pip-version-check',
            '--target', str(packages_path),
            pkg_name,
        ],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )

    for line in iter(p.stdout.readline, b''):
        if line:
            print(line.decode("utf-8"), end="")
    print()

    for err in iter(p.stderr.readline, b''):
        if err:
            print(err.decode("utf-8"), end="")
    print()

    p.stdout.close()
    p.stderr.close()
    p.wait(timeout=180)


def check_install_package(packages_import_name, package_name=None):
    """Install ``packages_import_name`` on demand if it is not importable.

    The check is cheap (``find_spec``): it is safe to call at workbench
    startup, and packages are only pip-installed the first time they are
    actually needed.
    """
    add_packages_path()
    if importlib.util.find_spec(packages_import_name) is None:
        pip_install(package_name or packages_import_name)


def reload_pure_python_module(module_name: str) -> bool:
    """Load a pure-Python module from the AdditionalPythonPackages dir.

    The AdditionalPythonPackages path is appended at the END of ``sys.path``,
    so a freshly pip-installed copy of a module that the FreeCAD/conda
    environment also provides (e.g. ``typing_extensions``) would lose to the
    conda version on a plain ``import``. This loads the module DIRECTLY from
    its file in the packages directory (via
    ``importlib.util.spec_from_file_location``), bypassing ``sys.path``
    entirely, and puts it into ``sys.modules`` — whether or not a stale copy
    was already imported.

    Only safe for pure-Python modules (e.g. ``typing_extensions``); do NOT use
    for modules with C extensions (e.g. ``pydantic``).

    Returns True if the module was loaded from the packages directory.
    """
    import importlib.util
    import sys

    packages_path = add_packages_path()
    module_file = Path(packages_path) / f'{module_name}.py'
    if not module_file.exists():
        return False
    old = sys.modules.get(module_name)
    old_file = getattr(old, '__file__', '') or ''
    # Already loaded from the packages directory: nothing to do.
    if str(module_file) == str(Path(old_file).resolve()):
        return False
    spec = importlib.util.spec_from_file_location(module_name, module_file)
    if spec is None or spec.loader is None:
        return False
    new_module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = new_module
    try:
        spec.loader.exec_module(new_module)
    except Exception:
        # Restore the previous module on failure.
        if old is not None:
            sys.modules[module_name] = old
        else:
            sys.modules.pop(module_name, None)
        return False
    return True