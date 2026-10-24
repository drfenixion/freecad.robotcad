"""Discover and check the workbench's pip dependencies.

The list of Python dependencies is declared in the addon's ``package.xml``
(the single source of truth) as::

    <content>
      <workbench>
        <depend type="python">urdf_parser_py</depend>
        <depend type="python" pip="pycollada" import="collada">pycollada</depend>
        <depend optional="true" type="python">mcp</depend>
      </workbench>
    </content>

Following FreeCAD's ``package.xml`` semantics, the text of a ``<depend>``
element is the **package (pip / PyPI) name** — the name used to install the
package. When the *import* name differs from the package name, it is given by
the ``import`` attribute (or resolved through :data:`KNOWN_IMPORT_NAMES`).
For example ``collada`` is installed from PyPI with ``pip install pycollada``,
so the package name is ``pycollada`` and the import name is ``collada``.

This module uses ``xml.etree`` / ``importlib`` only and does not import
FreeCAD at module level, so it can be reused by non-GUI callers.
"""

from __future__ import annotations

import importlib.util
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:  # Inside FreeCAD.
    from .wb_constants import MOD_PATH
except Exception:  # pragma: no cover - standalone / unexpected import path.
    MOD_PATH = None


# Some dependencies are installed under a PyPI name that differs from the
# import name. Maps package (pip) name -> import name.
KNOWN_IMPORT_NAMES: dict[str, str] = {
    'pycollada': 'collada',
    'ros-ament-index-python': 'ament_index_python',
    'PyYAML': 'yaml',
    'Pillow': 'PIL',
    'opencv-python': 'cv2',
    'scikit-image': 'skimage',
    'scikit-learn': 'sklearn',
    'pyserial': 'serial',
    'attrs': 'attr',
    'PyOpenGL': 'OpenGL',
}


@dataclass
class Dependency:
    """A single Python dependency declared in ``package.xml``.

    ``pip_name`` is the package name used to install (``pip install <pip_name>``);
    ``import_name`` is the name used to test whether it is already importable.
    """

    pip_name: str
    import_name: str
    optional: bool = False

    def __str__(self) -> str:
        return self.pip_name


def _package_xml_path(package_xml: Optional[Path] = None) -> Optional[Path]:
    if package_xml is not None:
        return Path(package_xml)
    if MOD_PATH is not None:
        return Path(MOD_PATH) / 'package.xml'
    # Fallback: this file lives in <addon root>/freecad/cross/dependencies.py.
    return Path(__file__).resolve().parents[2] / 'package.xml'


def get_dependencies(package_xml: Optional[Path] = None) -> list[Dependency]:
    """Return the Python dependencies declared in ``package.xml``.

    Returns an empty list if ``package.xml`` cannot be read or declares no
    Python dependencies.
    """
    path = _package_xml_path(package_xml)
    if path is None or not path.exists():
        return []

    try:
        tree = ET.parse(str(path))
    except ET.ParseError:
        return []

    root = tree.getroot()
    dependencies: list[Dependency] = []

    for elem in root.iter():
        # Element tags may carry a namespace, e.g. {ns}depend.
        if elem.tag.rsplit('}', 1)[-1] != 'depend':
            continue
        if elem.get('type') != 'python':
            continue

        # The element text is the package (pip) name.
        pip_name = (elem.text or '').strip()
        if not pip_name:
            continue

        import_name = (
            elem.get('import')
            or KNOWN_IMPORT_NAMES.get(pip_name)
            or pip_name
        )
        optional = str(elem.get('optional', '')).strip().lower() == 'true'

        dependencies.append(
            Dependency(
                pip_name=pip_name,
                import_name=import_name,
                optional=optional,
            ),
        )

    # De-duplicate, keeping declaration order.
    seen: set[str] = set()
    unique: list[Dependency] = []
    for dep in dependencies:
        if dep.pip_name in seen:
            continue
        seen.add(dep.pip_name)
        unique.append(dep)
    return unique


def is_installed(dep: Dependency | str) -> bool:
    """Return True if the dependency's import name can be found."""
    import_name = dep.import_name if isinstance(dep, Dependency) else dep
    try:
        return importlib.util.find_spec(import_name) is not None
    except (ImportError, ValueError, ModuleNotFoundError):
        return False


def missing_dependencies(
    package_xml: Optional[Path] = None,
    include_optional: bool = True,
) -> list[Dependency]:
    """Return the declared dependencies that are not currently importable.

    Non-optional dependencies are always checked; optional ones only when
    ``include_optional`` is True.
    """
    result: list[Dependency] = []
    for dep in get_dependencies(package_xml):
        if dep.optional and not include_optional:
            continue
        if not is_installed(dep):
            result.append(dep)
    return result
