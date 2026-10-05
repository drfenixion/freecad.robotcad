"""Material helpers for FreeCAD 1.1's `Materials` module.

`inertia_calculator.py` reads density via

    material.PhysicalProperties['Density']

on whatever object is assigned to a shape's `ShapeMaterial` property
(falling back to a `parent_material` passed down from a container/link
ancestor when `ShapeMaterial.Name == 'Default'` or the property is
absent). `ShapeMaterial` holds a full `Materials::Material` object (this
is the modern, FreeCAD >= 1.0 material system; it is unrelated to the
legacy `.FCMat`/`MaterialEditor` card-based material assignment used
elsewhere in this plugin by `command_set_material.py` for CROSS
Link/Robot's own `MaterialCardName`/`MaterialDensity` properties).

The construction pattern below (`Materials.Material()`, then
`addPhysicalModel(uuids.Density)`, then `setPhysicalValue('Density', ...)`)
follows FreeCAD's own `Mod/Material/materialtests/TestMaterialCreation.py`.
"""
from __future__ import annotations

import unittest

import FreeCAD as fc

from ...freecad_utils import getFreeCADversion

try:
    import Materials
except ImportError:  # pragma: no cover - depends on the FreeCAD build.
    Materials = None

# 1 m^3 = 1e9 mm^3, so kg/m^3 -> kg/mm^3 divides by 1e9. Tests specify
# densities in kg/m^3 (the usual way material densities are quoted, e.g.
# steel ~7850) and inertia_calculator.py works internally in kg/mm^3
# (FreeCAD's native length unit is mm), so this factor is needed whenever
# a test computes its own expected mass/inertia from a density it chose
# in kg/m^3.
KG_PER_M3_TO_KG_PER_MM3 = 1.0e-9


def require_materials_module() -> None:
    """Skip the calling test if the `Materials` module / FreeCAD >= 1.1 isn't available.

    This suite targets FreeCAD 1.1, where `ShapeMaterial` is backed by the
    `Materials` module; call this at the top of any test that needs to
    build or assign a `Materials.Material`.
    """
    major, minor = getFreeCADversion()
    if (major, minor) < (1, 1):
        raise unittest.SkipTest(f'requires FreeCAD >= 1.1 (Materials module), got {major}.{minor}')
    if Materials is None:
        raise unittest.SkipTest('the "Materials" module is not importable in this FreeCAD build')


def make_material(name: str, density_kg_per_m3: float) -> 'Materials.Material':
    """Build a standalone `Materials.Material` with a known density.

    `density_kg_per_m3` is in kg/m^3 (e.g. 7850.0 for steel); it is stored
    on the material as a quantity string with explicit units
    ('<value> kg/m^3'), exactly as `inertia_calculator.get_property`'s
    `fc.Units.Quantity(...)`-based reading expects.
    """
    require_materials_module()
    material = Materials.Material()
    material.Name = name
    uuids = Materials.UUIDs()
    material.addPhysicalModel(uuids.Density)
    material.setPhysicalValue('Density', f'{density_kg_per_m3!r} kg/m^3')
    return material


def assign_material(obj: fc.DocumentObject, material: 'Materials.Material') -> None:
    """Assign `material` to `obj.ShapeMaterial`."""
    obj.ShapeMaterial = material
