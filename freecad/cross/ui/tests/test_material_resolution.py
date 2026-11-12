"""Material resolution: own material, inherited material, and missing/malformed material.

`calculate()` resolves a solid object's material as: its own
`ShapeMaterial` if present and not named `'Default'`, otherwise whatever
`parent_material` was passed in by its caller (a container or link
ancestor), otherwise `None` (treated as "assume zero mass", with a
warning).

The "inherited material" case is tested by calling `calculate()` directly
with `parent_material=...`, which is the exact mechanism a container or
link ancestor uses to pass its own resolved material down -- this avoids
depending on whether `App::Part`/`App::Link` themselves expose a
`ShapeMaterial` property (which, unlike `Part::Feature`, is not
guaranteed), while still exactly exercising the inheritance branch in
`calculate()`'s own code.
"""
from __future__ import annotations

import numpy as np

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase

DENSITY_KG_PER_M3 = 7850.0
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3
LENGTH, WIDTH, HEIGHT = 100.0, 40.0, 25.0


class OwnMaterialTests(InertiaCalculatorTestCase):

    def test_own_shape_material_is_used(self):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))

        mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)
        expected = analytic.transform_inertia6d(local_j, local_com, mass, box.Placement)

        actual = ic.calculate(box)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class InheritedMaterialTests(InertiaCalculatorTestCase):

    def test_parent_material_is_used_when_own_material_is_absent(self):
        materials.require_materials_module()
        # No ShapeMaterial assigned to the box itself.
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        inherited = materials.make_material('InheritedSteel', DENSITY_KG_PER_M3)

        mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)
        expected = analytic.transform_inertia6d(local_j, local_com, mass, box.Placement)

        actual = ic.calculate(box, parent_material=inherited)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_own_material_named_default_falls_back_to_parent(self):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        if hasattr(box, 'ShapeMaterial'):
            self.assertEqual(box.ShapeMaterial.Name, 'Default', 'a fresh Part::Box is expected to start with the placeholder "Default" material')
        inherited = materials.make_material('InheritedSteel', DENSITY_KG_PER_M3)

        mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)
        expected = analytic.transform_inertia6d(local_j, local_com, mass, box.Placement)

        actual = ic.calculate(box, parent_material=inherited)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class MissingOrMalformedMaterialTests(InertiaCalculatorTestCase):

    def test_no_material_anywhere_assumes_zero_mass(self):
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        actual = ic.calculate(box)  # No ShapeMaterial, no parent_material.
        np.testing.assert_allclose(actual, np.zeros((6, 6)), atol=1e-12)

    def test_material_without_density_physical_model_assumes_zero_mass(self):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        import Materials  # Safe: require_materials_module() already checked this import.
        material = Materials.Material()
        material.Name = 'NoDensityModel'
        # Deliberately never call material.addPhysicalModel(uuids.Density),
        # so material.PhysicalProperties['Density'] raises KeyError, which
        # calculate() is documented to catch and treat as zero mass.
        materials.assign_material(box, material)

        actual = ic.calculate(box)
        np.testing.assert_allclose(actual, np.zeros((6, 6)), atol=1e-12)
