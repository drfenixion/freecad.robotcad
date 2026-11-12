"""Part boolean operations (Union / Fusion, Cut, Common) are solids, not containers.

A boolean feature has input objects (`Shapes` of `Part::MultiFuse`, `Base`
and `Tool` of `Part::Fuse` / `Part::Cut` / `Part::Common`); the tree view
shows them under the feature, but `calculate()` must not walk them: the
result is the `Shape` of the feature (overlaps counted once, the removed
volume of a cut not counted), with the material of the feature (own or
inherited). `calculate()` does not use the materials of the inputs; note
that FreeCAD itself gives a boolean created without a material the material
of its inputs (observed for `Part::MultiFuse`).

Geometry: two axis-aligned boxes 100 x 40 x 25, the second moved by 60 mm
along x, so they overlap on 40 mm:

* union: one box 160 x 40 x 25;
* cut (first minus second): one box 60 x 40 x 25;
* common: one box 40 x 40 x 25 starting at x = 60.
"""
from __future__ import annotations

import numpy as np

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase

STEEL_KG_PER_M3 = 7850.0
STEEL_KG_PER_MM3 = STEEL_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3
LEAD_KG_PER_M3 = 11340.0
LEAD_KG_PER_MM3 = LEAD_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3

LENGTH, WIDTH, HEIGHT = 100.0, 40.0, 25.0
OFFSET = 60.0
FEATURE_PLACEMENT = fc.Placement(fc.Vector(10.0, -20.0, 5.0), fc.Rotation(fc.Vector(0, 0, 1), 30.0))


def box6d(x0: float, lx: float, density: float = STEEL_KG_PER_MM3) -> np.ndarray:
    """Generalized 6x6 matrix of the box [x0, x0 + lx] x [0, WIDTH] x [0, HEIGHT]."""
    mass = density * lx * WIDTH * HEIGHT
    com = fc.Vector(x0 + lx / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
    return analytic.generalized_inertia6d(analytic.box_inertia_com(mass, lx, WIDTH, HEIGHT), com, mass)


def placed(JJ: np.ndarray, placement: fc.Placement) -> np.ndarray:
    """`JJ` (local frame of the feature) expressed in the parent frame."""
    mass, com, inertia = InertiaCalculatorTestCase.split_inertia6d(JJ)
    return analytic.transform_inertia6d(inertia, com, mass, placement)


UNION = box6d(0.0, LENGTH + OFFSET)
CUT = box6d(0.0, OFFSET)
COMMON = box6d(OFFSET, LENGTH - OFFSET)


class PartBooleanTestsBase(InertiaCalculatorTestCase):

    def setUp(self) -> None:
        super().setUp()
        materials.require_materials_module()
        # Inputs in lead: their material must not be used.
        lead = materials.make_material('Lead', LEAD_KG_PER_M3)
        self.box1 = shapes.make_part_box(self.doc, 'Box1', LENGTH, WIDTH, HEIGHT)
        self.box2 = shapes.make_part_box(
            self.doc, 'Box2', LENGTH, WIDTH, HEIGHT, placement=fc.Placement(fc.Vector(OFFSET, 0.0, 0.0), fc.Rotation()),
        )
        materials.assign_material(self.box1, lead)
        materials.assign_material(self.box2, lead)

    def make_feature(self, type_id: str, name: str, material: bool = True, placement: fc.Placement | None = None):
        feature = self.doc.addObject(type_id, name)
        if type_id == 'Part::MultiFuse':
            feature.Shapes = [self.box1, self.box2]
        else:
            feature.Base = self.box1
            feature.Tool = self.box2
        if placement is not None:
            feature.Placement = placement
        if material:
            materials.assign_material(feature, materials.make_material('Steel', STEEL_KG_PER_M3))
        self.recompute()
        return feature

    def check(self, feature, expected, density: float = STEEL_KG_PER_MM3) -> None:
        # Test assumption: FreeCAD's result is one solid with the expected volume.
        self.assertEqual(len(feature.Shape.Solids), 1, 'Test assumption failed: the result is not one solid.')
        self.assertAlmostEqual(
            feature.Shape.Volume, expected[3, 3] / density, delta=1e-6 * feature.Shape.Volume,
            msg='Test assumption failed: unexpected volume of the boolean result.',
        )
        actual = ic.calculate(feature)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class PartBooleanAssumptionTests(PartBooleanTestsBase):
    """FreeCAD facts the calculator relies on (no `calculate()`)."""

    def test_boolean_features_have_no_sub_objects(self):
        for type_id in ('Part::MultiFuse', 'Part::Fuse', 'Part::Cut', 'Part::Common'):
            with self.subTest(type_id=type_id):
                feature = self.make_feature(type_id, 'Feature')
                self.assertEqual(len(feature.getSubObjects()), 0, f'getSubObjects() = {feature.getSubObjects()}')
                self.assertFalse(hasattr(feature, 'PlacementList'))
                self.doc.removeObject(feature.Name)


class PartBooleanTests(PartBooleanTestsBase):

    def test_union(self):
        # GUI "Union" (Part_Fuse) creates a Part::MultiFuse ("Fusion").
        fusion = self.make_feature('Part::MultiFuse', 'Fusion')
        self.check(fusion, UNION)

    def test_union_with_placement(self):
        fusion = self.make_feature('Part::MultiFuse', 'Fusion', placement=FEATURE_PLACEMENT)
        self.check(fusion, placed(UNION, FEATURE_PLACEMENT))

    def test_fuse_with_base_and_tool(self):
        # Part::Fuse has a `Base` property: it must not be taken for an array.
        fuse = self.make_feature('Part::Fuse', 'Fuse')
        self.check(fuse, UNION)

    def test_cut(self):
        cut = self.make_feature('Part::Cut', 'Cut')
        self.check(cut, CUT)

    def test_common(self):
        common = self.make_feature('Part::Common', 'Common')
        self.check(common, COMMON)

    def test_union_without_material_gets_the_input_material(self):
        # FreeCAD copies the material of the inputs (lead) to a fusion created
        # without one; calculate() then uses the material of the fusion.
        fusion = self.make_feature('Part::MultiFuse', 'Fusion', material=False)
        self.assertEqual(fusion.ShapeMaterial.Name, 'Lead', 'Test assumption failed: FreeCAD did not copy the input material.')
        self.check(fusion, box6d(0.0, LENGTH + OFFSET, density=LEAD_KG_PER_MM3), density=LEAD_KG_PER_MM3)
