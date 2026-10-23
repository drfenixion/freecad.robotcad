"""Case 2: bodies with a non-identity `Placement`.

Split into translation-only, rotation-only and combined sub-cases, plus a
check that `ignore_placement=True` returns the un-placed (local-frame)
answer instead.
"""
from __future__ import annotations

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase

DENSITY_KG_PER_M3 = 7850.0
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3
LENGTH, WIDTH, HEIGHT = 100.0, 40.0, 25.0


class PlacementTestsBase(InertiaCalculatorTestCase):

    def _make_box(self, placement: fc.Placement):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT, placement=placement)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))
        return box

    def _local_mass_com_inertia(self):
        mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)
        return mass, local_com, local_j


class TranslationOnlyTests(PlacementTestsBase):

    def test_translation_only(self):
        placement = fc.Placement(fc.Vector(50.0, -30.0, 10.0), fc.Rotation())
        box = self._make_box(placement)
        mass, local_com, local_j = self._local_mass_com_inertia()
        expected = analytic.transform_inertia6d(local_j, local_com, mass, placement)
        actual = ic.calculate(box)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class RotationOnlyTests(PlacementTestsBase):

    def test_rotation_only(self):
        placement = fc.Placement(fc.Vector(0, 0, 0), fc.Rotation(fc.Vector(0, 0, 1), 90.0))
        box = self._make_box(placement)
        mass, local_com, local_j = self._local_mass_com_inertia()
        expected = analytic.transform_inertia6d(local_j, local_com, mass, placement)
        actual = ic.calculate(box)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class CombinedPlacementTests(PlacementTestsBase):

    def test_translation_and_rotation(self):
        placement = fc.Placement(fc.Vector(12.0, 34.0, -56.0), fc.Rotation(fc.Vector(1, 1, 0), 42.0))
        box = self._make_box(placement)
        mass, local_com, local_j = self._local_mass_com_inertia()
        expected = analytic.transform_inertia6d(local_j, local_com, mass, placement)
        actual = ic.calculate(box)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_ignore_placement_returns_local_frame_result(self):
        placement = fc.Placement(fc.Vector(12.0, 34.0, -56.0), fc.Rotation(fc.Vector(1, 1, 0), 42.0))
        box = self._make_box(placement)
        mass, local_com, local_j = self._local_mass_com_inertia()
        expected_local = analytic.generalized_inertia6d(local_j, local_com, mass)
        actual = ic.calculate(box, ignore_placement=True)
        self.assertInertia6Equal(actual, expected_local, rtol=1e-4, atol=1e-6)
