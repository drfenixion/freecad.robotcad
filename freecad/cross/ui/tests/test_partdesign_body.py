"""`PartDesign::Body` with several features: no double counting.

Every feature of a Body has the cumulative shape of the body up to that
feature, so summing the features would count the same material several
times. `calculate()` must use the shape of the Body (its Tip) only.

The body is built from PartDesign primitives:

* AdditiveBox 100 x 40 x 25 at the origin;
* AdditiveBox 20 x 40 x 25 at x = 100 (fused with the first one: a
  120 x 40 x 25 box);
* SubtractiveBox 10 x 10 x 25 at the origin (a pocket in one corner).

The expected value is the 120 x 40 x 25 box minus the 10 x 10 x 25 box,
assembled from the analytic box formulas (a negative mass for the pocket).
"""
from __future__ import annotations

import numpy as np

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase

DENSITY_KG_PER_M3 = 7850.0
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3

BODY_PLACEMENT = fc.Placement(fc.Vector(10.0, 20.0, -5.0), fc.Rotation(fc.Vector(1, 1, 0), 35.0))


def box6d(lx: float, ly: float, lz: float, corner: fc.Vector, density: float) -> np.ndarray:
    """Generalized 6x6 matrix of an axis-aligned box with its minimum corner at `corner`."""
    mass = density * lx * ly * lz
    com = corner + fc.Vector(lx / 2.0, ly / 2.0, lz / 2.0)
    return analytic.generalized_inertia6d(analytic.box_inertia_com(mass, lx, ly, lz), com, mass)


class PartDesignBodyFeaturesTests(InertiaCalculatorTestCase):

    def setUp(self) -> None:
        super().setUp()
        materials.require_materials_module()
        self.body = self.doc.addObject('PartDesign::Body', 'Body')
        self._add_feature('PartDesign::AdditiveBox', 'Box1', 100.0, 40.0, 25.0, fc.Vector(0.0, 0.0, 0.0))
        self._add_feature('PartDesign::AdditiveBox', 'Box2', 20.0, 40.0, 25.0, fc.Vector(100.0, 0.0, 0.0))
        self._add_feature('PartDesign::SubtractiveBox', 'Pocket', 10.0, 10.0, 25.0, fc.Vector(0.0, 0.0, 0.0))
        self.body.Placement = BODY_PLACEMENT
        materials.assign_material(self.body, materials.make_material('Steel', DENSITY_KG_PER_M3))
        self.recompute()

    def _add_feature(self, type_id: str, name: str, lx: float, ly: float, lz: float, position: fc.Vector):
        feature = self.doc.addObject(type_id, name)
        feature.Length = lx
        feature.Width = ly
        feature.Height = lz
        self.body.addObject(feature)
        feature.Placement = fc.Placement(position, fc.Rotation())
        self.recompute()
        return feature

    def _expected_local(self):
        return (
            box6d(120.0, 40.0, 25.0, fc.Vector(0.0, 0.0, 0.0), DENSITY_KG_PER_MM3)
            - box6d(10.0, 10.0, 25.0, fc.Vector(0.0, 0.0, 0.0), DENSITY_KG_PER_MM3)
        )

    def test_body_setup(self):
        # Guard of the test geometry itself, independent of `calculate()`.
        self.assertEqual(self.body.Tip.Name, 'Pocket')
        self.assertAlmostEqual(self.body.Shape.Volume, 120.0 * 40.0 * 25.0 - 10.0 * 10.0 * 25.0, places=3)

    def test_mass_counts_the_body_once(self):
        actual = ic.calculate(self.body, ignore_placement=True)
        expected_mass = DENSITY_KG_PER_MM3 * (120.0 * 40.0 * 25.0 - 10.0 * 10.0 * 25.0)
        self.assertAlmostEqual(actual[3, 3], expected_mass, delta=1e-9 * expected_mass + 1e-12)

    def test_local_frame(self):
        actual = ic.calculate(self.body, ignore_placement=True)
        self.assertInertia6Equal(actual, self._expected_local(), rtol=1e-4, atol=1e-6)

    def test_parent_frame(self):
        local = self._expected_local()
        mass, com, inertia = self.split_inertia6d(local)
        expected = analytic.transform_inertia6d(inertia, com, mass, BODY_PLACEMENT)
        actual = ic.calculate(self.body)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_body_in_part_container(self):
        container_placement = fc.Placement(fc.Vector(0.0, 0.0, 100.0), fc.Rotation(fc.Vector(0, 0, 1), 90.0))
        container = shapes.make_part_container(self.doc, 'Part', [self.body], placement=container_placement)
        local = self._expected_local()
        mass, com, inertia = self.split_inertia6d(local)
        expected = analytic.transform_inertia6d(inertia, com, mass, container_placement.multiply(BODY_PLACEMENT))
        actual = ic.calculate(container)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)
