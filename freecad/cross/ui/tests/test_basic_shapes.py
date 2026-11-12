"""Case 1: basic solids from the Part and PartDesign workbenches, plus compounds.

Local-frame conventions assumed below (standard for these FreeCAD
primitives, unplaced):

* `Part::Box` / `PartDesign::AdditiveBox`: spans `[0, L] x [0, W] x [0, H]`
  from the local origin, so its centroid is at `(L/2, W/2, H/2)`.
* `Part::Cylinder` / `PartDesign::AdditiveCylinder`: axis along local Z
  from `z=0` to `z=Height`, base circle centered on the axis, so its
  centroid is at `(0, 0, Height/2)`.

`calculate(obj)` (default `ignore_placement=False`) is expected to return
the inertia in world frame, since `obj.Shape.CenterOfMass`/`MatrixOfInertia`
already reflect the object's own `Placement`; `analytic.transform_inertia6d`
is used to compute that expectation regardless of whether `Placement` is
the identity, so these tests incidentally also cover the identity-placement
case of what test_placement.py tests more thoroughly for a non-identity one.
"""
from __future__ import annotations

import numpy as np

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase

DENSITY_KG_PER_M3 = 7850.0  # Steel-ish, arbitrary.
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3


class PartBoxTests(InertiaCalculatorTestCase):

    def test_part_box(self):
        materials.require_materials_module()
        length, width, height = 100.0, 40.0, 25.0
        box = shapes.make_part_box(self.doc, 'Box', length, width, height)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))

        mass = DENSITY_KG_PER_MM3 * (length * width * height)
        local_com = fc.Vector(length / 2.0, width / 2.0, height / 2.0)
        local_j = analytic.box_inertia_com(mass, length, width, height)
        expected = analytic.transform_inertia6d(local_j, local_com, mass, box.Placement)

        actual = ic.calculate(box)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class PartCylinderTests(InertiaCalculatorTestCase):

    def test_part_cylinder(self):
        materials.require_materials_module()
        radius, height = 15.0, 60.0
        cyl = shapes.make_part_cylinder(self.doc, 'Cylinder', radius, height)
        materials.assign_material(cyl, materials.make_material('Aluminum', 2700.0))

        density = 2700.0 * materials.KG_PER_M3_TO_KG_PER_MM3
        volume = np.pi * radius ** 2 * height
        mass = density * volume
        local_com = fc.Vector(0.0, 0.0, height / 2.0)
        local_j = analytic.cylinder_inertia_com(mass, radius, height, axis='z')
        expected = analytic.transform_inertia6d(local_j, local_com, mass, cyl.Placement)

        actual = ic.calculate(cyl)
        self.assertInertia6Equal(actual, expected, rtol=1e-3, atol=1e-6)


class PartDesignBoxTests(InertiaCalculatorTestCase):

    def test_partdesign_box(self):
        materials.require_materials_module()
        length, width, height = 80.0, 30.0, 10.0
        body = shapes.make_partdesign_box(self.doc, 'Body', length, width, height)
        materials.assign_material(body, materials.make_material('Steel', DENSITY_KG_PER_M3))

        mass = DENSITY_KG_PER_MM3 * (length * width * height)
        local_com = fc.Vector(length / 2.0, width / 2.0, height / 2.0)
        local_j = analytic.box_inertia_com(mass, length, width, height)
        expected = analytic.transform_inertia6d(local_j, local_com, mass, body.Placement)

        actual = ic.calculate(body)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class PartDesignCylinderTests(InertiaCalculatorTestCase):

    def test_partdesign_cylinder(self):
        materials.require_materials_module()
        radius, height = 12.0, 50.0
        body = shapes.make_partdesign_cylinder(self.doc, 'Body', radius, height)
        materials.assign_material(body, materials.make_material('Brass', 8500.0))

        density = 8500.0 * materials.KG_PER_M3_TO_KG_PER_MM3
        volume = np.pi * radius ** 2 * height
        mass = density * volume
        local_com = fc.Vector(0.0, 0.0, height / 2.0)
        local_j = analytic.cylinder_inertia_com(mass, radius, height, axis='z')
        expected = analytic.transform_inertia6d(local_j, local_com, mass, body.Placement)

        actual = ic.calculate(body)
        self.assertInertia6Equal(actual, expected, rtol=1e-3, atol=1e-6)


class ZeroVolumeAndShapelessObjectTests(InertiaCalculatorTestCase):
    """Additional case: objects with no usable volume/shape return zero."""

    def test_shapeless_object_returns_zero(self):
        # App::DocumentObjectGroup has no Shape at all: falls through every
        # branch except the final "other objects" one.
        obj = self.doc.addObject('App::FeaturePython', 'NoShape')
        self.doc.recompute()
        actual = ic.calculate(obj)
        np.testing.assert_allclose(actual, np.zeros((6, 6)), atol=1e-12)

    def test_null_shape_returns_zero(self):
        # A freshly-added Part::Feature with no shape assigned has a null
        # Shape: hasattr(obj, 'Shape') is True but obj.Shape.isNull() is True.
        obj = self.doc.addObject('Part::Feature', 'EmptyFeature')
        self.doc.recompute()
        actual = ic.calculate(obj)
        np.testing.assert_allclose(actual, np.zeros((6, 6)), atol=1e-12)


class CompoundTests(InertiaCalculatorTestCase):
    """Additional case: a `Part::Compound` combining several solids."""

    def test_compound_of_two_boxes(self):
        materials.require_materials_module()
        length, width, height = 20.0, 20.0, 20.0

        box1 = shapes.make_part_box(self.doc, 'Box1', length, width, height)
        box2 = shapes.make_part_box(
            self.doc, 'Box2', length, width, height,
            placement=fc.Placement(fc.Vector(100.0, 0.0, 0.0), fc.Rotation()),
        )
        compound = shapes.make_compound(self.doc, 'Compound', [box1, box2])
        materials.assign_material(compound, materials.make_material('Steel', DENSITY_KG_PER_M3))

        mass_each = DENSITY_KG_PER_MM3 * (length * width * height)
        local_com = fc.Vector(length / 2.0, width / 2.0, height / 2.0)
        local_j = analytic.box_inertia_com(mass_each, length, width, height)
        expected = analytic.sum_inertia6d(
            analytic.transform_inertia6d(local_j, local_com, mass_each, box1.Placement),
            analytic.transform_inertia6d(local_j, local_com, mass_each, box2.Placement),
        )
        # The compound's own Placement is identity here, so it does not
        # add a further transform on top of the sum above: assert that
        # explicitly rather than silently relying on it.
        self.assertEqual(compound.Placement, fc.Placement())

        actual = ic.calculate(compound)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)
