"""Case 5: override properties on a plain object and on a container.

Also covers the override-mode validation/error paths: the 'manual' mode's
positive-semi-definiteness and triangle-inequality checks, a missing
required override property, the mode-missing-but-values-present warning
path, and 'ignore' mode's non-traversal guarantee.

These tests add the override properties directly with `obj.addProperty`
(mirroring what `command_override_inertial_properties.py` does), rather
than invoking that command, since only `inertia_calculator.py` is under
test here.
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


def add_override_mode(obj: fc.DocumentObject, mode: str) -> None:
    obj.addProperty('App::PropertyEnumeration', ic.PROP_OVERRIDE_MODE, 'Test', '')
    obj.RobotCadOverrideMode = ic.OVERRIDE_MODES  # Sets the allowed values.
    setattr(obj, ic.PROP_OVERRIDE_MODE, mode)


def add_mass(obj: fc.DocumentObject, mass_kg: float) -> None:
    obj.addProperty('App::PropertyMass', ic.PROP_MASS, 'Test', '')
    setattr(obj, ic.PROP_MASS, mass_kg)


def add_density(obj: fc.DocumentObject, density_kg_per_m3: float) -> None:
    obj.addProperty('App::PropertyDensity', ic.PROP_DENSITY, 'Test', '')
    setattr(obj, ic.PROP_DENSITY, f'{density_kg_per_m3!r} kg/m^3')


def add_manual_properties(obj: fc.DocumentObject, ixx, iyy, izz, com_placement: fc.Placement, mass_kg: float, ixy=0.0, ixz=0.0, iyz=0.0) -> None:
    for name in (ic.PROP_IXX, ic.PROP_IXY, ic.PROP_IXZ, ic.PROP_IYY, ic.PROP_IYZ, ic.PROP_IZZ):
        obj.addProperty('App::PropertyFloat', name, 'Test', '')
    obj.RobotCadIxx, obj.RobotCadIxy, obj.RobotCadIxz = ixx, ixy, ixz
    obj.RobotCadIyy, obj.RobotCadIyz, obj.RobotCadIzz = iyy, iyz, izz
    obj.addProperty('App::PropertyPlacement', ic.PROP_CENTER_OF_MASS, 'Test', '')
    obj.RobotCadCenterOfMass = com_placement
    add_mass(obj, mass_kg)


class LeafDensityOverrideTests(InertiaCalculatorTestCase):

    def test_density_override_on_leaf(self):
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        add_override_mode(box, ic.OVERRIDE_MODE_DENSITY)
        add_density(box, DENSITY_KG_PER_M3)

        mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)
        expected = analytic.transform_inertia6d(local_j, local_com, mass, box.Placement)

        actual = ic.calculate(box)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_density_override_ignores_any_assigned_material(self):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        materials.assign_material(box, materials.make_material('Lead', 11340.0))  # Would give a different mass.
        add_override_mode(box, ic.OVERRIDE_MODE_DENSITY)
        add_density(box, DENSITY_KG_PER_M3)

        mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)
        expected = analytic.transform_inertia6d(local_j, local_com, mass, box.Placement)

        actual = ic.calculate(box)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class LeafMassOverrideTests(InertiaCalculatorTestCase):

    def test_mass_override_on_leaf(self):
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        add_override_mode(box, ic.OVERRIDE_MODE_MASS)
        target_mass = 12.5
        add_mass(box, target_mass)

        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(target_mass, LENGTH, WIDTH, HEIGHT)
        expected = analytic.transform_inertia6d(local_j, local_com, target_mass, box.Placement)

        actual = ic.calculate(box)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class LeafManualOverrideTests(InertiaCalculatorTestCase):

    def test_manual_override_identity_object_placement(self):
        obj = self.doc.addObject('Part::FeaturePython', 'Dummy')
        self.doc.recompute()
        add_override_mode(obj, ic.OVERRIDE_MODE_MANUAL)
        com_placement = fc.Placement(fc.Vector(10.0, 5.0, 2.0), fc.Rotation())
        add_manual_properties(obj, ixx=2000.0, iyy=3000.0, izz=2500.0, com_placement=com_placement, mass_kg=5.0)

        j_principal = np.diag([2000.0, 3000.0, 2500.0])
        expected = analytic.transform_inertia6d(j_principal, fc.Vector(0, 0, 0), 5.0, com_placement)

        actual = ic.calculate(obj)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_manual_override_with_object_placement_and_rotated_principal_frame(self):
        obj = self.doc.addObject('Part::FeaturePython', 'Dummy')
        self.doc.recompute()
        obj.Placement = fc.Placement(fc.Vector(1.0, 2.0, 3.0), fc.Rotation(fc.Vector(0, 0, 1), 30.0))
        add_override_mode(obj, ic.OVERRIDE_MODE_MANUAL)
        com_placement = fc.Placement(fc.Vector(10.0, 5.0, 2.0), fc.Rotation(fc.Vector(1, 0, 0), 20.0))
        add_manual_properties(obj, ixx=2000.0, iyy=3000.0, izz=2500.0, com_placement=com_placement, mass_kg=5.0)

        j_principal = np.diag([2000.0, 3000.0, 2500.0])
        combined_placement = obj.Placement.multiply(com_placement)
        expected = analytic.transform_inertia6d(j_principal, fc.Vector(0, 0, 0), 5.0, combined_placement)

        actual = ic.calculate(obj)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_manual_override_rejects_non_positive_semidefinite_tensor(self):
        obj = self.doc.addObject('Part::FeaturePython', 'Dummy')
        self.doc.recompute()
        add_override_mode(obj, ic.OVERRIDE_MODE_MANUAL)
        add_manual_properties(
            obj, ixx=-10.0, iyy=5.0, izz=5.0,
            com_placement=fc.Placement(), mass_kg=1.0,
        )
        with self.assertRaises(ValueError):
            ic.calculate(obj)

    def test_manual_override_rejects_triangle_inequality_violation(self):
        obj = self.doc.addObject('Part::FeaturePython', 'Dummy')
        self.doc.recompute()
        add_override_mode(obj, ic.OVERRIDE_MODE_MANUAL)
        # Eigenvalues (1, 1, 10): all positive (PSD ok) but 10 > 1 + 1.
        add_manual_properties(
            obj, ixx=1.0, iyy=1.0, izz=10.0,
            com_placement=fc.Placement(), mass_kg=1.0,
        )
        with self.assertRaises(ValueError):
            ic.calculate(obj)

    def test_manual_override_missing_required_property_raises_key_error(self):
        obj = self.doc.addObject('Part::FeaturePython', 'Dummy')
        self.doc.recompute()
        add_override_mode(obj, ic.OVERRIDE_MODE_MANUAL)
        # Deliberately do not add RobotCadIxx (or any of the other value
        # properties): the mode is 'manual' but none of its required
        # properties exist.
        with self.assertRaises(KeyError):
            ic.calculate(obj)


class OverrideModeMissingButValuePresentTests(InertiaCalculatorTestCase):

    def test_warns_and_proceeds_as_none(self):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))
        # RobotCadMass is present but RobotCadOverrideMode never was added:
        # calculate() should warn (not raise) and fall back to 'none',
        # i.e. the stray RobotCadMass value is ignored entirely.
        add_mass(box, 999.0)

        mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)
        expected = analytic.transform_inertia6d(local_j, local_com, mass, box.Placement)

        actual = ic.calculate(box)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class IgnoreModeTests(InertiaCalculatorTestCase):

    def test_ignore_mode_on_leaf_returns_zero(self):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))
        add_override_mode(box, ic.OVERRIDE_MODE_IGNORE)

        actual = ic.calculate(box)
        np.testing.assert_allclose(actual, np.zeros((6, 6)), atol=1e-12)

    def test_ignore_mode_on_container_does_not_traverse_children(self):
        # The child is deliberately broken (manual mode, missing
        # properties): calculate() must never visit it, or this would
        # raise KeyError instead of returning zero.
        poisoned_child = self.doc.addObject('App::FeaturePython', 'Poisoned')
        self.doc.recompute()
        add_override_mode(poisoned_child, ic.OVERRIDE_MODE_MANUAL)

        container = shapes.make_part_container(self.doc, 'Container', [poisoned_child])
        add_override_mode(container, ic.OVERRIDE_MODE_IGNORE)

        actual = ic.calculate(container)
        np.testing.assert_allclose(actual, np.zeros((6, 6)), atol=1e-12)


class ContainerOverrideTests(InertiaCalculatorTestCase):

    def _two_boxes(self):
        p1 = fc.Placement(fc.Vector(0, 0, 0), fc.Rotation())
        p2 = fc.Placement(fc.Vector(100.0, 0.0, 0.0), fc.Rotation(fc.Vector(0, 0, 1), 15.0))
        box1 = shapes.make_part_box(self.doc, 'Box1', LENGTH, WIDTH, HEIGHT, placement=p1)
        box2 = shapes.make_part_box(self.doc, 'Box2', LENGTH, WIDTH, HEIGHT, placement=p2)
        return box1, box2, p1, p2

    def test_density_override_on_container_applies_uniformly_to_all_children(self):
        box1, box2, p1, p2 = self._two_boxes()
        container_placement = fc.Placement(fc.Vector(0, 20.0, 0), fc.Rotation(fc.Vector(1, 0, 0), 10.0))
        container = shapes.make_part_container(self.doc, 'Container', [box1, box2], placement=container_placement)
        add_override_mode(container, ic.OVERRIDE_MODE_DENSITY)
        add_density(container, DENSITY_KG_PER_M3)

        mass_each = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass_each, LENGTH, WIDTH, HEIGHT)
        expected = analytic.sum_inertia6d(
            analytic.transform_inertia6d(local_j, local_com, mass_each, container_placement.multiply(p1)),
            analytic.transform_inertia6d(local_j, local_com, mass_each, container_placement.multiply(p2)),
        )

        actual = ic.calculate(container)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_mass_override_on_container_distributes_proportionally_to_volume(self):
        box1, box2, p1, p2 = self._two_boxes()  # Equal volumes.
        container = shapes.make_part_container(self.doc, 'Container', [box1, box2])
        add_override_mode(container, ic.OVERRIDE_MODE_MASS)
        target_mass = 42.0
        add_mass(container, target_mass)

        # Equal volumes -> equal share of the overridden mass each.
        mass_each = target_mass / 2.0
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass_each, LENGTH, WIDTH, HEIGHT)
        expected = analytic.sum_inertia6d(
            analytic.transform_inertia6d(local_j, local_com, mass_each, p1),
            analytic.transform_inertia6d(local_j, local_com, mass_each, p2),
        )

        actual = ic.calculate(container)
        self.assertInertia6Equal(actual, expected, rtol=1e-3, atol=1e-6)
