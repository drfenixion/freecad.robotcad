"""Override properties of children inside a container with a 'density' or 'mass' override.

A container (or link) in 'density' or 'mass' mode calculates the geometry
(volume) of its whole subtree and applies its own density or mass to it:
the override properties of the children are not used. In particular a
child in 'manual' mode must contribute its geometry, not its manual mass
and inertia (which are in kg and kg*mm^2 and would otherwise be mixed with
volumes), and a child whose override properties are incomplete must not
raise.

A child in 'ignore' mode is still excluded from the calculation: this is
the behavior chosen in `calculate()` (in volume mode, `calculate_volume`
set by the parent, only the 'ignore' mode of a child is applied),
documented here by a test.

The two children are boxes of different sizes, so a wrong distribution of
the 'mass' override would be visible.
"""
from __future__ import annotations

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase
from .test_overrides import add_density, add_manual_properties, add_mass, add_override_mode

DENSITY_KG_PER_M3 = 7850.0
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3

BOX1 = (100.0, 40.0, 25.0)
BOX2 = (30.0, 60.0, 20.0)
P1 = fc.Placement(fc.Vector(0.0, 0.0, 0.0), fc.Rotation())
P2 = fc.Placement(fc.Vector(120.0, 10.0, 0.0), fc.Rotation(fc.Vector(0, 0, 1), 15.0))
CONTAINER_PLACEMENT = fc.Placement(fc.Vector(0.0, 20.0, 0.0), fc.Rotation(fc.Vector(1, 0, 0), 10.0))


def volume(dims) -> float:
    return dims[0] * dims[1] * dims[2]


def box6d(dims, density: float, placement: fc.Placement):
    """Generalized 6x6 matrix of a `Part::Box` of size `dims` at `placement`."""
    mass = density * volume(dims)
    com = fc.Vector(dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0)
    return analytic.transform_inertia6d(analytic.box_inertia_com(mass, *dims), com, mass, placement)


class OverrideChildrenTestsBase(InertiaCalculatorTestCase):

    def make_children(self):
        box1 = shapes.make_part_box(self.doc, 'Box1', *BOX1, placement=P1)
        box2 = shapes.make_part_box(self.doc, 'Box2', *BOX2, placement=P2)
        return box1, box2

    def make_manual_child(self, box):
        """Give `box` a 'manual' override far from its geometry (1 kg at an arbitrary point)."""
        add_override_mode(box, ic.OVERRIDE_MODE_MANUAL)
        add_manual_properties(
            box, ixx=500.0, iyy=600.0, izz=700.0,
            com_placement=fc.Placement(fc.Vector(-300.0, 50.0, 80.0), fc.Rotation()),
            mass_kg=1.0,
        )

    def make_container(self, children, mode: str):
        container = shapes.make_part_container(self.doc, 'Container', children, placement=CONTAINER_PLACEMENT)
        add_override_mode(container, mode)
        return container

    def expected_density(self, density: float):
        return analytic.sum_inertia6d(
            box6d(BOX1, density, CONTAINER_PLACEMENT.multiply(P1)),
            box6d(BOX2, density, CONTAINER_PLACEMENT.multiply(P2)),
        )

    def expected_mass(self, total_mass: float):
        # Uniform density: the mass is distributed proportionally to volume.
        density = total_mass / (volume(BOX1) + volume(BOX2))
        return self.expected_density(density)


class DensityContainerTests(OverrideChildrenTestsBase):

    def test_manual_child_uses_geometry(self):
        box1, box2 = self.make_children()
        self.make_manual_child(box2)
        container = self.make_container([box1, box2], ic.OVERRIDE_MODE_DENSITY)
        add_density(container, DENSITY_KG_PER_M3)
        actual = ic.calculate(container)
        self.assertInertia6Equal(actual, self.expected_density(DENSITY_KG_PER_MM3), rtol=1e-4, atol=1e-6)

    def test_child_material_is_ignored(self):
        materials.require_materials_module()
        box1, box2 = self.make_children()
        materials.assign_material(box2, materials.make_material('Lead', 11340.0))
        container = self.make_container([box1, box2], ic.OVERRIDE_MODE_DENSITY)
        add_density(container, DENSITY_KG_PER_M3)
        actual = ic.calculate(container)
        self.assertInertia6Equal(actual, self.expected_density(DENSITY_KG_PER_MM3), rtol=1e-4, atol=1e-6)

    def test_child_with_incomplete_override_does_not_raise(self):
        # 'mass' mode without the RobotCadMass property: an error on its own,
        # but its override properties are not used inside the container.
        box1, box2 = self.make_children()
        add_override_mode(box2, ic.OVERRIDE_MODE_MASS)
        container = self.make_container([box1, box2], ic.OVERRIDE_MODE_DENSITY)
        add_density(container, DENSITY_KG_PER_M3)
        actual = ic.calculate(container)
        self.assertInertia6Equal(actual, self.expected_density(DENSITY_KG_PER_MM3), rtol=1e-4, atol=1e-6)

    def test_ignore_child_is_excluded(self):
        box1, box2 = self.make_children()
        add_override_mode(box2, ic.OVERRIDE_MODE_IGNORE)
        container = self.make_container([box1, box2], ic.OVERRIDE_MODE_DENSITY)
        add_density(container, DENSITY_KG_PER_M3)
        expected = box6d(BOX1, DENSITY_KG_PER_MM3, CONTAINER_PLACEMENT.multiply(P1))
        actual = ic.calculate(container)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class MassContainerTests(OverrideChildrenTestsBase):

    def test_manual_child_uses_geometry(self):
        box1, box2 = self.make_children()
        self.make_manual_child(box2)
        container = self.make_container([box1, box2], ic.OVERRIDE_MODE_MASS)
        add_mass(container, 5.0)
        actual = ic.calculate(container)
        self.assertInertia6Equal(actual, self.expected_mass(5.0), rtol=1e-4, atol=1e-6)

    def test_manual_child_in_nested_container(self):
        # The 'mass' override of the outer container also applies through an
        # intermediate container without override.
        box1, box2 = self.make_children()
        self.make_manual_child(box2)
        inner = shapes.make_part_container(self.doc, 'Inner', [box2])
        container = self.make_container([box1, inner], ic.OVERRIDE_MODE_MASS)
        add_mass(container, 5.0)
        actual = ic.calculate(container)
        self.assertInertia6Equal(actual, self.expected_mass(5.0), rtol=1e-4, atol=1e-6)

    def test_density_child_uses_geometry(self):
        box1, box2 = self.make_children()
        add_override_mode(box2, ic.OVERRIDE_MODE_DENSITY)
        add_density(box2, 1000.0)
        container = self.make_container([box1, box2], ic.OVERRIDE_MODE_MASS)
        add_mass(container, 5.0)
        actual = ic.calculate(container)
        self.assertInertia6Equal(actual, self.expected_mass(5.0), rtol=1e-4, atol=1e-6)


class LinkOverrideTests(OverrideChildrenTestsBase):

    def test_density_link_to_manual_object_uses_geometry(self):
        # Same rule for a link in 'density' mode: the 'manual' override of
        # the linked object is not used.
        _, box2 = self.make_children()
        self.make_manual_child(box2)
        link = shapes.make_link(self.doc, 'Link', box2, placement=CONTAINER_PLACEMENT, link_transform=True)
        add_override_mode(link, ic.OVERRIDE_MODE_DENSITY)
        add_density(link, DENSITY_KG_PER_M3)
        expected = box6d(BOX2, DENSITY_KG_PER_MM3, CONTAINER_PLACEMENT.multiply(P2))
        actual = ic.calculate(link)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)
