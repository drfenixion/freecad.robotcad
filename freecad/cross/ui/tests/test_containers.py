"""Case 4: `App::Part` and `App::DocumentObjectGroup` as basic containers.

`calculate()`'s container branch is entered whenever `obj.getSubObjects()`
is non-empty; this is itself the behavior under test (per the project's
"RobotCAD" notes: whether `App::Part`/`App::DocumentObjectGroup` actually
report their children through `getSubObjects()` on the target FreeCAD
version is treated as part of what these tests verify, not an assumption
to hedge against).

Each child's own `Placement` is honored as-is in the sum (the child is
recursed into with `ignore_placement=False`, exactly like a standalone
top-level object), and only then is the container's own `Placement`
(when it has one -- `App::Part` does, `App::DocumentObjectGroup` does
not) applied on top of the combined result. See test_links.py for the
same "outer placement composes with the already-placed inner result"
pattern applied to a single child instead of a sum.
"""
from __future__ import annotations

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase

DENSITY_KG_PER_M3 = 7850.0
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3
LENGTH, WIDTH, HEIGHT = 100.0, 40.0, 25.0


class ContainerTestsBase(InertiaCalculatorTestCase):

    def _make_box(self, name: str, placement: fc.Placement):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, name, LENGTH, WIDTH, HEIGHT, placement=placement)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))
        return box

    def _local_mass_com_inertia(self):
        mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)
        return mass, local_com, local_j


class PartContainerTests(ContainerTestsBase):

    def test_part_container_sums_children_then_applies_own_placement(self):
        p1 = fc.Placement(fc.Vector(0.0, 0.0, 0.0), fc.Rotation())
        p2 = fc.Placement(fc.Vector(100.0, 0.0, 0.0), fc.Rotation(fc.Vector(0, 0, 1), 15.0))
        box1 = self._make_box('Box1', p1)
        box2 = self._make_box('Box2', p2)

        container_placement = fc.Placement(fc.Vector(0.0, 20.0, 0.0), fc.Rotation(fc.Vector(1, 0, 0), 10.0))
        part = shapes.make_part_container(self.doc, 'Container', [box1, box2], placement=container_placement)

        # This is the getSubObjects()-returns-children assumption itself:
        # if App::Part stopped reporting its children this way, this
        # assertion -- not just the final inertia comparison below -- is
        # what would catch it directly. Subobject names are compared with
        # any trailing '.' subelement separator stripped, since the exact
        # suffix convention is incidental to what is being verified here.
        self.assertEqual({n.rstrip('.') for n in part.getSubObjects()}, {'Box1', 'Box2'})

        mass, local_com, local_j = self._local_mass_com_inertia()
        expected = analytic.sum_inertia6d(
            analytic.transform_inertia6d(local_j, local_com, mass, container_placement.multiply(p1)),
            analytic.transform_inertia6d(local_j, local_com, mass, container_placement.multiply(p2)),
        )

        actual = ic.calculate(part)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_part_container_with_identity_own_placement(self):
        p1 = fc.Placement(fc.Vector(0.0, 0.0, 0.0), fc.Rotation())
        p2 = fc.Placement(fc.Vector(0.0, 60.0, 0.0), fc.Rotation())
        box1 = self._make_box('Box1', p1)
        box2 = self._make_box('Box2', p2)
        part = shapes.make_part_container(self.doc, 'Container', [box1, box2])

        mass, local_com, local_j = self._local_mass_com_inertia()
        expected = analytic.sum_inertia6d(
            analytic.transform_inertia6d(local_j, local_com, mass, p1),
            analytic.transform_inertia6d(local_j, local_com, mass, p2),
        )

        actual = ic.calculate(part)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class DocumentObjectGroupTests(ContainerTestsBase):

    def test_group_sums_children_with_no_container_placement(self):
        p1 = fc.Placement(fc.Vector(0.0, 0.0, 0.0), fc.Rotation())
        p2 = fc.Placement(fc.Vector(0.0, 0.0, 70.0), fc.Rotation(fc.Vector(1, 0, 0), 45.0))
        box1 = self._make_box('Box1', p1)
        box2 = self._make_box('Box2', p2)
        group = shapes.make_group(self.doc, 'Group', [box1, box2])

        self.assertFalse(hasattr(group, 'Placement'))
        self.assertEqual({n.rstrip('.') for n in group.getSubObjects()}, {'Box1', 'Box2'})

        mass, local_com, local_j = self._local_mass_com_inertia()
        expected = analytic.sum_inertia6d(
            analytic.transform_inertia6d(local_j, local_com, mass, p1),
            analytic.transform_inertia6d(local_j, local_com, mass, p2),
        )

        actual = ic.calculate(group)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)
