"""Case 3: `App::Link` pointing at a body, with and without `LinkTransform`.

`App::Link.LinkTransform` controls whether the linked object's own
`Placement` is honored on top of the link's own `Placement`:

* `LinkTransform=True`: the link's final placement is
  `Link.Placement * LinkedObject.Placement` (the linked object's own
  placement still applies, composed under the link's).
* `LinkTransform=False`: the linked object's own `Placement` is ignored;
  only the link's own `Placement` applies, as if the linked object sat at
  the origin.

`calculate()` implements this by recursing into the linked object with
`ignore_placement=not obj.LinkTransform`, then applying the link's own
`Placement` on top of whatever came back (see the "link objects" branch).
The expected values below are built directly from this specification
(independently of the implementation) using `fc.Placement.multiply`.
"""
from __future__ import annotations

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase

DENSITY_KG_PER_M3 = 7850.0
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3
LENGTH, WIDTH, HEIGHT = 100.0, 40.0, 25.0


class LinkTestsBase(InertiaCalculatorTestCase):

    def _make_linked_box(self, box_placement: fc.Placement, link_placement: fc.Placement, link_transform: bool):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT, placement=box_placement)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))
        link = shapes.make_link(self.doc, 'Link', box, placement=link_placement, link_transform=link_transform)
        return link

    def _local_mass_com_inertia(self):
        mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
        local_com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        local_j = analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)
        return mass, local_com, local_j


class LinkTransformTrueTests(LinkTestsBase):

    def test_link_transform_true_composes_both_placements(self):
        box_placement = fc.Placement(fc.Vector(20.0, 0.0, 0.0), fc.Rotation(fc.Vector(0, 0, 1), 30.0))
        link_placement = fc.Placement(fc.Vector(0.0, 50.0, 10.0), fc.Rotation(fc.Vector(1, 0, 0), 60.0))
        link = self._make_linked_box(box_placement, link_placement, link_transform=True)

        mass, local_com, local_j = self._local_mass_com_inertia()
        composed = link_placement.multiply(box_placement)
        expected = analytic.transform_inertia6d(local_j, local_com, mass, composed)

        actual = ic.calculate(link)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class LinkTransformFalseTests(LinkTestsBase):

    def test_link_transform_false_ignores_linked_object_placement(self):
        box_placement = fc.Placement(fc.Vector(20.0, 0.0, 0.0), fc.Rotation(fc.Vector(0, 0, 1), 30.0))
        link_placement = fc.Placement(fc.Vector(0.0, 50.0, 10.0), fc.Rotation(fc.Vector(1, 0, 0), 60.0))
        link = self._make_linked_box(box_placement, link_placement, link_transform=False)

        mass, local_com, local_j = self._local_mass_com_inertia()
        expected = analytic.transform_inertia6d(local_j, local_com, mass, link_placement)

        actual = ic.calculate(link)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_link_transform_false_with_identity_box_placement_matches_true(self):
        # With no placement on the linked object itself, LinkTransform
        # shouldn't matter: both modes degenerate to the same answer.
        link_placement = fc.Placement(fc.Vector(5.0, 6.0, 7.0), fc.Rotation(fc.Vector(0, 1, 0), 15.0))
        link_false = self._make_linked_box(fc.Placement(), link_placement, link_transform=False)

        mass, local_com, local_j = self._local_mass_com_inertia()
        expected = analytic.transform_inertia6d(local_j, local_com, mass, link_placement)

        actual = ic.calculate(link_false)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)
