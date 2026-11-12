"""`App::Link` with a scale, and chains of links with placements and scales.

FreeCAD applies a link's scale (`ScaleVector`, or the uniform `Scale`) in the
local frame of the link, before its `Placement`:

    x_parent = Link.Placement * (diag(ScaleVector) @ x_linked)

where `x_linked` includes the linked object's own `Placement` if
`LinkTransform` is True. Each test checks `calculate()` against two
references built without `inertia_calculator`:

* an analytic one, from the box formulas and a general affine transform
  (`analytic.affine_transform_inertia6d`);
* FreeCAD's own geometry, `Part.getShape(link)`, which has all link
  placements and scales applied. Comparing the two references first checks
  the transform convention above; if that comparison fails, the assumption
  about FreeCAD's semantics is wrong, not `calculate()`.
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

BOX_PLACEMENT = fc.Placement(fc.Vector(20.0, 0.0, 0.0), fc.Rotation(fc.Vector(0, 0, 1), 30.0))
LINK_PLACEMENT = fc.Placement(fc.Vector(0.0, 50.0, 10.0), fc.Rotation(fc.Vector(1, 0, 0), 60.0))
LINK2_PLACEMENT = fc.Placement(fc.Vector(-40.0, 15.0, 5.0), fc.Rotation(fc.Vector(0, 1, 1), 25.0))


def box_mass_com_inertia():
    """Mass, center of mass and inertia about it of the box in its local frame."""
    mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
    com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
    return mass, com, analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)


class LinkScaleTestsBase(InertiaCalculatorTestCase):

    def setUp(self) -> None:
        super().setUp()
        materials.require_materials_module()
        import Part  # noqa: F401 (Part.getShape is used as a reference)
        self.Part = Part
        self.box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT, placement=BOX_PLACEMENT)
        materials.assign_material(self.box, materials.make_material('Steel', DENSITY_KG_PER_M3))

    def make_link(self, name, linked_obj, placement, link_transform=True, scale=None):
        link = shapes.make_link(self.doc, name, linked_obj, placement=placement, link_transform=link_transform)
        if scale is not None:
            link.ScaleVector = fc.Vector(*scale)
        self.recompute()
        return link

    def expected_for(self, affine_map):
        mass, com, inertia = box_mass_com_inertia()
        return analytic.affine_transform_inertia6d(inertia, com, mass, affine_map)

    def check(self, obj, expected):
        """Check the references against each other, then `calculate(obj)`."""
        reference = shapes.shape_inertia6d(self.Part.getShape(obj), DENSITY_KG_PER_MM3)
        self.assertInertia6Equal(
            reference, expected, rtol=1e-4, atol=1e-6,
            msg='Test assumption failed: FreeCAD shape of the link differs from the analytic reference.',
        )
        actual = ic.calculate(obj)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class LinkScaleTests(LinkScaleTestsBase):

    def test_uniform_scale_vector(self):
        link = self.make_link('Link', self.box, LINK_PLACEMENT, scale=(1.5, 1.5, 1.5))
        expected = self.expected_for(analytic.compose_affine(
            analytic.affine(LINK_PLACEMENT, (1.5, 1.5, 1.5)), analytic.affine(BOX_PLACEMENT),
        ))
        self.check(link, expected)

    def test_uniform_scale_property(self):
        # `Scale` is the uniform scale property of App::Link, kept in sync
        # with `ScaleVector` by FreeCAD.
        link = self.make_link('Link', self.box, LINK_PLACEMENT)
        link.Scale = 2.0
        self.recompute()
        expected = self.expected_for(analytic.compose_affine(
            analytic.affine(LINK_PLACEMENT, (2.0, 2.0, 2.0)), analytic.affine(BOX_PLACEMENT),
        ))
        self.check(link, expected)

    def test_non_uniform_scale_link_transform_true(self):
        # The box is rotated in the link frame, so the non-uniform scale
        # shears it: the result is not a box any more.
        scale = (2.0, 1.0, 0.5)
        link = self.make_link('Link', self.box, LINK_PLACEMENT, link_transform=True, scale=scale)
        expected = self.expected_for(analytic.compose_affine(
            analytic.affine(LINK_PLACEMENT, scale), analytic.affine(BOX_PLACEMENT),
        ))
        self.check(link, expected)

    def test_non_uniform_scale_link_transform_false(self):
        scale = (2.0, 1.0, 0.5)
        link = self.make_link('Link', self.box, LINK_PLACEMENT, link_transform=False, scale=scale)
        expected = self.expected_for(analytic.affine(LINK_PLACEMENT, scale))
        self.check(link, expected)

    def test_ignore_placement_keeps_the_scale(self):
        # `ignore_placement=True` drops the link Placement, not its scale.
        scale = (2.0, 1.0, 0.5)
        link = self.make_link('Link', self.box, LINK_PLACEMENT, link_transform=True, scale=scale)
        expected = self.expected_for(analytic.compose_affine(
            analytic.affine(None, scale), analytic.affine(BOX_PLACEMENT),
        ))
        actual = ic.calculate(link, ignore_placement=True)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_mirroring_scale_gives_positive_mass(self):
        # A negative scale component mirrors the linked object: the mass is
        # |sx * sy * sz| times the mass of the linked object, never negative.
        scale = (-1.0, 1.0, 1.0)
        link = self.make_link('Link', self.box, LINK_PLACEMENT, link_transform=True, scale=scale)
        expected = self.expected_for(analytic.compose_affine(
            analytic.affine(LINK_PLACEMENT, scale), analytic.affine(BOX_PLACEMENT),
        ))
        actual = ic.calculate(link)
        self.assertGreater(actual[3, 3], 0.0)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_scale_does_not_change_the_linked_object_result(self):
        # Regression guard: scaling a link must not modify the matrix of the
        # linked object (no in-place update of a shared array).
        before = ic.calculate(self.box)
        link = self.make_link('Link', self.box, LINK_PLACEMENT, scale=(2.0, 1.0, 0.5))
        ic.calculate(link)
        after = ic.calculate(self.box)
        np.testing.assert_allclose(after, before)


class LinkChainTests(LinkScaleTestsBase):
    """Link2 -> Link1 -> Box, every element with its own placement and scale."""

    def test_chain_with_link_transform(self):
        link1 = self.make_link('Link1', self.box, LINK_PLACEMENT, link_transform=True, scale=(1.0, 2.0, 1.0))
        link2 = self.make_link('Link2', link1, LINK2_PLACEMENT, link_transform=True, scale=(0.5, 1.0, 1.5))
        expected = self.expected_for(analytic.compose_affine(
            analytic.affine(LINK2_PLACEMENT, (0.5, 1.0, 1.5)),
            analytic.affine(LINK_PLACEMENT, (1.0, 2.0, 1.0)),
            analytic.affine(BOX_PLACEMENT),
        ))
        self.check(link2, expected)

    def test_chain_outer_link_without_link_transform(self):
        # Link2.LinkTransform = False drops the Placement of Link1, but the
        # scale of Link1 still applies.
        link1 = self.make_link('Link1', self.box, LINK_PLACEMENT, link_transform=True, scale=(1.0, 2.0, 1.0))
        link2 = self.make_link('Link2', link1, LINK2_PLACEMENT, link_transform=False, scale=(0.5, 1.0, 1.5))
        expected = self.expected_for(analytic.compose_affine(
            analytic.affine(LINK2_PLACEMENT, (0.5, 1.0, 1.5)),
            analytic.affine(None, (1.0, 2.0, 1.0)),
            analytic.affine(BOX_PLACEMENT),
        ))
        self.check(link2, expected)

    def test_chain_inner_link_without_link_transform(self):
        link1 = self.make_link('Link1', self.box, LINK_PLACEMENT, link_transform=False, scale=(1.0, 2.0, 1.0))
        link2 = self.make_link('Link2', link1, LINK2_PLACEMENT, link_transform=True)
        expected = self.expected_for(analytic.compose_affine(
            analytic.affine(LINK2_PLACEMENT),
            analytic.affine(LINK_PLACEMENT, (1.0, 2.0, 1.0)),
        ))
        self.check(link2, expected)

    def test_chain_of_three_links_without_scale(self):
        # Intermediate placements must not be skipped (a recursive
        # `getLinkedObject()` would jump straight to the box).
        link1 = self.make_link('Link1', self.box, LINK_PLACEMENT, link_transform=True)
        link2 = self.make_link('Link2', link1, LINK2_PLACEMENT, link_transform=True)
        link3_placement = fc.Placement(fc.Vector(3.0, -4.0, 8.0), fc.Rotation(fc.Vector(0, 0, 1), -70.0))
        link3 = self.make_link('Link3', link2, link3_placement, link_transform=True)
        expected = self.expected_for(analytic.compose_affine(
            analytic.affine(link3_placement),
            analytic.affine(LINK2_PLACEMENT),
            analytic.affine(LINK_PLACEMENT),
            analytic.affine(BOX_PLACEMENT),
        ))
        self.check(link3, expected)
