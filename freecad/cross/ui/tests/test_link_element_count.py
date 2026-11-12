"""`App::Link` array: an `App::Link` with `ElementCount` > 0 (not a Draft array).

Example: the four wheels of a mobile robot base. The wheel is modelled once
(a `Part::Cylinder` turned so that its axis is along Y). In the GUI: select
the wheel, "Make link" (Std_LinkMake), then set `ElementCount = 4` in the
property editor of the new link. With `ShowElement = True` (default) the link
gets four element objects (`App::LinkElement`, in `ElementList`) whose
`Placement` is set like any object, and `PlacementList` stays empty; with
`ShowElement = False` there are no element objects and the element
transforms are in `PlacementList` / `ScaleList`. Contrary to a Draft link
array, such a link has no `Base` and no `ExpandArray` property and its
`LinkedObject` is the wheel itself.

The tests use `LinkTransform = False`: the placement of the wheel is not
used and each element placement is the full placement of a wheel. The
transform of element `i` is then

    x = Link.Placement * diag(Link.ScaleVector) * P_i * diag(S_i) * x_wheel

with `P_i`, `S_i` the element placement and scale (`x_wheel` in the local
frame of the cylinder). Without scales this model is first checked against
FreeCAD's own geometry, `Part.getShape(link)`; with scales that check is
skipped: the scales change the 3D view but not `Shape.Solids`.

With `LinkTransform = True` the placement `B` of the wheel is applied too,
inside the element placement, as for a single link and as FreeCAD displays
it (checked in the 3D view):

    x = Link.Placement * P_i * B * x_wheel

`Part.getShape(link)` does not follow this order (it applies `B` outside
`P_i`, `LinkArrayAssumptionTests.test_part_get_shape_with_link_transform`),
so it is not used as a reference with `LinkTransform = True`.
"""
from __future__ import annotations

import math
import unittest

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase

DENSITY_KG_PER_M3 = 7850.0
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3

# Wheel: radius 60 mm, width 30 mm. The cylinder is created along local Z and
# turned so that its axis is along Y, centered on the y = 0 plane, with its
# center 60 mm above the ground (z = 60).
RADIUS, WIDTH = 60.0, 30.0
WHEEL_PLACEMENT = fc.Placement(fc.Vector(0.0, WIDTH / 2.0, RADIUS), fc.Rotation(fc.Vector(1, 0, 0), 90.0))

# Wheel positions (wheelbase 300 mm, track 260 mm): front-left, front-right,
# rear-left, rear-right.
WHEEL_POSITIONS = [
    fc.Vector(150.0, 130.0, 0.0),
    fc.Vector(150.0, -130.0, 0.0),
    fc.Vector(-150.0, 130.0, 0.0),
    fc.Vector(-150.0, -130.0, 0.0),
]
COUNT = len(WHEEL_POSITIONS)

# Placement of the whole array (e.g. the wheels moved with the base).
ARRAY_PLACEMENT = fc.Placement(fc.Vector(10.0, -20.0, 5.0), fc.Rotation(fc.Vector(0, 0, 1), 30.0))

# Chassis plate of the base, above the wheels.
CHASSIS = (400.0, 220.0, 20.0)
CHASSIS_PLACEMENT = fc.Placement(fc.Vector(-200.0, -110.0, 70.0), fc.Rotation())


def wheel_mass_com_inertia(density: float = DENSITY_KG_PER_MM3):
    """Mass, center of mass and inertia about it of the wheel in its local frame (cylinder along Z)."""
    mass = density * math.pi * RADIUS ** 2 * WIDTH
    com = fc.Vector(0.0, 0.0, WIDTH / 2.0)
    return mass, com, analytic.cylinder_inertia_com(mass, RADIUS, WIDTH, axis='z')


def translations():
    """Element placements that only move the wheel (used with `LinkTransform = True`)."""
    return [fc.Placement(position, fc.Rotation()) for position in WHEEL_POSITIONS]


def wheel_placements():
    """Full wheel placements: moved and turned (used with `LinkTransform = False`)."""
    return [p.multiply(WHEEL_PLACEMENT) for p in translations()]


class LinkArrayTestsBase(InertiaCalculatorTestCase):

    def setUp(self) -> None:
        super().setUp()
        materials.require_materials_module()
        import Part
        self.Part = Part
        self.wheel = shapes.make_part_cylinder(self.doc, 'Wheel', RADIUS, WIDTH, placement=WHEEL_PLACEMENT)
        materials.assign_material(self.wheel, materials.make_material('Steel', DENSITY_KG_PER_M3))

    def make_link_array(
            self,
            placements,
            scales=None,
            link_transform: bool = False,
            show_element: bool = True,
            placement: fc.Placement | None = None,
    ):
        """`App::Link` to the wheel with one element per entry of `placements`.

        Element transforms are set like a user does: on the element objects
        when they are shown, in `PlacementList` / `ScaleList` otherwise.
        """
        link = shapes.make_link(self.doc, 'Wheels', self.wheel, placement=placement, link_transform=link_transform)
        link.ShowElement = show_element
        link.ElementCount = len(placements)
        self.recompute()
        if show_element:
            self.assertEqual(len(link.ElementList), len(placements), 'Test assumption failed: no element objects.')
            for i, (element, p) in enumerate(zip(link.ElementList, placements)):
                element.Placement = p
                if scales is not None:
                    element.ScaleVector = scales[i]
        else:
            # ScaleList is left as FreeCAD sets it when no scale is given.
            link.PlacementList = list(placements)
            if scales is not None:
                link.ScaleList = list(scales)
        self.recompute()
        return link

    def model(self, placements, scales=None, placement=None, link_scale=(1.0, 1.0, 1.0), density=DENSITY_KG_PER_MM3, base=None):
        """Analytic 6x6 matrix of the array (see the module docstring).

        `base`: placement of the wheel applied inside the element placement
        (`P_i * base`), as FreeCAD displays it with `LinkTransform = True`.
        """
        mass, com, inertia = wheel_mass_com_inertia(density)
        if scales is None:
            scales = [fc.Vector(1.0, 1.0, 1.0)] * len(placements)
        parent = analytic.affine(placement, link_scale)
        base_map = analytic.affine(base)
        return analytic.sum_inertia6d(*[
            analytic.affine_transform_inertia6d(
                inertia, com, mass,
                analytic.compose_affine(parent, analytic.affine(p, (s.x, s.y, s.z)), base_map),
            )
            for p, s in zip(placements, scales)
        ])

    def check(self, obj, expected, compare_shape: bool = True, density=DENSITY_KG_PER_MM3):
        """Check the analytic model against FreeCAD's shape (optional), then `calculate(obj)`."""
        if compare_shape:
            reference = shapes.shape_inertia6d(self.Part.getShape(obj), density)
            self.assertInertia6Equal(
                reference, expected, rtol=1e-4, atol=1e-6,
                msg='Test assumption failed: FreeCAD shape of the link array differs from the analytic model.',
            )
        actual = ic.calculate(obj)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class LinkArrayAssumptionTests(LinkArrayTestsBase):
    """FreeCAD facts about an `App::Link` with `ElementCount` > 0 (no `calculate()`)."""

    def test_not_a_draft_link_array(self):
        link = self.make_link_array(wheel_placements())
        self.assertFalse(hasattr(link, 'Base'))
        self.assertFalse(hasattr(link, 'ExpandArray'))
        self.assertIs(link.LinkedObject, self.wheel)

    def test_linked_object_of_a_link_array(self):
        # LinkBaseExtension returns the link itself when ElementCount > 0, so
        # `calculate()` does not take the "link" branch for it.
        link = self.make_link_array(wheel_placements())
        self.assertIs(link.getLinkedObject(recursive=False), link)

    def test_sub_objects_are_the_elements(self):
        for show_element in (True, False):
            with self.subTest(show_element=show_element):
                link = self.make_link_array(wheel_placements(), show_element=show_element)
                self.assertEqual(len(link.getSubObjects()), COUNT, f'getSubObjects() = {link.getSubObjects()}')
                self.doc.removeObject(link.Name)

    def test_placement_list_is_empty_with_element_objects(self):
        # With ShowElement = True the transforms are only on the elements.
        link = self.make_link_array(wheel_placements())
        self.assertEqual(len(link.ElementList), COUNT)
        self.assertEqual(len(link.PlacementList), 0)

    def test_shape_volume(self):
        link = self.make_link_array(wheel_placements())
        self.assertAlmostEqual(
            self.Part.getShape(link).Volume, COUNT * math.pi * RADIUS ** 2 * WIDTH, delta=1e-6 * COUNT * RADIUS ** 2 * WIDTH,
        )

    def test_part_get_shape_with_link_transform(self):
        # Observed: with LinkTransform = True `Part.getShape()` applies the
        # wheel placement B outside the element placement,
        # x = Placement * B * P_i * x_wheel, while the 3D view (and
        # `calculate()`) use Placement * P_i * B. Not a reference for this case.
        link = self.make_link_array(translations(), link_transform=True, placement=ARRAY_PLACEMENT)
        mass, com, inertia = wheel_mass_com_inertia()
        expected = analytic.sum_inertia6d(*[
            analytic.affine_transform_inertia6d(
                inertia, com, mass,
                analytic.compose_affine(analytic.affine(ARRAY_PLACEMENT), analytic.affine(WHEEL_PLACEMENT), analytic.affine(p)),
            )
            for p in translations()
        ])
        reference = shapes.shape_inertia6d(self.Part.getShape(link), DENSITY_KG_PER_MM3)
        self.assertInertia6Equal(reference, expected, rtol=1e-4, atol=1e-6)


class LinkArrayTests(LinkArrayTestsBase):

    def test_four_wheels(self):
        link = self.make_link_array(wheel_placements())
        self.check(link, self.model(wheel_placements()))

    def test_hidden_elements(self):
        # ShowElement = False: no element objects, transforms in PlacementList.
        link = self.make_link_array(wheel_placements(), show_element=False)
        self.check(link, self.model(wheel_placements()))

    def test_array_placement(self):
        link = self.make_link_array(wheel_placements(), placement=ARRAY_PLACEMENT)
        self.check(link, self.model(wheel_placements(), placement=ARRAY_PLACEMENT))

    def test_ignore_placement(self):
        link = self.make_link_array(wheel_placements(), placement=ARRAY_PLACEMENT)
        actual = ic.calculate(link, ignore_placement=True)
        self.assertInertia6Equal(actual, self.model(wheel_placements()), rtol=1e-4, atol=1e-6)

    def test_element_scales(self):
        # Wider rear wheels: element scale 1.5 along the cylinder axis (local
        # Z of the wheel, the scale acts before the element placement).
        scales = [fc.Vector(1.0, 1.0, 1.0)] * 2 + [fc.Vector(1.0, 1.0, 1.5)] * 2
        for show_element in (True, False):
            with self.subTest(show_element=show_element):
                link = self.make_link_array(wheel_placements(), scales=scales, show_element=show_element)
                self.check(link, self.model(wheel_placements(), scales=scales), compare_shape=False)
                self.doc.removeObject(link.Name)

    def test_link_scale_applies_to_the_whole_array(self):
        link = self.make_link_array(wheel_placements(), placement=ARRAY_PLACEMENT)
        link.ScaleVector = fc.Vector(0.5, 0.5, 0.5)
        self.recompute()
        expected = self.model(wheel_placements(), placement=ARRAY_PLACEMENT, link_scale=(0.5, 0.5, 0.5))
        self.check(link, expected, compare_shape=False)

    def test_mass_override_is_distributed_over_the_wheels(self):
        from .test_overrides import add_mass, add_override_mode
        link = self.make_link_array(wheel_placements())
        add_override_mode(link, ic.OVERRIDE_MODE_MASS)
        add_mass(link, 8.0)
        density = 8.0 / (COUNT * math.pi * RADIUS ** 2 * WIDTH)
        actual = ic.calculate(link)
        self.assertInertia6Equal(actual, self.model(wheel_placements(), density=density), rtol=1e-4, atol=1e-6)

    def test_mobile_base(self):
        # Real element of a "base_link": a Part with the chassis plate and the
        # link array of the wheels.
        chassis = shapes.make_part_box(self.doc, 'Chassis', *CHASSIS, placement=CHASSIS_PLACEMENT)
        materials.assign_material(chassis, materials.make_material('Aluminium', 2700.0))
        link = self.make_link_array(wheel_placements())
        base = shapes.make_part_container(self.doc, 'Base', [chassis, link])
        chassis_mass = 2700.0 * materials.KG_PER_M3_TO_KG_PER_MM3 * CHASSIS[0] * CHASSIS[1] * CHASSIS[2]
        chassis_com = fc.Vector(CHASSIS[0] / 2.0, CHASSIS[1] / 2.0, CHASSIS[2] / 2.0)
        expected = analytic.sum_inertia6d(
            analytic.transform_inertia6d(analytic.box_inertia_com(chassis_mass, *CHASSIS), chassis_com, chassis_mass, CHASSIS_PLACEMENT),
            self.model(wheel_placements()),
        )
        actual = ic.calculate(base)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_link_transform(self):
        # LinkTransform = True: x = Link.Placement * P_i * B * x_wheel, the
        # order FreeCAD displays (Part.getShape() differs, see the module
        # docstring).
        link = self.make_link_array(translations(), link_transform=True, placement=ARRAY_PLACEMENT)
        expected = self.model(translations(), placement=ARRAY_PLACEMENT, base=WHEEL_PLACEMENT)
        self.check(link, expected, compare_shape=False)

    def test_link_transform_hidden_elements_with_scales(self):
        # Wider rear wheels: with LinkTransform the wheel axis is already along
        # Y in the element frame.
        scales = [fc.Vector(1.0, 1.0, 1.0)] * 2 + [fc.Vector(1.0, 1.5, 1.0)] * 2
        link = self.make_link_array(translations(), scales=scales, link_transform=True, show_element=False, placement=ARRAY_PLACEMENT)
        expected = self.model(translations(), scales=scales, placement=ARRAY_PLACEMENT, base=WHEEL_PLACEMENT)
        self.check(link, expected, compare_shape=False)

if __name__ == '__main__':  # pragma: no cover
    unittest.main()
