"""Draft arrays: link arrays (with scales, expanded or not) and regular arrays (fused or not).

The base object is a steel `Part::Box` with a non-identity Placement. The
arrays are built with `Draft.make_ortho_array()`.

References:

* Regular arrays: FreeCAD's own geometry, `Part.getShape(array)` (a compound
  of copies of the base).
* Link arrays: an analytic model built from the link properties of the
  array after recompute, with FreeCAD's link semantics

      x = Placement * diag(ScaleVector) * P_i * diag(S_i) * B * x_base

  where `P_i`, `S_i` are the element placement and scale (`PlacementList` /
  `ScaleList`, or the `Placement` / `ScaleVector` of the elements of
  `ElementList` when `ExpandArray` is set: the elements are link-like
  objects whose (uniform) scale, observed in the tests, is the x component
  of their former `ScaleList` entry) and `B` is the Placement of the
  base if `LinkTransform` is set, identity otherwise. Without scales the
  model is first checked against `Part.getShape(array)`. With scales that
  check is skipped: `Part.getShape()` of a Draft link array ignores
  `ScaleList` and `ScaleVector` (its mass was the unscaled one; probably the
  compound Shape that Draft builds from the placements only, not
  confirmed).

Each test also checks the mass of the reference against the analytic value
(number of elements times the mass of the base, times the scale factors):
a failure there means that FreeCAD did not build the array as the test
assumes (e.g. a property reset by Draft on recompute), not that
`calculate()` is wrong.

Materials: link arrays and regular arrays that are not fused are processed
through their base object, with the material of the base; a material
assigned to the array is only inherited by a base without material.
Overlapping elements are counted several times. A fused regular array
(`Fuse = True`) is processed as one solid, its `Shape` (overlaps counted
once), with the material of the array (or the inherited one): the material
of the base is not used.
"""
from __future__ import annotations

import unittest

import numpy as np

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase

DENSITY_KG_PER_M3 = 7850.0
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3
LENGTH, WIDTH, HEIGHT = 100.0, 40.0, 25.0
BASE_MASS = DENSITY_KG_PER_MM3 * LENGTH * WIDTH * HEIGHT
LEAD_DENSITY_KG_PER_M3 = 11340.0
LEAD_DENSITY_KG_PER_MM3 = LEAD_DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3
LEAD_TO_STEEL = LEAD_DENSITY_KG_PER_M3 / DENSITY_KG_PER_M3

BOX_PLACEMENT = fc.Placement(fc.Vector(20.0, -10.0, 5.0), fc.Rotation(fc.Vector(0, 0, 1), 30.0))
ARRAY_PLACEMENT = fc.Placement(fc.Vector(-30.0, 40.0, 15.0), fc.Rotation(fc.Vector(1, 0, 0), 20.0))
N_X, N_Y = 3, 2
COUNT = N_X * N_Y


class DraftArrayTestsBase(InertiaCalculatorTestCase):

    def setUp(self) -> None:
        super().setUp()
        materials.require_materials_module()
        try:
            import Draft
            import Part
        except ImportError as e:  # pragma: no cover - depends on the FreeCAD build.
            raise unittest.SkipTest(f'Draft workbench not available: {e}')
        self.Draft = Draft
        self.Part = Part
        self.base = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT, placement=BOX_PLACEMENT)
        materials.assign_material(self.base, materials.make_material('Steel', DENSITY_KG_PER_M3))

    def make_array(self, use_link: bool, base=None):
        array = self.Draft.make_ortho_array(
            self.base if base is None else base,
            v_x=fc.Vector(150.0, 0.0, 0.0),
            v_y=fc.Vector(0.0, 80.0, 0.0),
            v_z=fc.Vector(0.0, 0.0, 50.0),
            n_x=N_X,
            n_y=N_Y,
            n_z=1,
            use_link=use_link,
        )
        self.recompute()
        return array

    def reference(self, array, expected_mass: float, density: float = DENSITY_KG_PER_MM3) -> np.ndarray:
        """Reference matrix from FreeCAD's shape of the array, after a mass sanity check."""
        JJ = shapes.shape_inertia6d(self.Part.getShape(array), density)
        self.assertAlmostEqual(
            JJ[3, 3], expected_mass, delta=1e-6 * expected_mass,
            msg='Test assumption failed: FreeCAD shape of the array does not have the expected mass.',
        )
        return JJ

    def check(self, array, expected_mass: float, density: float = DENSITY_KG_PER_MM3) -> None:
        expected = self.reference(array, expected_mass, density)
        actual = ic.calculate(array)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class DraftLinkArrayTestsBase(DraftArrayTestsBase):
    """Helpers for link arrays: analytic model and check."""

    @staticmethod
    def element_transforms(array):
        """(placement, scale) of each element, as stored in the array."""
        if array.ExpandArray:
            elements = []
            for element in array.ElementList:
                scale = getattr(element, 'ScaleVector', fc.Vector(1.0, 1.0, 1.0))
                elements.append((element.Placement, (scale.x, scale.y, scale.z)))
            return elements
        scales = list(array.ScaleList) or [fc.Vector(1.0, 1.0, 1.0)] * len(array.PlacementList)
        return [(placement, (scale.x, scale.y, scale.z)) for placement, scale in zip(array.PlacementList, scales)]

    def model(self, array, density: float = DENSITY_KG_PER_MM3) -> np.ndarray:
        """Analytic reference of a link array (see the module docstring)."""
        mass = density * LENGTH * WIDTH * HEIGHT
        com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
        inertia = analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)
        base_map = analytic.affine(array.Base.Placement if array.LinkTransform else None)
        global_scale = array.ScaleVector
        array_map = analytic.affine(array.Placement, (global_scale.x, global_scale.y, global_scale.z))
        JJ = np.zeros((6, 6))
        for placement, scale in self.element_transforms(array):
            element_map = analytic.compose_affine(array_map, analytic.affine(placement, scale), base_map)
            JJ += analytic.affine_transform_inertia6d(inertia, com, mass, element_map)
        return JJ

    def check_link(
            self, array, expected_mass: float, compare_shape: bool = True,
            density: float = DENSITY_KG_PER_MM3,
    ) -> None:
        expected = self.model(array, density)
        self.assertAlmostEqual(
            expected[3, 3], expected_mass, delta=1e-6 * expected_mass,
            msg='Test assumption failed: the properties of the array after recompute are not the ones set by the test.',
        )
        if compare_shape:
            shape_reference = shapes.shape_inertia6d(self.Part.getShape(array), density)
            self.assertInertia6Equal(
                shape_reference, expected, rtol=1e-4, atol=1e-6,
                msg='Test assumption failed: the analytic link-array model differs from FreeCAD shape of the array.',
            )
        actual = ic.calculate(array)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class DraftLinkArrayTests(DraftLinkArrayTestsBase):

    def test_link_array(self):
        array = self.make_array(use_link=True)
        self.check_link(array, COUNT * BASE_MASS)

    def test_link_array_with_placement(self):
        array = self.make_array(use_link=True)
        array.Placement = ARRAY_PLACEMENT
        self.recompute()
        self.check_link(array, COUNT * BASE_MASS)

    def test_link_array_ignore_placement(self):
        array = self.make_array(use_link=True)
        array.Placement = ARRAY_PLACEMENT
        self.recompute()
        # Reference: the array shape brought back to the local frame of the array.
        shape = self.Part.getShape(array).copy()
        shape.transformShape(ARRAY_PLACEMENT.inverse().toMatrix())
        expected = shapes.shape_inertia6d(shape, DENSITY_KG_PER_MM3)
        actual = ic.calculate(array, ignore_placement=True)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_link_array_with_element_scales(self):
        array = self.make_array(use_link=True)
        scales = [fc.Vector(1.0 + 0.25 * i, 1.0, 1.0 - 0.1 * i) for i in range(COUNT)]
        array.ScaleList = scales
        self.recompute()
        expected_mass = BASE_MASS * sum(abs(s.x * s.y * s.z) for s in scales)
        self.check_link(array, expected_mass, compare_shape=False)

    def test_link_array_with_global_scale(self):
        array = self.make_array(use_link=True)
        array.ScaleVector = fc.Vector(1.5, 1.0, 0.5)
        self.recompute()
        self.check_link(array, COUNT * BASE_MASS * 1.5 * 1.0 * 0.5, compare_shape=False)

    def test_link_array_with_element_and_global_scales(self):
        array = self.make_array(use_link=True)
        scales = [fc.Vector(1.0 + 0.25 * i, 1.0, 1.0 - 0.1 * i) for i in range(COUNT)]
        array.ScaleList = scales
        array.ScaleVector = fc.Vector(1.5, 1.0, 0.5)
        array.Placement = ARRAY_PLACEMENT
        self.recompute()
        expected_mass = BASE_MASS * 0.75 * sum(abs(s.x * s.y * s.z) for s in scales)
        self.check_link(array, expected_mass, compare_shape=False)

    def test_expanded_link_array(self):
        array = self.make_array(use_link=True)
        array.ExpandArray = True
        array.Placement = ARRAY_PLACEMENT
        self.recompute()
        self.check_link(array, COUNT * BASE_MASS)

    def test_expanded_link_array_with_global_scale(self):
        array = self.make_array(use_link=True)
        array.ScaleVector = fc.Vector(1.5, 1.0, 0.5)
        array.Placement = ARRAY_PLACEMENT
        array.ExpandArray = True
        self.recompute()
        self.check_link(array, COUNT * BASE_MASS * 0.75, compare_shape=False)

    def test_expanded_link_array_keeps_uniform_element_scales(self):
        # Observed in FreeCAD: on expansion an element keeps a uniform scale
        # equal to the x component of its `ScaleList` entry (a non-uniform
        # entry (1 + 0.25 i, 1, 1 - 0.1 i) gave an element scale of
        # (1 + 0.25 i) on every axis). Uniform entries are used here.
        array = self.make_array(use_link=True)
        factors = [1.0 + 0.25 * i for i in range(COUNT)]
        array.ScaleList = [fc.Vector(f, f, f) for f in factors]
        array.ScaleVector = fc.Vector(1.5, 1.0, 0.5)
        array.Placement = ARRAY_PLACEMENT
        array.ExpandArray = True
        self.recompute()
        self.check_link(array, BASE_MASS * 0.75 * sum(f ** 3 for f in factors), compare_shape=False)

    def test_expanded_and_collapsed_arrays_agree(self):
        array = self.make_array(use_link=True)
        array.Placement = ARRAY_PLACEMENT
        self.recompute()
        collapsed = ic.calculate(array)
        array.ExpandArray = True
        self.recompute()
        expanded = ic.calculate(array)
        self.assertInertia6Equal(expanded, collapsed, rtol=1e-6, atol=1e-6)

    def test_expanded_and_collapsed_arrays_agree_with_scales(self):
        array = self.make_array(use_link=True)
        factors = [1.0 + 0.25 * i for i in range(COUNT)]
        array.ScaleList = [fc.Vector(f, f, f) for f in factors]
        array.ScaleVector = fc.Vector(1.5, 1.0, 0.5)
        array.Placement = ARRAY_PLACEMENT
        self.recompute()
        collapsed = ic.calculate(array)
        array.ExpandArray = True
        self.recompute()
        expanded = ic.calculate(array)
        self.assertInertia6Equal(expanded, collapsed, rtol=1e-6, atol=1e-6)


class DraftLinkArrayMaterialTests(DraftLinkArrayTestsBase):
    """A material assigned to a link array is inherited by a base without material only."""

    def test_base_material_wins_over_array_material(self):
        array = self.make_array(use_link=True)
        materials.assign_material(array, materials.make_material('Lead', LEAD_DENSITY_KG_PER_M3))
        array.Placement = ARRAY_PLACEMENT
        self.recompute()
        self.check_link(array, COUNT * BASE_MASS)

    def test_array_material_is_used_for_base_without_material(self):
        plain = shapes.make_part_box(self.doc, 'PlainBox', LENGTH, WIDTH, HEIGHT, placement=BOX_PLACEMENT)
        array = self.make_array(use_link=True, base=plain)
        materials.assign_material(array, materials.make_material('Lead', LEAD_DENSITY_KG_PER_M3))
        scales = [fc.Vector(1.0 + 0.25 * i, 1.0, 1.0 - 0.1 * i) for i in range(COUNT)]
        array.ScaleList = scales
        array.ScaleVector = fc.Vector(1.5, 1.0, 0.5)
        self.recompute()
        expected_mass = BASE_MASS * LEAD_TO_STEEL * 0.75 * sum(abs(s.x * s.y * s.z) for s in scales)
        self.check_link(array, expected_mass, compare_shape=False, density=LEAD_DENSITY_KG_PER_MM3)

    def test_expanded_array_base_material_wins(self):
        array = self.make_array(use_link=True)
        materials.assign_material(array, materials.make_material('Lead', LEAD_DENSITY_KG_PER_M3))
        array.ExpandArray = True
        self.recompute()
        self.check_link(array, COUNT * BASE_MASS)


class DraftArrayTests(DraftArrayTestsBase):
    """Regular (non-link), not fused Draft array: `Base` and `PlacementList`, no scale properties.

    `calculate()` processes it like a link array without scales: the base
    object placed by each entry of `PlacementList`, with the material of the
    base (a material on the array is only inherited by a base without one).
    """

    def test_array(self):
        array = self.make_array(use_link=False)
        self.check(array, COUNT * BASE_MASS)

    def test_array_with_placement(self):
        array = self.make_array(use_link=False)
        array.Placement = ARRAY_PLACEMENT
        self.recompute()
        self.check(array, COUNT * BASE_MASS)

    def test_base_material_wins_over_array_material(self):
        array = self.make_array(use_link=False)
        materials.assign_material(array, materials.make_material('Lead', LEAD_DENSITY_KG_PER_M3))
        array.Placement = ARRAY_PLACEMENT
        self.recompute()
        self.check(array, COUNT * BASE_MASS)

    def test_array_material_is_used_for_base_without_material(self):
        plain = shapes.make_part_box(self.doc, 'PlainBox', LENGTH, WIDTH, HEIGHT, placement=BOX_PLACEMENT)
        array = self.make_array(use_link=False, base=plain)
        materials.assign_material(array, materials.make_material('Steel', DENSITY_KG_PER_M3))
        self.recompute()
        self.check(array, COUNT * BASE_MASS)

    def test_array_in_volume_mode_ignores_array_material(self):
        # In volume mode (here: a link with a 'density' override pointing to
        # the array) the array material is not used either. A link is used
        # instead of an App::Part: adding the array to a Part also moves its
        # (hidden) base into the Part, where it would be counted as a child.
        from .test_overrides import add_density, add_override_mode
        array = self.make_array(use_link=False)
        materials.assign_material(array, materials.make_material('Lead', LEAD_DENSITY_KG_PER_M3))
        link = shapes.make_link(self.doc, 'Link', array, link_transform=True)
        add_override_mode(link, ic.OVERRIDE_MODE_DENSITY)
        add_density(link, DENSITY_KG_PER_M3)
        self.recompute()
        self.check(link, COUNT * BASE_MASS)


class OverlappingDraftArrayTests(DraftArrayTestsBase):
    """Regular Draft array whose copies overlap (x spacing 60 mm < box length 100 mm).

    Not fused: every copy is counted (overlaps counted several times).
    Fused (`Fuse = True`): the array is one solid, its `Shape` is used with
    the material of the array; the material of the base is not used.
    """

    def make_overlapping_array(self, fuse: bool):
        array = self.Draft.make_ortho_array(
            self.base,
            v_x=fc.Vector(60.0, 0.0, 0.0),
            v_y=fc.Vector(0.0, 80.0, 0.0),
            v_z=fc.Vector(0.0, 0.0, 50.0),
            n_x=N_X,
            n_y=N_Y,
            n_z=1,
            use_link=False,
        )
        array.Fuse = fuse
        array.Placement = ARRAY_PLACEMENT
        self.recompute()
        return array

    def test_overlaps_are_counted_twice_without_fuse(self):
        array = self.make_overlapping_array(fuse=False)
        actual = ic.calculate(array)
        self.assertAlmostEqual(actual[3, 3], COUNT * BASE_MASS, delta=1e-6 * COUNT * BASE_MASS)

    def test_fused_array_is_one_solid_with_the_array_material(self):
        array = self.make_overlapping_array(fuse=True)
        materials.assign_material(array, materials.make_material('Lead', LEAD_DENSITY_KG_PER_M3))
        self.recompute()
        shape = array.Shape
        self.assertLess(len(shape.Solids), COUNT, 'Test assumption failed: the copies are not fused.')
        self.assertLess(
            shape.Volume, COUNT * LENGTH * WIDTH * HEIGHT,
            'Test assumption failed: the copies of the array do not overlap.',
        )
        expected = shapes.shape_inertia6d(shape, LEAD_DENSITY_KG_PER_MM3)
        actual = ic.calculate(array)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_fused_array_without_material_has_zero_mass(self):
        # The base material (steel) is not used for a fused array.
        array = self.make_overlapping_array(fuse=True)
        actual = ic.calculate(array, output_level=ic.LOG_LEVEL_WARN)
        self.assertEqual(actual[3, 3], 0.0)
