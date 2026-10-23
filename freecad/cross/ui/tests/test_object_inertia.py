"""Single-object calculation used by the "Calculate object mass and inertia" command.

Covers `transform_to_frame` and its use with `calculate(..., ignore_placement=True)`
for an object in the local and in the global frame, the global placement of an
object inside containers as returned by `getSubObject(subname, 3)` and
`getSubObject(subname, 2)` (object and matrix, which the command relies on),
and the text output of the command module (`format_inertial_properties`;
skipped when the GUI is not available, see `commands.py`).
"""
from __future__ import annotations

import numpy as np

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .base import InertiaCalculatorTestCase
from .commands import import_command

DENSITY_KG_PER_M3 = 7850.0
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3
LENGTH, WIDTH, HEIGHT = 100.0, 40.0, 25.0

BOX_PLACEMENT = fc.Placement(fc.Vector(500.0, -200.0, 70.0), fc.Rotation(fc.Vector(0, 1, 0), 33.0))
CONTAINER_PLACEMENT = fc.Placement(fc.Vector(-30.0, 80.0, 10.0), fc.Rotation(fc.Vector(1, 1, 0), 42.0))


def _box_local_mass_com_inertia():
    mass = DENSITY_KG_PER_MM3 * (LENGTH * WIDTH * HEIGHT)
    com = fc.Vector(LENGTH / 2.0, WIDTH / 2.0, HEIGHT / 2.0)
    return mass, com, analytic.box_inertia_com(mass, LENGTH, WIDTH, HEIGHT)


class TransformToFrameTests(InertiaCalculatorTestCase):

    def test_matches_analytic_transform(self):
        placement = fc.Placement(fc.Vector(3, -4, 7), fc.Rotation(fc.Vector(1, 1, 0), 37.0))
        j3 = np.diag([1.0, 2.0, 3.0])
        com = fc.Vector(1.0, -2.0, 0.5)
        mass = 4.0
        actual = ic.transform_to_frame(analytic.generalized_inertia6d(j3, com, mass), placement)
        expected = analytic.transform_inertia6d(j3, com, mass, placement)
        np.testing.assert_allclose(actual, expected, atol=1e-6)

    def test_identity(self):
        JJ = analytic.generalized_inertia6d(np.diag([1.0, 2.0, 3.0]), fc.Vector(1, 2, 3), 2.0)
        np.testing.assert_allclose(ic.transform_to_frame(JJ, fc.Placement()), JJ, atol=1e-12)


class ObjectFrameTests(InertiaCalculatorTestCase):
    """Local and global frame results from `calculate()` and `transform_to_frame()`."""

    def _make_box(self, placement=BOX_PLACEMENT, name='Box'):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, name, LENGTH, WIDTH, HEIGHT, placement=placement)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))
        return box

    def test_local_frame_ignores_object_placement(self):
        box = self._make_box()
        mass, local_com, local_j = _box_local_mass_com_inertia()
        expected = analytic.generalized_inertia6d(local_j, local_com, mass)
        actual = ic.calculate(box, ignore_placement=True)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_global_frame_applies_global_placement(self):
        box = self._make_box()
        global_placement = CONTAINER_PLACEMENT * BOX_PLACEMENT
        mass, local_com, local_j = _box_local_mass_com_inertia()
        expected = analytic.transform_inertia6d(local_j, local_com, mass, global_placement)
        actual = ic.transform_to_frame(ic.calculate(box, ignore_placement=True), global_placement)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_global_frame_of_top_level_object_is_the_document_result(self):
        box = self._make_box()
        actual = ic.transform_to_frame(ic.calculate(box, ignore_placement=True), box.Placement)
        self.assertInertia6Equal(actual, ic.calculate(box), rtol=1e-4, atol=1e-6)

    def test_container_in_global_frame(self):
        box = self._make_box(placement=fc.Placement(fc.Vector(10, 0, 0), fc.Rotation()))
        container = shapes.make_part_container(self.doc, 'Container', [box], placement=CONTAINER_PLACEMENT)
        mass, local_com, local_j = _box_local_mass_com_inertia()
        # Container local frame: the box keeps its placement relative to the container.
        box_in_container = analytic.transform_inertia6d(local_j, local_com, mass, box.Placement)
        _, com_c, j_c = self.split_inertia6d(box_in_container)
        expected = analytic.transform_inertia6d(j_c, com_c, mass, container.Placement)
        actual = ic.transform_to_frame(ic.calculate(container, ignore_placement=True), container.Placement)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)


class CommandCalculateObjectTests(InertiaCalculatorTestCase):
    """Functions defined in `command_calculate_object_mass_and_inertia.py`."""

    def setUp(self) -> None:
        super().setUp()
        self.command = import_command('command_calculate_object_mass_and_inertia')

    def test_text_contains_values_and_units(self):
        text = self.command.format_inertial_properties(
            0.785, fc.Vector(50.0, 20.0, 12.5), np.diag([1.0e3, 2.0e3, 3.0e3]), header='Header line',
        )
        lines = text.splitlines()
        self.assertEqual(lines[0], 'Header line')
        self.assertIn('Mass: 0.785 kg', text)
        self.assertIn('x = 50 mm', text)
        self.assertIn('y = 20 mm', text)
        self.assertIn('z = 12.5 mm', text)
        self.assertIn('kg*mm^2', text)
        self.assertIn('1.000000e+03', text)   # Ixx in kg*mm^2.

    def test_text_without_header(self):
        text = self.command.format_inertial_properties(1.0, fc.Vector(), np.eye(3))
        self.assertTrue(text.startswith('  Mass: 1 kg'))


class SubObjectPlacementTests(InertiaCalculatorTestCase):
    """Global placement of a nested object from `root.getSubObject(subname, 3)`.

    The command uses `getSubObject(subname, 2)`, whose matrix is the same
    (see `SubObjectTypeTests`).
    """

    def test_sub_object_placement_includes_container_and_own_placement(self):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT, placement=BOX_PLACEMENT)
        container = shapes.make_part_container(self.doc, 'Container', [box], placement=CONTAINER_PLACEMENT)
        placement = container.getSubObject('Box.', 3)
        expected = CONTAINER_PLACEMENT * BOX_PLACEMENT
        self.assertIsInstance(placement, fc.Placement)
        self.assertVectorAlmostEqual(placement.Base, expected.Base, places=6)
        np.testing.assert_allclose(
            analytic.rotation_matrix(placement.Rotation),
            analytic.rotation_matrix(expected.Rotation),
            atol=1e-9,
        )


class SubObjectTypeTests(InertiaCalculatorTestCase):
    """What `getSubObject(subname, retType)` returns, as used by the command."""

    def setUp(self) -> None:
        super().setUp()
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT)
        self.container = shapes.make_part_container(self.doc, 'Container', [box])

    def test_ret_type_2_returns_object_and_matrix(self):
        # `getSubObject(subname, 2)` returns (object, matrix, geometry), the
        # matrix is the one of `getSubObject(subname, 3)`; with a sub-element at
        # the end of the path the object is the owning object.
        for subname in ('Box.', 'Box.Face6', 'Box.Edge3', 'Box.Solid1'):
            with self.subTest(subname=subname):
                obj, matrix, _ = self.container.getSubObject(subname, 2)
                self.assertIsInstance(obj, fc.DocumentObject)
                self.assertEqual(obj.Name, 'Box')
                self.assertIsInstance(matrix, fc.Matrix)
                expected = self.container.getSubObject('Box.', 3)
                self.assertVectorAlmostEqual(fc.Placement(matrix).Base, expected.Base, places=6)


class SelectedObjectTests(InertiaCalculatorTestCase):
    """`_selected_object_and_global_placement()` with a real selection (needs the GUI)."""

    def setUp(self) -> None:
        super().setUp()
        self.command = import_command('command_calculate_object_mass_and_inertia')
        if not fc.GuiUp:
            self.skipTest('The GUI is required to select objects.')
        import FreeCADGui as fcgui
        self.fcgui = fcgui
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT, placement=BOX_PLACEMENT)
        self.container = shapes.make_part_container(self.doc, 'Container', [box], placement=CONTAINER_PLACEMENT)
        self.addCleanup(fcgui.Selection.clearSelection)

    def _select(self, subname: str) -> None:
        self.fcgui.Selection.clearSelection()
        self.fcgui.Selection.addSelection(self.doc.Name, 'Container', subname)

    def _assert_placement(self, placement, expected) -> None:
        self.assertIsInstance(placement, fc.Placement)
        self.assertVectorAlmostEqual(placement.Base, expected.Base, places=6)
        np.testing.assert_allclose(
            analytic.rotation_matrix(placement.Rotation),
            analytic.rotation_matrix(expected.Rotation),
            atol=1e-9,
        )

    def test_nested_object_and_its_face(self):
        expected = CONTAINER_PLACEMENT * BOX_PLACEMENT
        for subname in ('Box.', 'Box.Face6'):
            with self.subTest(subname=subname):
                self._select(subname)
                obj, placement = self.command._selected_object_and_global_placement()
                self.assertIsNotNone(obj)
                self.assertEqual(obj.Name, 'Box')
                self._assert_placement(placement, expected)

    def test_top_level_object(self):
        self.fcgui.Selection.clearSelection()
        self.fcgui.Selection.addSelection(self.doc.Name, 'Container')
        obj, placement = self.command._selected_object_and_global_placement()
        self.assertEqual(obj.Name, 'Container')
        self._assert_placement(placement, CONTAINER_PLACEMENT)

    def test_nothing_selected(self):
        self.fcgui.Selection.clearSelection()
        self.assertEqual(self.command._selected_object_and_global_placement(), (None, None))
