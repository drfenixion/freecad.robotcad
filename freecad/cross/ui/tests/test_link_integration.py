"""Link-level helpers used by the "Calculate mass and inertia" command.

Covers `mass_com_inertia` (inverse of `inertia_matrix6d`),
`calculate(..., ignore_placement=True)` as used for a link's Real element
(calculated in its local frame), `center_of_mass_placement_in_link_frame` (defined in
`command_calculate_mass_and_inertia.py`; the convention shared with the "basic"
method, i.e. the URDF `<inertial>` tag), the log helper and the enumeration
constants shared with the Robot properties.

Convention (same as the "basic" method): the inertia tensor stays in the axes
of the Real element's local frame; the link `CenterOfMass` placement has the
center of mass in the link frame as Base (`MountedPlacement * cog`) and
`MountedPlacement.Rotation` as Rotation, i.e. the orientation of the inertia
axes in the link frame. `MountedPlacement` is the transform from the element's
local frame to the link frame (the URDF export writes a visual at
`MountedPlacement * element placement`).

`center_of_mass_placement_in_link_frame` lives in a command module, so its tests are
skipped when the GUI is not available (see `commands.py`).
"""
from __future__ import annotations

from unittest import mock

import numpy as np

import FreeCAD as fc

from ... import wb_constants
from .. import inertia_calculator as ic
from . import analytic, materials, shapes
from .commands import import_command
from .base import InertiaCalculatorTestCase

DENSITY_KG_PER_M3 = 7850.0
DENSITY_KG_PER_MM3 = DENSITY_KG_PER_M3 * materials.KG_PER_M3_TO_KG_PER_MM3
LENGTH, WIDTH, HEIGHT = 100.0, 40.0, 25.0

# Element placement in the document (must be ignored) and element-to-link
# placement (must be applied).
ELEMENT_PLACEMENT = fc.Placement(fc.Vector(500.0, -200.0, 70.0), fc.Rotation(fc.Vector(0, 1, 0), 33.0))
MOUNTED_PLACEMENT = fc.Placement(fc.Vector(12.0, 34.0, -56.0), fc.Rotation(fc.Vector(1, 1, 0), 42.0))


def _box_local_mass_com_inertia(length=LENGTH, width=WIDTH, height=HEIGHT):
    mass = DENSITY_KG_PER_MM3 * (length * width * height)
    com = fc.Vector(length / 2.0, width / 2.0, height / 2.0)
    return mass, com, analytic.box_inertia_com(mass, length, width, height)


class MassComInertiaTests(InertiaCalculatorTestCase):

    def test_round_trip_with_analytic_assembly(self):
        j3 = np.array([[2.0, 0.1, 0.2], [0.1, 3.0, 0.3], [0.2, 0.3, 4.0]])
        com = fc.Vector(1.5, -2.5, 0.5)
        mass = 6.0
        JJ = analytic.generalized_inertia6d(j3, com, mass)
        actual_mass, actual_com, actual_j3 = ic.mass_com_inertia(JJ)
        self.assertAlmostEqual(actual_mass, mass, places=9)
        self.assertVectorAlmostEqual(actual_com, com, places=9)
        np.testing.assert_allclose(actual_j3, j3, atol=1e-9)

    def test_matches_test_helper_split(self):
        j3 = np.diag([1.0, 2.0, 3.0])
        JJ = analytic.generalized_inertia6d(j3, fc.Vector(3.0, -4.0, 5.0), 2.0)
        mass, com, inertia3 = ic.mass_com_inertia(JJ)
        ref_mass, ref_com, ref_inertia3 = self.split_inertia6d(JJ)
        self.assertAlmostEqual(mass, ref_mass, places=9)
        self.assertVectorAlmostEqual(com, ref_com, places=9)
        np.testing.assert_allclose(inertia3, ref_inertia3, atol=1e-9)

    def test_zero_matrix(self):
        mass, com, inertia3 = ic.mass_com_inertia(np.zeros((6, 6)))
        self.assertEqual(mass, 0.0)
        self.assertVectorAlmostEqual(com, fc.Vector(0, 0, 0))
        np.testing.assert_allclose(inertia3, np.zeros((3, 3)))

    def test_inverse_of_inertia_matrix6d(self):
        j3 = np.array([[2.0, 0.1, 0.2], [0.1, 3.0, 0.3], [0.2, 0.3, 4.0]])
        J = fc.Matrix(
            j3[0, 0], j3[0, 1], j3[0, 2], 0.0,
            j3[1, 0], j3[1, 1], j3[1, 2], 0.0,
            j3[2, 0], j3[2, 1], j3[2, 2], 0.0,
            0.0, 0.0, 0.0, 1.0,
        )
        com = fc.Vector(-7.0, 2.0, 11.0)
        JJ = ic.inertia_matrix6d(J, com, 4.5)
        mass, actual_com, actual_j3 = ic.mass_com_inertia(JJ)
        self.assertAlmostEqual(mass, 4.5, places=9)
        self.assertVectorAlmostEqual(actual_com, com, places=9)
        np.testing.assert_allclose(actual_j3, j3, atol=1e-9)


class LinkRealElementTests(InertiaCalculatorTestCase):
    """The Real element of a link is calculated in its local frame."""

    def _make_box(self, name='Box', placement=ELEMENT_PLACEMENT):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, name, LENGTH, WIDTH, HEIGHT, placement=placement)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))
        return box

    def test_result_is_in_element_local_frame(self):
        # The element placement is ignored; nothing else is applied.
        box = self._make_box()
        mass, local_com, local_j = _box_local_mass_com_inertia()
        expected = analytic.generalized_inertia6d(local_j, local_com, mass)
        actual = ic.calculate(box, ignore_placement=True)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_result_does_not_depend_on_element_placement(self):
        box_a = self._make_box('BoxA', placement=fc.Placement())
        box_b = self._make_box('BoxB', placement=ELEMENT_PLACEMENT)
        result_a = ic.calculate(box_a, ignore_placement=True)
        result_b = ic.calculate(box_b, ignore_placement=True)
        self.assertInertia6Equal(result_a, result_b, rtol=1e-9, atol=1e-9)

    def test_compound_link_as_part_container(self):
        # A compound link is an App::Part: children keep their placement
        # relative to the container, the container placement is ignored.
        materials.require_materials_module()
        box_a = self._make_box('BoxA', placement=fc.Placement())
        box_b = self._make_box('BoxB', placement=fc.Placement(fc.Vector(200.0, 0.0, 0.0), fc.Rotation()))
        container = shapes.make_part_container(self.doc, 'Container', [box_a, box_b], placement=ELEMENT_PLACEMENT)
        mass, local_com, local_j = _box_local_mass_com_inertia()
        expected = (
            analytic.generalized_inertia6d(local_j, local_com, mass)
            + analytic.generalized_inertia6d(local_j, local_com + fc.Vector(200.0, 0.0, 0.0), mass)
        )
        actual = ic.calculate(container, ignore_placement=True)
        self.assertInertia6Equal(actual, expected, rtol=1e-4, atol=1e-6)

    def test_extraction_gives_local_com_and_tensor(self):
        box = self._make_box()
        mass, local_com, local_j = _box_local_mass_com_inertia()
        actual_mass, actual_com, actual_j = ic.mass_com_inertia(ic.calculate(box, ignore_placement=True))
        self.assertAlmostEqual(actual_mass, mass, delta=mass * 1e-4)
        self.assertVectorAlmostEqual(actual_com, local_com, places=3)
        np.testing.assert_allclose(actual_j, local_j, rtol=1e-4, atol=1e-6 * mass)


class CenterOfMassInLinkFrameTests(InertiaCalculatorTestCase):
    """The link `CenterOfMass` follows the URDF `<inertial>` convention.

    The tensor is in the element axes; `CenterOfMass` says where the center of
    mass is in the link frame and how the tensor axes are oriented in it. A
    URDF reader that applies that rotation to the tensor obtains the tensor
    in link axes, which must equal the physical answer obtained by rotating
    the body into the link frame with `MountedPlacement`.
    """

    def setUp(self) -> None:
        super().setUp()
        command = import_command('command_calculate_mass_and_inertia')
        self.center_of_mass_placement_in_link_frame = command.center_of_mass_placement_in_link_frame

    def test_base_and_rotation(self):
        cog = fc.Vector(50.0, 20.0, 12.5)
        placement = self.center_of_mass_placement_in_link_frame(cog, MOUNTED_PLACEMENT)
        self.assertVectorAlmostEqual(placement.Base, MOUNTED_PLACEMENT.multVec(cog), places=9)
        np.testing.assert_allclose(
            analytic.rotation_matrix(placement.Rotation),
            analytic.rotation_matrix(MOUNTED_PLACEMENT.Rotation),
            atol=1e-12,
        )

    def test_matches_physical_transform_to_link_frame(self):
        materials.require_materials_module()
        box = shapes.make_part_box(self.doc, 'Box', LENGTH, WIDTH, HEIGHT, placement=ELEMENT_PLACEMENT)
        materials.assign_material(box, materials.make_material('Steel', DENSITY_KG_PER_M3))

        mass, local_com, local_j = _box_local_mass_com_inertia()
        # Physical answer: the body moved into the link frame by MountedPlacement.
        expected = analytic.transform_inertia6d(local_j, local_com, mass, MOUNTED_PLACEMENT)
        _, expected_com, expected_j = self.split_inertia6d(expected)

        # What the command writes.
        actual_mass, com, tensor = ic.mass_com_inertia(ic.calculate(box, ignore_placement=True))
        placement = self.center_of_mass_placement_in_link_frame(com, MOUNTED_PLACEMENT)
        R = analytic.rotation_matrix(placement.Rotation)

        self.assertAlmostEqual(actual_mass, mass, delta=mass * 1e-4)
        self.assertVectorAlmostEqual(placement.Base, expected_com, places=3)
        # Tensor as seen in link axes by a URDF reader.
        np.testing.assert_allclose(R @ tensor @ R.T, expected_j, rtol=1e-4, atol=1e-6 * mass)


class LogMessageTests(InertiaCalculatorTestCase):

    def _capture(self, output_level, msg_level, level, message):
        messages, warnings = [], []
        with mock.patch.object(fc.Console, 'PrintMessage', side_effect=messages.append), \
                mock.patch.object(fc.Console, 'PrintWarning', side_effect=warnings.append):
            ic.log_message(output_level, msg_level, level, message)
        return messages, warnings

    def test_info_printed_only_at_info_output_level(self):
        messages, warnings = self._capture(ic.LOG_LEVEL_INFO, ic.LOG_LEVEL_INFO, 0, 'hello')
        self.assertEqual(messages, ['INFO: hello\n'])
        self.assertEqual(warnings, [])
        messages, warnings = self._capture(ic.LOG_LEVEL_WARN, ic.LOG_LEVEL_INFO, 0, 'hello')
        self.assertEqual((messages, warnings), ([], []))

    def test_warning_printed_at_both_output_levels(self):
        for output_level in ic.LOG_LEVELS:
            with self.subTest(output_level=output_level):
                messages, warnings = self._capture(output_level, ic.LOG_LEVEL_WARN, 0, 'careful')
                self.assertEqual(messages, [])
                self.assertEqual(warnings, ['WARN: careful\n'])

    def test_leader_imitates_tree_structure(self):
        # Level 0: no leader. Level 1: branch symbol. Deeper levels: 4 spaces
        # for each level above the first and the branch symbol.
        for level, expected in [
            (0, 'INFO: root\n'),
            (1, f'INFO: {ic.LOG_BRANCH}root\n'),
            (2, f'INFO: {ic.LOG_INDENT}{ic.LOG_BRANCH}root\n'),
            (3, f'INFO: {ic.LOG_INDENT * 2}{ic.LOG_BRANCH}root\n'),
        ]:
            with self.subTest(level=level):
                messages, _ = self._capture(ic.LOG_LEVEL_INFO, ic.LOG_LEVEL_INFO, level, 'root')
                self.assertEqual(messages, [expected])

    def test_leader_literal_values(self):
        self.assertEqual(ic.log_leader(0), '')
        self.assertEqual(ic.log_leader(1), '└─> ')
        self.assertEqual(ic.log_leader(2), '    └─> ')
        self.assertEqual(len(ic.LOG_INDENT), len(ic.LOG_BRANCH))


class ConstantsTests(InertiaCalculatorTestCase):

    def test_defaults_are_first_in_the_enumerations(self):
        # Robot properties use these lists; the first item is the default.
        self.assertEqual(wb_constants.INERTIA_METHODS[0], wb_constants.INERTIA_METHOD_BASIC)
        self.assertEqual(ic.LOG_LEVELS[0], ic.LOG_LEVEL_WARN)
        self.assertEqual(set(ic.LOG_LEVELS), {ic.LOG_LEVEL_WARN, ic.LOG_LEVEL_INFO})
        self.assertEqual(
            set(wb_constants.INERTIA_METHODS),
            {wb_constants.INERTIA_METHOD_BASIC, wb_constants.INERTIA_METHOD_ADVANCED},
        )
