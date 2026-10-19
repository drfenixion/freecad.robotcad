"""Principal axes of inertia in the "Calculate object mass and inertia" window.

With "Principal axes" checked, the command module
(`command_calculate_object_mass_and_inertia.py`) prints the rotation from the
target frame axes to the principal axes, then the inertia matrix in the
principal axes (diagonal: the principal moments). `principal_axes(J)` returns the moments in ascending order
and a Rotation `R` with `J = R @ diag(moments) @ R.T`; the axes form a
right-handed frame and, among the free sign choices, the smallest rotation is
returned (identity for a diagonal matrix with ascending moments).

Also covers `error_summary()` of the "Calculate mass and inertia" command,
the text of the dialog that lists the links the tool could not calculate.

The functions live in command modules, so these tests are skipped without the
FreeCAD GUI (see `commands.py`).
"""
from __future__ import annotations

import numpy as np

import FreeCAD as fc

from . import analytic
from .base import InertiaCalculatorTestCase
from .commands import import_command

INERTIA = np.array([
    [3.0e3, 0.5e3, -0.2e3],
    [0.5e3, 2.0e3, 0.1e3],
    [-0.2e3, 0.1e3, 1.5e3],
])


class PrincipalAxesTests(InertiaCalculatorTestCase):

    def setUp(self) -> None:
        super().setUp()
        self.command = import_command('command_calculate_object_mass_and_inertia')

    def assertPrincipalDecomposition(self, inertia: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        moments, rotation = self.command.principal_axes(inertia)
        R = analytic.rotation_matrix(rotation)
        # Same physical inertia.
        np.testing.assert_allclose(R @ np.diag(moments) @ R.T, inertia, atol=1e-9 * np.abs(inertia).max())
        # Eigenvalues in ascending order.
        np.testing.assert_allclose(moments, np.linalg.eigvalsh(inertia), atol=1e-9 * np.abs(inertia).max())
        # A proper rotation (right-handed axes).
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=9)
        return moments, R

    def test_general_matrix(self):
        self.assertPrincipalDecomposition(INERTIA)

    def test_rotated_diagonal_matrix(self):
        rotation = fc.Rotation(fc.Vector(1, 2, 3), 50.0)
        Rg = analytic.rotation_matrix(rotation)
        moments, R = self.assertPrincipalDecomposition(Rg @ np.diag([1.0, 2.0, 3.0]) @ Rg.T)
        np.testing.assert_allclose(moments, [1.0, 2.0, 3.0], atol=1e-12)
        # The principal axes are the axes of the rotation (smallest rotation chosen).
        np.testing.assert_allclose(R, Rg, atol=1e-9)

    def test_diagonal_matrix_gives_identity(self):
        _, R = self.assertPrincipalDecomposition(np.diag([1.0, 2.0, 3.0]))
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_diagonal_matrix_in_descending_order(self):
        moments, R = self.assertPrincipalDecomposition(np.diag([3.0, 2.0, 1.0]))
        np.testing.assert_allclose(moments, [1.0, 2.0, 3.0], atol=1e-12)
        # x' is along +-z, z' along +-x.
        self.assertAlmostEqual(abs(R[2, 0]), 1.0, places=12)
        self.assertAlmostEqual(abs(R[0, 2]), 1.0, places=12)

    def test_text_with_principal_axes(self):
        rotation = fc.Rotation(fc.Vector(0, 0, 1), 90.0)
        text = self.command.format_inertial_properties(
            0.785, fc.Vector(50.0, 20.0, 12.5), np.diag([1.0e3, 2.0e3, 3.0e3]),
            header='Header line', axes_rotation=rotation,
        )
        lines = text.splitlines()
        self.assertEqual(lines[0], 'Header line')
        rotation_line = next(i for i, line in enumerate(lines) if 'Rotation to principal axes' in line)
        matrix_line = next(i for i, line in enumerate(lines) if 'principal axes, kg*mm^2' in line)
        # The rotation is printed just before the matrix.
        self.assertEqual(matrix_line, rotation_line + 1)
        self.assertIn('axis ( 0.000000,  0.000000,  1.000000)', lines[rotation_line])
        self.assertIn('angle 90 deg', lines[rotation_line])
        self.assertIn('yaw-pitch-roll: 90, 0, 0 deg', lines[rotation_line])
        self.assertIn('2.000000e+03', lines[matrix_line + 2])   # Iyy on the second matrix row.

    def test_text_without_principal_axes(self):
        text = self.command.format_inertial_properties(1.0, fc.Vector(), np.eye(3))
        self.assertNotIn('principal', text)
        self.assertNotIn('Rotation', text)


class ErrorSummaryTests(InertiaCalculatorTestCase):

    def setUp(self) -> None:
        super().setUp()
        self.command = import_command('command_calculate_mass_and_inertia')

    def test_lists_every_message(self):
        messages = ['Link "a" skipped. No bound Real element for Link.', 'Link "b" skipped. Calculated mass is zero.']
        text = self.command.error_summary(messages)
        self.assertIn('2 link(s)', text)
        for message in messages:
            self.assertIn(f'- {message}', text)
        self.assertIn('previous inertial properties', text)
