"""Pure math / property-reading unit tests.

`screw_tf_matrix`, `inertia_matrix6d` and `rotation_matrix3` are tested
against bare `FreeCAD.Matrix`/`Vector`/`Rotation` value objects, with no
`Document` involved. `get_property`/`get_property_optional` do need a
`DocumentObject` to read properties off of, so they use the same
`InertiaCalculatorTestCase` document fixture as the integration tests,
but exercise nothing about the object tree walk itself.
"""
from __future__ import annotations

import numpy as np

import FreeCAD as fc

from .. import inertia_calculator as ic
from . import analytic
from .base import InertiaCalculatorTestCase


class ScrewTfMatrixTests(InertiaCalculatorTestCase):

    def test_identity(self):
        T = fc.Placement().Matrix
        X = ic.screw_tf_matrix(T)
        np.testing.assert_allclose(X, np.eye(6), atol=1e-12)

    def test_pure_translation(self):
        t = fc.Vector(10.0, -20.0, 5.0)
        T = fc.Placement(t, fc.Rotation()).Matrix
        X = ic.screw_tf_matrix(T)
        expected = np.zeros((6, 6))
        expected[0:3, 0:3] = np.eye(3)
        expected[3:6, 3:6] = np.eye(3)
        expected[3:6, 0:3] = analytic.skew(t)
        np.testing.assert_allclose(X, expected, atol=1e-9)

    def test_pure_rotation(self):
        rot = fc.Rotation(fc.Vector(0, 0, 1), 90.0)
        T = fc.Placement(fc.Vector(0, 0, 0), rot).Matrix
        X = ic.screw_tf_matrix(T)
        R = analytic.rotation_matrix(rot)
        expected = np.zeros((6, 6))
        expected[0:3, 0:3] = R
        expected[3:6, 3:6] = R
        np.testing.assert_allclose(X, expected, atol=1e-9)

    def test_combined_matches_analytic_transform_with_nonzero_com(self):
        # A screw transform built from an arbitrary placement should move
        # a generalized inertia matrix the same way
        # analytic.transform_inertia6d does, since both are independent
        # implementations of the same rigid-body transform. The body's
        # own center of mass is deliberately nonzero: with com=0 the
        # mass/com cross term vanishes regardless of its sign convention,
        # which would make this comparison pass even if the two disagreed
        # about that sign (see InertiaMatrix6dTests below, where they do).
        placement = fc.Placement(fc.Vector(3, -4, 7), fc.Rotation(fc.Vector(1, 1, 0), 37.0))
        j3 = np.diag([1.0, 2.0, 3.0])
        com = fc.Vector(1.0, -2.0, 0.5)
        mass = 4.0
        J0 = analytic.generalized_inertia6d(j3, com, mass)
        X = ic.screw_tf_matrix(placement.Matrix.inverse())
        via_screw = X.T @ J0 @ X
        via_analytic = analytic.transform_inertia6d(j3, com, mass, placement)
        np.testing.assert_allclose(via_screw, via_analytic, atol=1e-6)


class InertiaMatrix6dTests(InertiaCalculatorTestCase):
    """
    `inertia_matrix6d` used to assemble its top-left block as

        J + m * S @ S           (S = skew(com))

    which has the mass/center-of-mass cross term backwards: the
    generalized parallel axis theorem requires `J + m * (S.T @ S)`
    (equivalently `J - m * S @ S`, since `S.T == -S` for a
    skew-symmetric matrix) -- see `analytic.generalized_inertia6d`'s
    docstring for the derivation. The difference was not cosmetic: with
    the old `+` sign, `test_rejects_decreasing_inertia_away_from_com`
    below would show a solid's inertia about a point *decreasing*, and
    even going negative, the further that point is from the center of
    mass, which is impossible for any real rigid body -- this mattered
    for every leaf solid `calculate()` computes (it calls
    `inertia_matrix6d(shape.MatrixOfInertia, shape.CenterOfMass,
    shape.Mass)`, and `Shape.MatrixOfInertia` is documented to be the
    inertia about the shape's own center of mass -- see
    https://dev.opencascade.org/doc/refman/html/class_g_prop___g_props.html),
    i.e. whenever a shape's center of mass isn't exactly at its local
    origin, which is the common case (e.g. any `Part::Box`, whose
    centroid is at `(L/2, W/2, H/2)`, not `(0, 0, 0)`). It was also
    inconsistent with `screw_tf_matrix` elsewhere in the same module,
    which `ScrewTfMatrixTests` above confirms matches the textbook
    parallel-axis theorem.

    This has since been fixed upstream (now `J - mS @ S`); both tests
    below are expected to pass as written -- they were not adjusted to
    match the old behavior, so they needed no changes to pick up the fix.
    """

    def test_rejects_decreasing_inertia_away_from_com(self):
        # Textbook parallel axis theorem, directly: a point mass m (zero
        # inertia about itself) at distance d from a reference point has
        # inertia m*d^2 about that point around any axis perpendicular to
        # the offset -- never less than the (zero) inertia it has about
        # its own center of mass, regardless of d.
        m, d = 3.0, 2.0
        zero_j = fc.Matrix(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1)
        com = fc.Vector(d, 0.0, 0.0)
        JJ = ic.inertia_matrix6d(zero_j, com, m)
        expected = np.diag([0.0, m * d * d, m * d * d])
        np.testing.assert_allclose(
            JJ[0:3, 0:3], expected, atol=1e-9,
            err_msg=(
                'inertia_matrix6d does not satisfy the parallel axis theorem for a'
                ' point mass offset from the reference point -- see this class\'s'
                ' docstring.'
            ),
        )

    def test_matches_analytic_assembly(self):
        # See this class's docstring for why this, and not a `+` sign,
        # is the correct cross term.
        j3 = np.array([
            [2.0, 0.1, 0.2],
            [0.1, 3.0, 0.3],
            [0.2, 0.3, 4.0],
        ])
        com = fc.Vector(1.5, -2.5, 0.5)
        mass = 6.0
        J = fc.Matrix(
            j3[0, 0], j3[0, 1], j3[0, 2], 0.0,
            j3[1, 0], j3[1, 1], j3[1, 2], 0.0,
            j3[2, 0], j3[2, 1], j3[2, 2], 0.0,
            0.0, 0.0, 0.0, 1.0,
        )
        actual = ic.inertia_matrix6d(J, com, mass)
        expected = analytic.generalized_inertia6d(j3, com, mass)
        np.testing.assert_allclose(actual, expected, atol=1e-9)

    def test_zero_com_reduces_to_block_diagonal(self):
        j3 = np.diag([1.0, 2.0, 3.0])
        mass = 4.0
        J = fc.Matrix(
            j3[0, 0], 0, 0, 0,
            0, j3[1, 1], 0, 0,
            0, 0, j3[2, 2], 0,
            0, 0, 0, 1,
        )
        actual = ic.inertia_matrix6d(J, fc.Vector(0, 0, 0), mass)
        expected = np.zeros((6, 6))
        expected[0:3, 0:3] = j3
        expected[3:6, 3:6] = mass * np.eye(3)
        np.testing.assert_allclose(actual, expected, atol=1e-9)


class RotationMatrix3Tests(InertiaCalculatorTestCase):

    def test_matches_independent_quaternion_based_implementation(self):
        for axis, angle in [
            (fc.Vector(0, 0, 1), 90.0),
            (fc.Vector(1, 0, 0), 37.0),
            (fc.Vector(1, 1, 1), 123.0),
        ]:
            rot = fc.Rotation(axis, angle)
            actual = ic.rotation_matrix3(rot)
            expected = analytic.rotation_matrix(rot)
            np.testing.assert_allclose(actual, expected, atol=1e-9, err_msg=f'axis={axis}, angle={angle}')

    def test_identity(self):
        actual = ic.rotation_matrix3(fc.Rotation())
        np.testing.assert_allclose(actual, np.eye(3), atol=1e-12)


class GetPropertyTests(InertiaCalculatorTestCase):

    def setUp(self) -> None:
        super().setUp()
        self.obj = self.doc.addObject('Part::Feature', 'Dummy')

    def test_quantity_property_native_units(self):
        self.obj.addProperty('App::PropertyLength', 'TestLength', 'Test', '')
        self.obj.TestLength = 1500.0  # mm, the property's native unit.
        value = ic.get_property(self.obj, 'TestLength', 'App::PropertyLength')
        self.assertAlmostEqual(value, 1500.0, places=6)

    def test_quantity_property_unit_conversion(self):
        self.obj.addProperty('App::PropertyLength', 'TestLength', 'Test', '')
        self.obj.TestLength = 1500.0  # mm
        value = ic.get_property(self.obj, 'TestLength', 'App::PropertyLength', units='m')
        self.assertAlmostEqual(value, 1.5, places=9)

    def test_bare_float_property(self):
        self.obj.addProperty('App::PropertyFloat', 'TestFloat', 'Test', '')
        self.obj.TestFloat = 3.25
        value = ic.get_property(self.obj, 'TestFloat', 'App::PropertyFloat')
        self.assertAlmostEqual(value, 3.25, places=9)

    def test_missing_property_raises_key_error(self):
        with self.assertRaises(KeyError):
            ic.get_property(self.obj, 'DoesNotExist', 'App::PropertyFloat')

    def test_missing_property_with_default(self):
        value = ic.get_property(self.obj, 'DoesNotExist', 'App::PropertyFloat', default=42.0)
        self.assertEqual(value, 42.0)

    def test_wrong_type_raises_type_error(self):
        self.obj.addProperty('App::PropertyFloat', 'TestFloat', 'Test', '')
        with self.assertRaises(TypeError):
            ic.get_property(self.obj, 'TestFloat', 'App::PropertyLength')

    def test_check_func_failure_raises_value_error(self):
        self.obj.addProperty('App::PropertyFloat', 'TestFloat', 'Test', '')
        self.obj.TestFloat = -1.0
        with self.assertRaises(ValueError):
            ic.get_property(
                self.obj, 'TestFloat', 'App::PropertyFloat',
                check_func=lambda v: v >= 0.0, check_desc='must be non-negative',
            )


class GetPropertyOptionalTests(InertiaCalculatorTestCase):

    def setUp(self) -> None:
        super().setUp()
        self.obj = self.doc.addObject('Part::Feature', 'Dummy')

    def test_missing_returns_none(self):
        self.assertIsNone(ic.get_property_optional(self.obj, 'DoesNotExist', 'App::PropertyFloat'))

    def test_present_returns_value(self):
        self.obj.addProperty('App::PropertyFloat', 'TestFloat', 'Test', '')
        self.obj.TestFloat = 9.5
        value = ic.get_property_optional(self.obj, 'TestFloat', 'App::PropertyFloat')
        self.assertAlmostEqual(value, 9.5, places=9)

    def test_check_func_still_applies_when_present(self):
        self.obj.addProperty('App::PropertyFloat', 'TestFloat', 'Test', '')
        self.obj.TestFloat = -1.0
        with self.assertRaises(ValueError):
            ic.get_property_optional(
                self.obj, 'TestFloat', 'App::PropertyFloat',
                check_func=lambda v: v >= 0.0, check_desc='must be non-negative',
            )
