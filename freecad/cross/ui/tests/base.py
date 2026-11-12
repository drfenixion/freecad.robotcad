"""Shared `unittest.TestCase` base class and numeric assertion helpers.

Everything here talks to a real FreeCAD document; none of it depends on
`inertia_calculator.py` itself, so it is safe to share between the pure-math
tests and the full-document integration tests.
"""
from __future__ import annotations

import itertools
import unittest

import numpy as np

import FreeCAD as fc


class InertiaCalculatorTestCase(unittest.TestCase):
    """Base test case that gives every test its own scratch document.

    Subclasses should not need to override `setUp`/`tearDown`; add any
    extra per-class fixtures in `setUpClass`/`tearDownClass` instead.
    """

    def setUp(self) -> None:
        # A unique name per test avoids collisions if a previous test's
        # tearDown failed to run (e.g. because the test itself errored
        # out before reaching an assertion).
        self._doc_name = f'InertiaCalculatorTest_{self._testMethodName}_{id(self)}'
        self.doc = fc.newDocument(self._doc_name)

    def tearDown(self) -> None:
        try:
            fc.closeDocument(self.doc.Name)
        except Exception:
            # Best-effort: if the document was already closed/renamed by
            # the test, do not let teardown mask the real failure.
            pass

    def recompute(self) -> None:
        self.doc.recompute()

    # -- Generalized 6x6 inertia matrix helpers ---------------------------

    @staticmethod
    def split_inertia6d(JJ: np.ndarray):
        """Decompose a generalized 6x6 rigid-body inertia matrix.

        Returns `(mass, com, inertia3)` where `com` is an `fc.Vector` and
        `inertia3` is the 3x3 rotational inertia tensor about that center
        of mass, both expressed in whatever frame `JJ` itself is
        expressed in.

        This mirrors the block layout produced by
        `analytic.generalized_inertia6d` (see that function's docstring
        for why the cross term's sign is `J - m*S@S` rather than `J +
        m*S@S`, which is `inertia_calculator.inertia_matrix6d`'s own
        convention and the two are not the same):

            JJ = [[J - m*S@S,      m*S],
                  [    m*S.T, m*eye(3)]]

        with `S` the skew-symmetric matrix of the center of mass. It is
        written independently of `inertia_calculator.mass_com_inertia`
        (which this suite does not use, see test_math_helpers.py) so that
        test expectations never depend on the correctness of the module
        under test.
        """
        mass = (JJ[3, 3] + JJ[4, 4] + JJ[5, 5]) / 3.0
        if mass == 0.0:
            return 0.0, fc.Vector(0, 0, 0), np.zeros((3, 3))
        mS = JJ[0:3, 3:6]
        # mS = m * skew(com) = [[0, -m*cz, m*cy], [m*cz, 0, -m*cx], [-m*cy, m*cx, 0]]
        com = fc.Vector(mS[2, 1], mS[0, 2], mS[1, 0]) / mass
        S = np.array([
            [0.0, -com.z, com.y],
            [com.z, 0.0, -com.x],
            [-com.y, com.x, 0.0],
        ])
        inertia3 = JJ[0:3, 0:3] + mass * (S @ S)
        return mass, com, inertia3

    def assertInertia6Equal(self, actual: np.ndarray, expected: np.ndarray, atol: float = 1e-6, rtol: float = 1e-6, msg: str = '') -> None:
        """Compare two generalized 6x6 inertia matrices.

        On failure, prints the decomposed mass/center-of-mass/inertia-3x3
        for both sides, which is much easier to read than a raw 6x6 diff.
        """
        actual = np.asarray(actual, dtype=float)
        expected = np.asarray(expected, dtype=float)
        self.assertEqual(actual.shape, (6, 6), 'actual is not a 6x6 matrix')
        self.assertEqual(expected.shape, (6, 6), 'expected is not a 6x6 matrix')
        if np.allclose(actual, expected, atol=atol, rtol=rtol):
            return
        a_mass, a_com, a_j = self.split_inertia6d(actual)
        e_mass, e_com, e_j = self.split_inertia6d(expected)
        details = (
            f'{msg}\n'
            f'mass:     actual={a_mass!r} expected={e_mass!r}\n'
            f'com:      actual={a_com!r} expected={e_com!r}\n'
            f'inertia3: actual=\n{a_j}\nexpected=\n{e_j}\n'
            f'full 6x6: actual=\n{actual}\nexpected=\n{expected}'
        )
        raise AssertionError(details)

    def assertVectorAlmostEqual(self, actual: fc.Vector, expected: fc.Vector, places: int = 6, msg: str = '') -> None:
        for a, e, axis in zip((actual.x, actual.y, actual.z), (expected.x, expected.y, expected.z), 'xyz'):
            self.assertAlmostEqual(a, e, places=places, msg=f'{msg} (axis {axis}): actual={actual!r} expected={expected!r}')


def all_shape_types(doc: fc.Document):
    """Yield every object currently in `doc`, for debugging a failed container test."""
    return itertools.chain(doc.Objects)
