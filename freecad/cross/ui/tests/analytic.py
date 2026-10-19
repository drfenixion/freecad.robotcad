"""Independent, closed-form reference values for the integration tests.

Nothing in this module imports `inertia_calculator`: every formula here is
rebuilt from first principles (standard rigid-body inertia formulas, the
generalized spatial inertia matrix, and a from-scratch rigid transform of
one) so that a bug shared between the module under test and its test's
"expected" values can't cancel out. `generalized_inertia6d` below is
intentionally not a call to `inertia_calculator.inertia_matrix6d`, and is
derived directly from the textbook parallel-axis theorem rather than from
reading that function's source -- which is important, because its sign
convention for the mass/center-of-mass cross term turned out to disagree
with `inertia_matrix6d`'s (see `generalized_inertia6d`'s docstring and
test_math_helpers.py's `InertiaMatrix6dTests`, which cross-checks the two
explicitly and documents which one matches the textbook formula).

All lengths are in mm and densities in kg/mm^3, matching FreeCAD's native
units (see materials.py for the kg/m^3 -> kg/mm^3 conversion used when a
test picks a density in the more common kg/m^3).
"""
from __future__ import annotations

import numpy as np

import FreeCAD as fc


def skew(v: fc.Vector) -> np.ndarray:
    """3x3 skew-symmetric (cross-product) matrix of a vector."""
    return np.array([
        [0.0, -v.z, v.y],
        [v.z, 0.0, -v.x],
        [-v.y, v.x, 0.0],
    ])


def rotation_matrix(rot: fc.Rotation) -> np.ndarray:
    """3x3 rotation matrix of an `fc.Rotation`, built from its quaternion.

    Deliberately not a call to `inertia_calculator.rotation_matrix3`
    (which itself is a unit-test target in test_math_helpers.py), so that
    the two are cross-checked against each other there instead of one
    silently reusing the other.
    """
    x, y, z, w = rot.Q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def box_inertia_com(mass: float, lx: float, ly: float, lz: float) -> np.ndarray:
    """3x3 inertia tensor of a solid cuboid about its centroid, axis-aligned.

    `lx`, `ly`, `lz` are the full edge lengths (not half-extents), matching
    `Part::Box`'s `Length`/`Width`/`Height`.
    """
    ixx = mass / 12.0 * (ly ** 2 + lz ** 2)
    iyy = mass / 12.0 * (lx ** 2 + lz ** 2)
    izz = mass / 12.0 * (lx ** 2 + ly ** 2)
    return np.diag([ixx, iyy, izz])


def cylinder_inertia_com(mass: float, radius: float, height: float, axis: str = 'z') -> np.ndarray:
    """3x3 inertia tensor of a solid cylinder about its centroid.

    `axis` is the cylinder's own axis of revolution ('x', 'y' or 'z'),
    matching `Part::Cylinder`/`PartDesign::AdditiveCylinder`, which are
    both extruded along local Z from a base circle, so `axis='z'` is the
    usual case.
    """
    i_axis = mass * radius ** 2 / 2.0
    i_perp = mass * (3.0 * radius ** 2 + height ** 2) / 12.0
    diag = {'x': [i_axis, i_perp, i_perp], 'y': [i_perp, i_axis, i_perp], 'z': [i_perp, i_perp, i_axis]}[axis]
    return np.diag(diag)


def generalized_inertia6d(inertia3_com: np.ndarray, com: fc.Vector, mass: float) -> np.ndarray:
    """Assemble the generalized 6x6 rigid-body inertia matrix.

    Assembled as

        JJ = [[J - m*S@S,      m*S],
              [    m*S.T, m*eye(3)]]

    with `J` the 3x3 inertia about the center of mass, `S = skew(com)`
    and `m` the mass, all expressed in the same frame. Equivalently, since
    `S` is skew-symmetric (`S.T == -S`), the top-left block is
    `J + m*(S.T @ S)` or `J + m*(S @ S.T)`.

    The sign of the `S@S` cross term here is deliberate and was checked
    against the textbook parallel-axis theorem directly (a point mass `m`
    at `com = (d, 0, 0)` with `J = 0` must give inertia `diag(0, m*d^2,
    m*d^2)` about the origin): `S@S == com @ com.T - |com|^2 * I` (a
    standard skew-symmetric-matrix identity), so `J - m*S@S == J +
    m*(|com|^2 * I - com @ com.T)`, which is the generalized parallel-axis
    theorem (inertia strictly increases away from the center of mass).
    `inertia_calculator.inertia_matrix6d` assembles this same block as `J
    + m*S@S` (positive, not negative); test_math_helpers.py cross-checks
    the two directly, since that plus/minus sign is exactly the kind of
    thing that is easy to get backwards.
    """
    S = skew(com)
    mS = mass * S
    JJ = np.zeros((6, 6))
    JJ[0:3, 0:3] = inertia3_com - mS @ S
    JJ[0:3, 3:6] = mS
    JJ[3:6, 0:3] = mS.T
    JJ[3:6, 3:6] = mass * np.eye(3)
    return JJ


def transform_inertia6d(inertia3_com: np.ndarray, com: fc.Vector, mass: float, placement: fc.Placement) -> np.ndarray:
    """Relocate a (inertia3_com, com, mass) triple by a rigid `fc.Placement`.

    Returns the generalized 6x6 matrix expressed in the frame that
    `placement` maps the body's local frame into, i.e. with
    `world_com = placement.multVec(com)` and the inertia tensor rotated
    (but not translated again -- the parallel-axis term is implicit in
    `generalized_inertia6d` via `world_com`).
    """
    R = rotation_matrix(placement.Rotation)
    world_com = placement.multVec(com)
    world_inertia3 = R @ inertia3_com @ R.T
    return generalized_inertia6d(world_inertia3, world_com, mass)


def sum_inertia6d(*matrices: np.ndarray) -> np.ndarray:
    """Add several generalized 6x6 matrices already expressed in a common frame."""
    total = np.zeros((6, 6))
    for m in matrices:
        total += m
    return total


# -- Affine (scaled) transforms --------------------------------------------
#
# A body of uniform density mapped by x' = A @ x + t (A any invertible 3x3
# matrix, e.g. rotation @ diag(scale)) has
#
#     mass'  = |det A| * mass
#     com'   = A @ com + t
#     P'     = |det A| * A @ P @ A.T
#
# where P = integral((x - com) (x - com)^T dm) is the second-moment matrix
# about the center of mass, related to the inertia tensor J about the
# center of mass by J = trace(P) * I - P, i.e. P = trace(J) / 2 * I - J.
# This is a general linear-map formula, written independently of
# `inertia_calculator.scale_inertia` (which only handles diagonal scales).


def affine(placement: fc.Placement | None = None, scale=(1.0, 1.0, 1.0)):
    """Return `(A, t)` of the map `x' = placement * (diag(scale) @ x)`.

    This is how FreeCAD applies `App::Link.ScaleVector` together with the
    link `Placement`: the scale acts in the local frame of the link, before
    its placement.
    """
    if placement is None:
        placement = fc.Placement()
    A = rotation_matrix(placement.Rotation) @ np.diag([float(v) for v in scale])
    t = np.array([placement.Base.x, placement.Base.y, placement.Base.z])
    return A, t


def compose_affine(*maps):
    """Compose affine maps `(A, t)`; the first one is applied last.

    `compose_affine(m2, m1)` is the map `x -> m2(m1(x))`.
    """
    A = np.eye(3)
    t = np.zeros(3)
    for A_i, t_i in reversed(maps):
        A, t = A_i @ A, A_i @ t + t_i
    return A, t


def affine_transform_inertia6d(inertia3_com: np.ndarray, com: fc.Vector, mass: float, affine_map) -> np.ndarray:
    """Generalized 6x6 matrix of a uniform-density body mapped by `affine_map = (A, t)`."""
    A, t = affine_map
    det = abs(np.linalg.det(A))
    P = 0.5 * np.trace(inertia3_com) * np.eye(3) - inertia3_com
    P_new = det * A @ P @ A.T
    J_new = np.trace(P_new) * np.eye(3) - P_new
    c = A @ np.array([com.x, com.y, com.z]) + t
    return generalized_inertia6d(J_new, fc.Vector(*c), mass * det)
