import numpy as np

EPSILON = 1.0e-5


def are_parallel(vec1, vec2):
    """Determine if two vectors are parallel."""
    vec1_unit = vec1 / np.linalg.norm(vec1)
    vec2_unit = vec2 / np.linalg.norm(vec2)

    return np.all(abs(np.cross(vec1_unit, vec2_unit)) < EPSILON)


def are_collinear(point1, vec1, point2, vec2):
    """Determine if vectors are collinear."""

    # To be collinear, vectors must be parallel
    if not are_parallel(vec1, vec2):
        return False

    # If parallel and point1 is coincident with point2, vectors are collinear
    if all(np.isclose(point1, point2)):
        return True

    # If vectors are parallel, point2 can be defined as p2 = p1 + t * v1
    t = np.zeros(3)
    for idx in range(0, 3):
        if vec1[idx] != 0:
            t[idx] = (point2[idx] - point1[idx]) / vec1[idx]
    p2 = point1 + t * vec1

    return np.allclose(p2, point2)


def lines_intersect(point1, vec1, point2, vec2):
    """Determine if two lines intersect."""
    epsilon = 1e-6
    x = np.zeros(2)

    # If lines are collinear, they have an infinite number of intersections.
    # Choose point1.
    if are_collinear(point1, vec1, point2, vec2):
        return True, np.atleast_1d(point1).copy()

    # If lines are parallel, they don't intersect.
    if are_parallel(vec1, vec2):
        return False, np.zeros_like(point1)

    # Test if lines intersect. Need to find non-singular
    # pair to solve for coefficients.
    for idx in range(3):
        i = idx
        j = (idx + 1) % 3
        A = np.array([[vec1[i], -vec2[i]], [vec1[j], -vec2[j]]])
        b = np.array([[point2[i] - point1[i]], [point2[j] - point1[j]]])

        # If singular matrix, go to next set
        if np.isclose(np.linalg.det(A), 0):
            continue
        else:
            x = np.linalg.solve(A, b)

            # Test if solution generates a point of intersection
            p1 = point1 + x[0] * vec1
            p2 = point2 + x[1] * vec2

            if all(np.less(np.abs(p1 - p2), epsilon * np.ones(3))):
                return True, x.flatten()

    return False, x
