"""Module to compute the inverse kinematics of a Robot."""

from enum import Enum

import FreeCAD as fc

# Stubs and type hints.
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501


class IKAlgorithm(Enum):
    """The available inverse kinematics algorithms."""
    # The names are used in a GUI, so they should be user-friendly.
    PINOCCHIO_SINGLE = "Pinocchio - Single solution (search-based)"
    PINOCCHIO_NR = "Pinocchio - Newton-Raphson (search-based)"
    PINOCCHIO_DLS = "Pinocchio - Damped Least Squares (search-based)"
    PINOCCHIO_TL = "Pinocchio - Transpose (search-based)"
    PINOCCHIO_CLOSED_FORM = "Pinocchio - Closed form (analytical)"


def ik(
        robot: CrossRobot,
        from_link: str,
        to_link: str,
        target: fc.Placement,
        algorithm: IKAlgorithm = IKAlgorithm.PINOCCHIO_SINGLE,
        seed: list[float] | None = None,
        fixed_joints: dict[str, float] | None = None,
        max_iter=1000,
        transl_tol=1e-6,
        rot_tol=1e-6,
) -> list[list[float]]:
    """Return some solutions to the inverse kinematics of a Robot.

    Compute the joint positions to reach the given target.
    Some algorithms may return multiple solutions.

    For search-based algorithms, a seed configuration must be provided.
    The optional arguments `max_iter`, `transl_tol` and `rot_tol` are used to
    tune the search in this case.

    Return a list of solutions, each solution being a list of joint values in FreeCAD
    units (mm and degrees).

    Args:
        - robot: The Robot to compute the inverse kinematics for.
        - from_link: The name of the link to start from.
        - to_link: The name of the link to reach.
        - target: The target placement of `to_link` relative to `from_link`.
        - seed: An optional seed configuration to start the search from.
                The number of values must match the number of joints from
                `to_link` to `from_link`, including fixed ones.
        - fixed_joints: An optional dictionary of joint names to fix with their
                value.
                This argument is mandatory for closed-form algorithms for robots
                with more joints than the degree of freedom of their end-effector.
        - max_iter: The maximum number of iterations to perform.
        - transl_tol: The translational tolerance to consider the target reached.
        - rot_tol: The rotational tolerance to consider the target reached.
    """
    from .solver_wrappers.pinocchio import ik

    if not isinstance(algorithm, IKAlgorithm):
        raise TypeError("The algorithm must be an instance of IKAlgorithm")

    if algorithm == IKAlgorithm.PINOCCHIO_SINGLE:
        try:
            return ik(
                    robot,
                    from_link,
                    to_link,
                    target,
                    seed,
                    fixed_joints,
                    max_iter,
                    transl_tol,
                    rot_tol,
            )
        except ImportError:
            raise ImportError(
                "The pinocchio module is required to use the ik() function."
                "Please install it from robotpkg (robotpkg-py3??-pinocchio) or from"
                "the OpenRobots PPA (python3-pinocchio)."
            )
    else:
        raise NotImplementedError(f"The algorithm {algorithm} is not implemented yet")
