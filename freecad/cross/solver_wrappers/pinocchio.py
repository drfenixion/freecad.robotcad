from math import pi, radians

import FreeCAD as fc

from ..ik_utils import axis_pose_freecad_from_si_units
from ..ik_utils import get_kinematic_urdf
from ..ik_utils import joint_values_si_units_from_freecad
from ..ik_utils import robot_axis_pose_si_units
from ..wb_utils import warn

# Stubs and type hints.
from ..robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501


def ik(
        robot: CrossRobot,
        from_link: str,
        to_link: str,
        target: fc.Placement,
        seed: list[float] | None = None,
        fixed_joints: dict[str, float | fc.Units.Quantity] | None = None,
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

    Args:
        - robot: The Robot to compute the inverse kinematics for.
        - from_link: The name of the link to start from.
        - to_link: The name of the link to reach.
        - target: The target placement of `to_link` relative to `from_link`.
        - seed: An optional seed configuration to start the search from.
                The number of values must match the number of joints from
                `to_link` to `from_link`, including fixed ones.
        - fixed_joints: An optional dictionary of joint names to fix with their
                value in FreeCAD units or as quantity.
        - max_iter: The maximum number of iterations to perform.
        - transl_tol: The translational tolerance to consider the target
                reached.
        - rot_tol: The rotational tolerance to consider the target reached.

    Return a list of solutions in FreeCAD units (mm and degrees).
    """

    # Late import to avoid hard dependency on pinocchio and to avoid slowing
    # down FreeCAD's launch.
    import tempfile
    import os
    import numpy as np

    # Add /opt/openrobots/lib/python?.?/site-packages before importing
    # pinocchio.
    # Otherwise, pinocchio, pinocchio tries to load hppfcl (provided by ROS)
    # instead of coal (provided by robotpkg) and fails.
    from ..solver_wrappers.robotpkg import add_robotpkg_library_path
    add_robotpkg_library_path()

    try:
        import pinocchio
    except ImportError as e:
        warn(
            "The pinocchio module is required to use the ik() function."
            " Please install it e.g. with 'pip install pinocchio'."
            f" The thrown error was: {e}",
            gui=True,
        )
        raise

    if (robot, from_link, to_link) in ik._model_cache:
        model = ik._model_cache[(robot, from_link, to_link)]
    else:
        urdf_str = get_kinematic_urdf(robot, from_link, to_link)
        # Write URDF to a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix='.urdf') as tmp:
            tmp.write(urdf_str.encode('utf-8'))
            urdf_path = tmp.name

        # Load model from URDF
        model = pinocchio.buildModelFromUrdf(urdf_path)
        ik._model_cache[(robot, from_link, to_link)] = model

        # Remove the temporary file
        os.unlink(urdf_path)

    data = model.createData()

    # The first joint is the universe joint, we skip it.
    joint_names = [name for name in model.names][1:]

    fixed_joints = fixed_joints or {}
    for name in fixed_joints:
        if name not in joint_names:
            raise ValueError(f'The fixed joint "{name}" is not part of the kinematic chain from "{from_link}" to "{to_link}"')

    fixed_joints_si = joint_values_si_units_from_freecad(robot, fixed_joints)

    # Initial configuration
    if seed is not None:
        if len(seed) != len(joint_names):
            raise ValueError(f"The seed configuration must have {len(joint_names)} values, but got {len(seed)}")
        q = np.array(seed)
    else:
        q = np.array(robot_axis_pose_si_units(robot, joint_names))
    q = _fix_joint_values(q, joint_names, fixed_joints_si)

    # Get target placement as SE3
    target_se3 = pinocchio.SE3(np.array(target.Matrix.A).reshape((4, 4)))
    fc_to_pin = 0.001  # FreeCAD to Pinocchio units (mm to m)
    target_se3.translation *= fc_to_pin

    # Find joint id for to_link
    to_link_id = model.getFrameId(to_link)

    # Simple iterative IK (Gauss-Newton)
    # Cf. https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b_examples_d_inverse_kinematics.html
    # for an alternative implementation.
    last_lin_err = float('inf')
    last_ang_err = float('inf')
    for _ in range(max_iter):
        pinocchio.forwardKinematics(model, data, q)
        pinocchio.updateFramePlacements(model, data)
        current_se3 = data.oMf[to_link_id]
        err = pinocchio.log6(current_se3.inverse() * target_se3).vector
        lin_err = np.linalg.norm(err[:3])
        ang_err = np.linalg.norm(err[3:])
        if lin_err < transl_tol and ang_err < rot_tol:
            return [axis_pose_freecad_from_si_units(robot, q, joint_names)]
        if abs(lin_err - last_lin_err) < (transl_tol / 10.0) and abs(ang_err - last_ang_err) < (rot_tol / 10.0):
            # The error is not decreasing anymore, we are likely stuck in a local minimum.
            break
        J = pinocchio.computeFrameJacobian(model, data, q, to_link_id)
        v = np.linalg.pinv(J) @ err
        q = _fix_joint_values(q + v, joint_names, fixed_joints_si)
        q = _modulo_2pi(q, joint_names, robot)
        last_lin_err = lin_err
        last_ang_err = ang_err
    warn('IK did not converge', gui=True)
    return []

ik._model_cache: dict[tuple[CrossRobot, str, str], 'pinocchio.pinocchio_pywrap_default.Model'] = {}


def _fix_joint_values(
        q: 'numpy.ndarray',
        joint_names: list[str],
        fixed_joints: dict[str, float],
) -> 'numpy.ndarray':
    """Return a new configuration with the fixed joints set to their fixed value."""
    q_fixed = q.copy()
    for i, name in enumerate(joint_names):
        if name in fixed_joints:
            q_fixed[i] = fixed_joints[name]
    return q_fixed


def _modulo_2pi_single(
        value: float,
        min_value: float,
        max_value: float,
) -> float:
    while value < min_value:
        value += 2 * pi
    while value > max_value:
        value -= 2 * pi
    return value


def _modulo_2pi(
        q: 'numpy.ndarray',
        joint_names: list[str],
        robot: CrossRobot,
) -> 'numpy.ndarray':
    """Return a new configuration with the revolute joints wrapped within their limit."""
    q_mod = q.copy()
    for i, name in enumerate(joint_names):
        joint = robot.Proxy.get_joint(name)
        if joint and joint.Type == 'revolute':
            min_value = radians(joint.LowerLimit)
            max_value = radians(joint.UpperLimit)
            q_mod[i] = _modulo_2pi_single(q[i], min_value, max_value)
    return q_mod
