import xml.etree.ElementTree as et

import FreeCAD as fc

from .wb_utils import get_chain_from_to
from .wb_utils import get_valid_urdf_name
from .wb_utils import is_joint
from .wb_utils import is_link
from .wb_utils import is_robot
from .wb_utils import joint_quantities_from_si_units as wb_fc_from_si
from .wb_utils import joint_values_si_units_from_freecad as wb_si_from_fc
from .wb_utils import ros_name
from .wb_utils import warn

# Stubs and type hints.
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501


def get_kinematic_urdf(
        robot: CrossRobot,
        from_link: str,
        to_link: str,
) -> str:
    """Return the URDF of the kinematic chain between two links.

    Visual and collision are not part of the URDF.
    """
    if not is_robot(robot):
        raise TypeError("The given object is not a Robot")
    chain = get_chain_from_to(robot, from_link, to_link)
    if not chain:
        raise ValueError(
            f"Could not find a kinematic chain from '{from_link}' to '{to_link}'",
        )
    # Implementation note: no need to check for validity here, because
    # this was done in `get_chain_from_to()`.
    to = robot.Proxy.get_link(to_link)

    augmented_chain = chain + [to]

    xml = et.fromstring('<robot/>')
    xml.attrib['name'] = get_valid_urdf_name(robot.Label)
    for element in augmented_chain:
        if is_link(element):
            xml.append(et.fromstring(
                f'<link name="{get_valid_urdf_name(ros_name(element))}" />',
            ))
        elif is_joint(element):
            if not element.Parent:
                warn(f"Joint '{element.Label}' has no parent link", True)
                return ""
            if not element.Child:
                warn(f"Joint '{element.Label}' has no child link", True)
                continue
            # Implementation note: no need to check for validity of `element.Proxy`.
            xml.append(element.Proxy.export_urdf())

    return et.tostring(xml, encoding="unicode")


def get_kinematic_urdf_from_root(
        robot: CrossRobot,
        to_link: str,
) -> str:
    """
    Return the URDF of the kinematic chain between the root link and a given link.

    Visual and collision are not part of the URDF.
    """
    if not is_robot(robot):
        raise TypeError("The given object is not a Robot")
    base_link = robot.Proxy.get_root_link()
    if not base_link:
        raise ValueError("The robot has no base link")
    return get_kinematic_urdf(robot, ros_name(base_link), to_link)


def joint_values_freecad_from_si_units(
        robot: CrossRobot,
        joint_values: dict[str, float],
) -> dict[str, float]:
    """
    Return the joint values in FreeCAD units.

    Return the joint values in FreeCAD units (millimeters and degrees)
    from a dictionary of joint values in SI units.
    """
    if not is_robot(robot):
        raise TypeError("The given object is not a Robot")

    crossjoint_values = {robot.Proxy.get_joint(j): v for j, v in joint_values.items()}
    if None in crossjoint_values:
        msg = "At least one of {list(joint_values.keys())} is not part of the robot"
        raise RuntimeError(msg)

    return {ros_name(j): q.Value for j, q in wb_fc_from_si(crossjoint_values).items()}


def joint_values_si_units_from_freecad(
        robot: CrossRobot,
        joint_values: dict[str, float | fc.Units.Quantity],
) -> dict[str, float]:
    """
    Return the joint values in SI units.

    Return the joint values in SI units (meters and radians) from a dictionary
    of joint values in FreeCAD units (millimeters and degrees).
    """
    if not is_robot(robot):
        raise TypeError("The given object is not a Robot")
    crossjoint_values = {robot.Proxy.get_joint(j): v for j, v in joint_values.items()}
    if None in crossjoint_values:
        msg = "At least one of {list(joint_values.keys())} is not part of the robot"
        raise RuntimeError(msg)

    return {ros_name(j): v for j, v in wb_si_from_fc(crossjoint_values).items()}


def axis_pose_freecad_from_si_units(
        robot: CrossRobot,
        joint_values: list[float],
        joint_names: list[str],
) -> list[float]:
    """Return the joint-space pose in FreeCAD units.

    Return the robot's joint-space pose in FreeCAD units (millimeters and degrees)
    from a list of joint values in SI units (meters and radians) and associated names.
    """
    if len(joint_names) != len(joint_values):
        raise ValueError("The number of joint names must match the number of joint values")

    value_map_si = {j: v for j, v in zip(joint_names, joint_values)}
    value_map_fc = joint_values_freecad_from_si_units(robot, value_map_si)
    return [value_map_fc[j] for j in joint_names]


def robot_axis_pose_si_units(
        robot: CrossRobot,
        joint_names: list[str],
) -> list[float]:
    """
    Return the robot's joint-space pose in SI units.

    Return the robot's joint-space pose in SI units (meters and radians).
    """
    if not is_robot(robot):
        raise TypeError("The given object is not a Robot")

    joint_values = robot.Proxy.get_joint_values()
    robot_joint_names = [ros_name(j) for j in joint_values.keys()]
    for joint_name in joint_names:
        if joint_name not in robot_joint_names:
            raise RuntimeError(f"Joint '{joint_name}' is not part of the robot")

    joint_values_si_units = wb_si_from_fc(joint_values)
    return [joint_values_si_units[robot.Proxy.get_joint(j)] for j in joint_names]
