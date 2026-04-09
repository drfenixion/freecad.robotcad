# Generate a CROSS::Robot from Khalil-Kleinfinger representation.

from __future__ import annotations

from dataclasses import dataclass
from math import copysign
from math import degrees
from typing import TYPE_CHECKING

import FreeCAD as fc

from .freecad_utils import add_object
from .freecad_utils import make_group
from .joint_proxy import make_joint
from .kk_robot import KKFrame
from .link_proxy import make_link
from .robot_proxy import make_robot
from .utils import sorted_unique
from .wb_utils import ros_name

# Stubs and typing hints.
if TYPE_CHECKING:
    # from .joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
    from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
    from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
    DO = fc.DocumentObject
    DOG = fc.DocumentObjectGroup
    Part = DO  # A DocumentObject with type `App::Part`

@dataclass
class SymoroKKFrame:
    pre_rz: float
    pre_tz: float
    rx: float
    tx: float
    rz: float
    tz: float
    type: int
    antecedent: int


def robot_from_kk(
        doc: fc.Document,
        pre_rz: list[float],
        pre_tz: list[float],
        rx: list[float],
        tx: list[float],
        rz: list[float],
        tz: list[float],
        types: list[str],
        antecedents: list[int] = None,
        name: str = 'robot',
) -> CrossRobot:
    """
    Creates a CROSS::Robot from Khalil-Kleinfinger parameters.

    Parameters
    ==========
    pre_rz: γi, angle between X(i-1) and Xi about Zi, in radians.
    pre_tz: εi, distance between Oi and X(i-1), in meters.
    rx: ɑi, angle between Z(i-1) and Zi about X(i-1), in radians.
    tx: di, distance between O(i-1) and Zi, in meters.
    rz: Θi, angle between Xi and X'i about Zi, in radians.
    tz: ri, distance between Oi and O'i, in meters.
    types: 0: revolute, 1: prismatic, 2: fixed.
    antecedents: list of indices of the antecedent of each joint.

    """
    if (
               len(pre_rz) != len(pre_tz)
            or len(pre_rz) != len(tx)
            or len(pre_rz) != len(rx)
            or len(pre_rz) != len(tz)
            or len(pre_rz) != len(rz)
            or len(pre_rz) != len(types)
    ):
        msg = 'All lists must have the same length.'
        raise ValueError(msg)

    if not antecedents:
        antecedents = list(range(len(pre_tz)))

    if len(pre_tz) != len(antecedents):
        msg = (
                f'The length of antecedents ({len(antecedents)})'
                f' must be equal to the number of joints ({len(pre_tz)}).'
        )
        raise ValueError(msg)

    # Check that no antecedent is missing (no "hole") and that the largest
    # index is smaller that `len(pre_rz)`.
    unique_antecedents = sorted_unique(antecedents)
    no_hole = (unique_antecedents == list(range(len(unique_antecedents))))
    all_exist = (unique_antecedents[-1] < len(pre_rz))
    if not no_hole or not all_exist:
        msg = 'Antecedents do not correspond to a tree structure.'
        raise ValueError(msg)
    kk_frames = [SymoroKKFrame(*p) for p in zip(pre_rz, pre_tz, rx, tx, rz, tz, types, antecedents)]

    robot, parts_group = _make_robot(doc, name)

    if len(pre_rz) == 0:
        return robot

    _make_struct(robot, parts_group, kk_frames)
    return robot


def _make_robot(
        doc: fc.Document,
        name: str = 'robot',
) -> tuple[CrossRobot, DOG]:
    """Create a CROSS::Robot

    Return (robot object, parts group).

    The group called 'Parts' is potentially created and returned. If the object
    'Parts' is not a group, a different name will be given.

    """

    robot: CrossRobot = make_robot(name, doc)
    # Create a group 'Parts' to hold all parts in the assembly document.
    parts_group = make_group(doc, 'Parts', visible=False)
    robot.Proxy.created_objects.append(parts_group)

    return robot, parts_group


def _make_struct(
        robot: CrossRobot,
        parts_group: DOG,
        kk_frames: list[SymoroKKFrame],
) -> None:
    """Create a chain of alterning links and joints."""
    max_length = max(
            [abs(f.pre_tz) for f in kk_frames]
            + [abs(f.tx) for f in kk_frames]
            + [abs(f.tz) for f in kk_frames]
            + [0.010]  # In case all lengths are zero.
    )
    params_to_freecad = 1000.0  # Convert meters to millimeters.
    diameter_ratio = 0.1  # Diameter of the cylinder is 10% of the length.
    cylinder_diameter_mm = min(20.0, params_to_freecad * max_length * diameter_ratio)

    base_link = make_link('base_link', robot.Document)
    robot.addObject(base_link)

    links: list[CrossLink] = [base_link]
    # Loop over all frames and add a joint and a link for each.
    # The leaf links will be missing and added later.
    for j, f in enumerate(kk_frames):
        antecedent_part, link_part = _make_link_parts(
            parts_group,
            f'part_for_{f.antecedent}_{j + 1}',
            f,
            cylinder_diameter_mm,
        )
        link_part.ViewObject.Visibility = False

        joint = make_joint(f'joint_{j + 1}', robot.Document)
        # Will provoke a warning that the joint has no parent but
        # `Parent` cannot be set before a joint belongs to a robot.
        robot.addObject(joint)
        this_kk_joint = KKFrame(
                pre_rz=f.pre_rz,
                pre_tz=f.pre_tz,
                rx=f.rx,
                tx=f.tx,
                rz=f.rz,
                tz=f.tz,
                prismatic=False,  # Irrelevant.
        )
        if f.antecedent == 0:
            joint.Parent = ros_name(base_link)
            # The first joint of a branch is only determined by the
            # "pre"-transform of the current frame.
            joint.Origin, _ = this_kk_joint.to_placements()
            print(f'{joint.Label2}, {this_kk_joint=}, {joint.Origin=}')
        else:
            print(f'Antecedent of {joint.Label2} (j = {j}): {f.antecedent - 1}')
            # We subtract 1 because the KK frame for `base_link` is not
            # part of the KK parameters.
            parent_frame = kk_frames[f.antecedent - 1]
            joint.Parent = ros_name(links[f.antecedent])
            parent_kk_joint = KKFrame(
                    pre_rz=parent_frame.pre_rz,
                    pre_tz=parent_frame.pre_tz,
                    rx=parent_frame.rx,
                    tx=parent_frame.tx,
                    rz=parent_frame.rz,
                    tz=parent_frame.tz,
                    prismatic=False,  # Irrelevant.
            )
            _, parent_transform = parent_kk_joint.to_placements()
            this_pre_transform, _ = this_kk_joint.to_placements()
            print(f'{joint.Label2}, {parent_kk_joint=}, {parent_transform=}')
            print(f'{joint.Label2}, {this_kk_joint=}, {this_pre_transform=}')
            joint.Origin = parent_transform * this_pre_transform

        if f.type == 0:
            joint.Type = 'revolute'
        elif f.type == 1:
            joint.Type = 'prismatic'
        elif f.type == 2:
            joint.Type = 'fixed'
        else:
            raise ValueError(
                    f'Invalid type: {f.type}, expected 0, 1, or 2'
                    ' for resp. revolute, prismatic, or fixed.'
            )

        link = make_link(f'link_{j + 1}', robot.Document)
        robot.addObject(link)
        links.append(link)
        # Implementation note: `+=` doesn't work.
        link.Visual = link.Visual + [link_part]

        antecedent_link = links[f.antecedent]
        antecedent_link.Visual = antecedent_link.Visual + [antecedent_part]
        # Place `link_part` at its child joint.
        # link.MountedPlacement = joint.Origin

        joint.Child = ros_name(link)

    # Add the leaf links.
    antecedents = [f.antecedent for f in kk_frames]
    leafs = _leafs(antecedents)
    tool_number = 1
    for j in leafs:
        antecedent = antecedents[j]
        # We must use `antecedent + 1` to get the parent link
        # because links contains `base_link` whereas `kk_frames` does not.
        parent_link = links[antecedent + 1]
        print(f'Leaf {j}, antecedent: {antecedent}, parent_link: {ros_name(parent_link)}')
        parent_frame = kk_frames[f.antecedent]
        print(f'Leaf {j}, parent_frame: {parent_frame}')
        joint = make_joint(f'{ros_name(parent_link)}-tool{tool_number}', robot.Document)
        # Will provoke a warning that the joint has no parent but
        # `Parent` cannot be set before a joint belongs to a robot.
        robot.addObject(joint)
        joint.Parent = ros_name(parent_link)
        kk_joint = KKFrame(
                pre_rz=parent_frame.pre_rz,
                pre_tz=parent_frame.pre_tz,
                rx=parent_frame.rx,
                tx=parent_frame.tx,
                rz=parent_frame.rz,
                tz=parent_frame.tz,
                prismatic=False,  # Irrelevant.
        )
        _, joint.Origin = kk_joint.to_placements()
        joint.Type = 'fixed'

        link = make_link(f'tool{tool_number}', robot.Document)
        robot.addObject(link)
        links.append(link)

        joint.Child = ros_name(link)

        tool_number += 1


def _make_link_parts(
        parts_group: DOG,
        name: str,
        kk_frame: SymoroKKFrame,
        cylinder_diameter_mm: float = 20.0,
        ) -> (Part, Part):
    """Return the parts representing a KK frame.

    Return the part fixed to the antecedent frame and the part
    fixed to the current frame (i.e. moving with its associated
    joint).
    Both parts start at the origin.

    Parameters
    ==========
    parts_group: The group to which the part will be added.
    name: The name of the part as well as the prefix for the
          objects within the part.
    kk_frame: The Khalil-Kleinfinger frame.

    """
    def cylinder(name: str, height: float, part: Part):
        cyl = add_object(part, 'Part::Cylinder', name)
        cyl.Radius = cylinder_diameter_mm / 2
        cyl.Height = max(abs(height * params_to_freecad), min_cyl_height_mm)
        # Rotate by 180° if height < 0.
        cyl.Placement = fc.Placement(
           fc.Vector(0.0, 0.0, 0.0),
           fc.Rotation(fc.Vector(1.0, 0.0, 0.0), 90.0 - copysign(90.0, height)),
        )
        return cyl

    def rot_x(angle_rad: float):
        return fc.Rotation(fc.Vector(1.0, 0.0, 0.0), degrees(angle_rad))

    def rot_z(angle_rad: float):
        return fc.Rotation(fc.Vector(0.0, 0.0, 1.0), degrees(angle_rad))

    def translate_x(distance_raw_unit: float):
        return fc.Placement(fc.Vector(params_to_freecad * distance_raw_unit, 0.0, 0.0), fc.Rotation())

    def translate_z(distance_raw_unit: float):
        return fc.Placement(fc.Vector(0.0, 0.0, params_to_freecad * distance_raw_unit), fc.Rotation())

    min_cyl_height_mm = 0.1
    params_to_freecad = 1000.0

    # Part to be fixed to the antecedent frame.
    part_pre = add_object(parts_group, 'App::Part', name)
    part_pre.Label = f'{name}_pre'
    cyl_pre_z = cylinder(f'{name}_pre_z', kk_frame.pre_tz, part_pre)
    # Rotate about the cylinder axis (z) to align the seam.
    cyl_pre_z.Placement *= rot_z(kk_frame.pre_rz)

    # Part to be fixed to the current frame.
    part = add_object(parts_group, 'App::Part', name)
    part.Label = name
    cyl_tx = cylinder(f'{name}_x', kk_frame.tx, part)
    rot_z_to_x = fc.Rotation(fc.Vector(0.0, 0.0, 1.0), fc.Vector(1.0, 0.0, 0.0))
    cyl_tx.Placement = (
            fc.Placement(
                fc.Vector(),
                rot_z_to_x,
              )
            * cyl_tx.Placement
            * rot_z(kk_frame.rx)  # Rotate about the cylinder axis (z).
    )
    cyl_tz = cylinder(f'{name}_z', kk_frame.tz, part)
    transform = rot_x(kk_frame.rx) * translate_x(kk_frame.tx)
    # Rotate also about the cylinder axis (z).
    cyl_tz.Placement = transform * cyl_tz.Placement * rot_z(kk_frame.rz)
    return part_pre, part


def _leafs(antecedents: list[int]) -> list[int]:
    """Return the indices of the leafs in the tree.

    >>> leafs([])
    []

    >>> leafs([0])
    [0]

    >>> leafs([0, 1, 2])
    [2]

    >>> leafs([0, 1, 0])
    [1, 2]

    >>> leafs([0, 0, 1])
    [1, 2]

    Parameters
    ==========
    antecedents: The list of antecedents for each node.
                 The index of the tree root is 0.
                 The root node is not part of the list.

    """
    uniques = set(antecedents)
    return [i - 1 for i in range(1, len(antecedents) + 1) if i not in uniques]
