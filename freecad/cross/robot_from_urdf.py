# Generates a CROSS::Robot from a URDF robot.

from __future__ import annotations

from dataclasses import dataclass
from math import degrees
from pathlib import Path
from typing import Any, List, Optional, Tuple
from typing import TYPE_CHECKING

import FreeCAD as fc
import FreeCADGui as fcgui
from freecad.cross.gui_utils import tr
from freecad.cross.urdf_loader import UrdfLoader
try:
    from PySide import QtGui, QtCore, QtWidgets
except:
    from PySide2 import QtGui, QtCore, QtWidgets

from freecad.cross.freecadgui_utils import get_progress_bar, set_collision_appearance
from .freecad_utils import add_object
from .freecad_utils import make_group
from .freecad_utils import warn
from .joint_proxy import make_joint
from .link_proxy import make_link
from .robot_proxy import make_robot
from .wb_utils import get_joints

try:
    from .urdf_parser_utils import axis_to_z
    from .urdf_parser_utils import obj_from_geometry
    from .urdf_parser_utils import placement_along_z_from_joint
    from .urdf_parser_utils import placement_from_origin
except ModuleNotFoundError:
    pass

if TYPE_CHECKING:
    try:
        from urdf_parser_py.urdf import Collision as UrdfCollision
        from urdf_parser_py.urdf import Joint as UrdfJoint
        from urdf_parser_py.urdf import Link as UrdfLink
        from urdf_parser_py.urdf import Material as UrdfMaterial
        from urdf_parser_py.urdf import Robot as UrdfRobot
        from urdf_parser_py.urdf import Visual as UrdfVisual
    except ModuleNotFoundError:
        UrdfCollision = Any
        UrdfJoint = Any
        UrdfLink = Any
        UrdfMaterial = Any
        UrdfRobot = Any
        UrdfVisual = Any

    # Stubs and typing hints.
    from .joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
    from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
    from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
    DO = fc.DocumentObject
    DOList = List[DO]
    DOG = fc.DocumentObjectGroup
    AppLink = DO  # TypeId == 'App::Link'
    AppPart = DO  # TypeId == 'App::Part'
    # List of UrdfVisual or List of UrdfCollision.
    VisualList = List[UrdfVisual]
    CollisionList = List[UrdfCollision]


from . import robot_from_urdf as current_file

urdf_filename = None
package_path = None
repository_path = None


@dataclass
class Color:
    """An RGBA color with float values ∈ [0, 1]."""
    r: float
    g: float
    b: float
    a: float

    def to_rgb_tuple(self) -> tuple[float, float, float]:
        return (self.r, self.g, self.b)

    def to_material(self) -> fc.Material:
        # TODO: Get the AmbientColor, SpecularColor, EmissiveColor, and
        # Shininess from FreeCAD's preferences.
        return fc.Material(
            DiffuseColor=self.to_rgb_tuple(),
            AmbientColor=(0.33, 0.33, 0.33),
            SpecularColor=(0.53, 0.53, 0.53),
            EmissiveColor=(0.00, 0.00, 0.00),
            Transparency=1.0 - self.a,
            Shininess=0.90,
        )

def robot_from_urdf_path(
        doc: fc.Document,
        filename_path,
        package_path = None,
        repository_path = None,
) -> CrossRobot:
    current_file.urdf_filename = filename_path
    current_file.package_path = package_path
    current_file.repository_path = repository_path
    urdf_robot = UrdfLoader.load_from_file(urdf_filename, current_file)
    robot = robot_from_urdf(doc, urdf_robot)

    return robot


def get_real_pkg_path(
        pkg_name: str,
        default_pkg_path_from_desc_pkg_path:bool = False,
) -> str | None:
    """Get real package path based on package_path, repository_path and pkg_name.
    Working with robot_descriptions module and useless if URDF is gotten not from it."""
    pkg_path = None
    # loaded from robot_descriptions
    if not pkg_path and current_file.package_path:
        last_path_segment = Path(current_file.package_path).name
        if last_path_segment == pkg_name:
            pkg_path = current_file.package_path

    # In some cases package_path can not be not really package path
    # and we check reposity path as package (f.e. Panda Gepetto)
    if not pkg_path and current_file.repository_path:
        last_path_segment = Path(current_file.repository_path).name
        if last_path_segment == pkg_name:
            pkg_path = current_file.repository_path

    # if we dont have pkg_name
    # just use package_path (in some cases can be not really package path)
    # it is problem of description packages integrated in robot_descriptions
    if default_pkg_path_from_desc_pkg_path and not pkg_path and current_file.package_path:
        pkg_path = current_file.package_path
    return pkg_path



def robot_from_urdf(
        doc: fc.Document,
        urdf_robot: UrdfRobot,
) -> CrossRobot:
    """Creates a CROSS::Robot from URDF."""
    doc.openTransaction(tr('Robot from URDF'))

    pkg_name = ''
    if current_file.package_path:
        pkg_name = 'of ' + Path(current_file.package_path).name
    progressBar = get_progress_bar(
        title = "Creating model based on URDF/xacro " + pkg_name + ". May take a few minutes...",
        min = 0,
        max = len(urdf_robot.links) + len(urdf_robot.joints) + 10,
    )
    progressBar.show()

    i = 0
    progressBar.setValue(i)
    QtGui.QApplication.processEvents()
    i += 1

    robot, parts_group = _make_robot(doc, urdf_robot.name)
    # Change the Show properties before having added all links.
    # Disable show of all for not creating any link to real, visial, collision
    # in progress of creating robot links
    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        robot.ViewObject.ShowReal = False
        robot.ViewObject.ShowVisual = False
        robot.ViewObject.ShowCollision = False

    progressBar.setValue(i)
    QtGui.QApplication.processEvents()
    i += 1

    colors = _get_colors(urdf_robot)
    progressBar.setValue(i)
    QtGui.QApplication.processEvents()
    i += 1


    for urdf_link in urdf_robot.links:
        progressBar.setValue(i)
        QtGui.QApplication.processEvents()
        i += 1
        ros_link, visual_part, collision_part, real_part = _add_ros_link(
            urdf_link, robot, parts_group,
        )

        geoms, geom_containers = _add_visual(
                urdf_link, parts_group, ros_link, visual_part, colors,
        )
        for geom in geoms:
            robot.Proxy.created_objects.append(geom)
        for geom_container in geom_containers:
            robot.Proxy.created_objects.append(geom_container)

        geoms, geom_containers = _add_real(
                urdf_link, parts_group, ros_link, real_part, colors, convert_mesh_to_solid = True,
        )
        for geom in geoms:
            robot.Proxy.created_objects.append(geom)
        for geom_container in geom_containers:
            robot.Proxy.created_objects.append(geom_container)

        geoms, geom_containers = _add_collision(
                urdf_link, parts_group, ros_link, collision_part, colors,
        )
        for geom in geoms:
            robot.Proxy.created_objects.append(geom)
        for geom_container in geom_containers:
            set_collision_appearance(geom_container)
            robot.Proxy.created_objects.append(geom_container)

    joint_map: dict[str, CrossJoint] = {}
    for urdf_joint in urdf_robot.joints:
        progressBar.setValue(i)
        QtGui.QApplication.processEvents()
        i += 1
        ros_joint = _add_ros_joint(urdf_joint, robot)
        joint_map[urdf_joint.name] = ros_joint
    # Mimic joints must be handled after creating all joints because the
    # mimicking joint can be defined before the mimicked joint in URDF.
    _define_mimic_joints(urdf_robot, joint_map)
    progressBar.setValue(i)
    QtGui.QApplication.processEvents()
    i += 2

    _compensate_joint_placement(robot, urdf_robot, joint_map)
    progressBar.setValue(i)
    QtGui.QApplication.processEvents()
    i += 3

    # Change the visual properties after having added all links.
    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        robot.ViewObject.ShowReal = False
        robot.ViewObject.ShowVisual = True
        robot.ViewObject.ShowCollision = False
    progressBar.setValue(i)
    QtGui.QApplication.processEvents()
    i += 2

    doc.recompute()
    progressBar.setValue(i)
    QtGui.QApplication.processEvents()

    progressBar.close()
    QtGui.QApplication.processEvents()
    doc.commitTransaction()
    return robot


def _make_robot(
        doc: fc.Document,
        name: str = 'robot',
) -> tuple[CrossRobot, DOG]:
    """Create a CROSS::Robot

    Return (robot object, parts group).

    The group called 'URDF Parts' is potentially created and returned. If the object
    'URDF Parts' is not a group, a different name will be given.

    """

    robot: CrossRobot = make_robot(name, doc)
    # Create a group 'Parts' to hold all parts in the assembly document.
    parts_group = make_group(doc, 'URDF Parts', visible=False)
    robot.Proxy.created_objects.append(parts_group)

    return robot, parts_group


def _get_colors(
        urdf_robot: UrdfRobot,
) -> dict[str, Color]:
    """
    Get the colors of the links from the URDF description.

    Return a dictionary ({material_name: Color}) with the available colors, the key is the material
    name, the value is the color.

    """
    colors: dict[str, Color] = {}
    if not hasattr(urdf_robot, 'materials'):
        return colors
    for material in urdf_robot.materials:
        color = _material_color(material)
        if (color and (material.name is not None)):
            colors[material.name] = color
    return colors


def _material_color(material: UrdfMaterial) -> Optional[Color]:
    def clamp(v: float) -> float:
        return max(0.0, min(1.0, v))

    if not hasattr(material, 'color'):
        return None

    if not material.color:
        return None

    try:
        r, g, b, a = [float(c) for c in material.color.rgba]
    except (AttributeError, IndexError, ValueError):
        r, g, b = [float(c) for c in material.color.rgba]
        a = 1.0
    except (AttributeError, IndexError, ValueError):
        return None
    return Color(
        clamp(r),
        clamp(g),
        clamp(b),
        clamp(a),
    )


def _add_ros_link(
        urdf_link: UrdfLink,
        robot: CrossRobot,
        parts_group: DOG,
) -> Tuple[CrossLink, AppPart, AppPart]:
    """Add three App::Part to the group and links to them to the robot.

    - Add an "App::Part" for the visual of each link.
    - Add an "App::Part for the collision of each link.
    - Add an "App::Part for the real of each link.
    - Add a "Cross::Link" with references to them.

    Return (CROSS::Link, part for visual, part for collision, part for real).

    Parameters
    ----------
    - urdf_link: link from the URDF description.
    - robot: robot to add the Cross::Link to.
    - part_groups: group for geometries and "App::Part" objects.

    """
    name = urdf_link.name
    doc = robot.Document

    visual_part = add_object(parts_group, 'App::Part', f'visual_{name}_')
    visual_part.Visibility = False
    robot.Proxy.created_objects.append(visual_part)

    real_part = add_object(parts_group, 'App::Part', f'real_{name}_')
    real_part.Visibility = False
    robot.Proxy.created_objects.append(real_part)

    collision_part = add_object(parts_group, 'App::Part', f'collision_{name}_')
    collision_part.Visibility = False
    robot.Proxy.created_objects.append(collision_part)

    ros_link = make_link(name, doc, recompute_after = False)
    ros_link.Label2 = name
    ros_link.adjustRelativeLinks(robot)
    _set_link_inertial(ros_link, urdf_link)
    robot.addObject(ros_link)

    ros_link.Visual = [visual_part]
    ros_link.Real = [real_part]
    ros_link.Collision = [collision_part]

    return ros_link, visual_part, collision_part, real_part


def _set_link_inertial(
        ros_link: CrossLink,
        urdf_link: UrdfLink,
) -> None:
    """Set the inertial properties of a Cross::Link.

    Parameters
    ----------
    - ros_link: link from the CROSS::Robot.
    - urdf_link: link from the URDF description.

    """
    if urdf_link.inertial is None:
        return
    if urdf_link.inertial.origin is not None:
        ros_link.CenterOfMass = placement_from_origin(urdf_link.inertial.origin)
    if urdf_link.inertial.mass is not None:
        # Both in kilogram.
        ros_link.Mass = urdf_link.inertial.mass
    if urdf_link.inertial.inertia is not None:
        # All in kg.m^2.
        ros_link.Ixx = urdf_link.inertial.inertia.ixx
        ros_link.Ixy = urdf_link.inertial.inertia.ixy
        ros_link.Ixz = urdf_link.inertial.inertia.ixz
        ros_link.Iyy = urdf_link.inertial.inertia.iyy
        ros_link.Iyz = urdf_link.inertial.inertia.iyz
        ros_link.Izz = urdf_link.inertial.inertia.izz


def _add_ros_joint(
        urdf_joint: UrdfJoint,
        robot: CrossRobot,
) -> CrossJoint:
    doc = robot.Document
    ros_joint = make_joint(urdf_joint.name, doc, recompute_after = False)
    ros_joint.Label2 = urdf_joint.name
    ros_joint.adjustRelativeLinks(robot)
    robot.addObject(ros_joint)
    ros_joint.Parent = urdf_joint.parent
    ros_joint.Child = urdf_joint.child
    ros_joint.Type = urdf_joint.type
    ros_joint.Origin = placement_along_z_from_joint(urdf_joint)
    if urdf_joint.limit is not None:
        if ros_joint.Proxy.get_unit_type() == 'Angle':
            factor = degrees(1.0)  # radians to degrees.
        elif ros_joint.Proxy.get_unit_type() == 'Length':
            factor = 1000.0  # meters to millimeters.
        else:
            factor = 1.0
        # All attributes of `limit` are compulsory.
        ros_joint.LowerLimit = factor * urdf_joint.limit.lower
        ros_joint.UpperLimit = factor * urdf_joint.limit.upper
        ros_joint.Effort = urdf_joint.limit.effort
        ros_joint.Velocity = urdf_joint.limit.velocity
    return ros_joint


def _define_mimic_joints(
        urdf_robot: UrdfRobot,
        joint_map: dict[str, CrossJoint],
) -> None:
    """Add the correct properties for mimic joints."""
    for name, ros_joint in joint_map.items():
        urdf_joint = urdf_robot.joint_map[name]
        if urdf_joint.mimic is None:
            continue
        if urdf_joint.type not in ['prismatic', 'revolute', 'continuous']:
            warn(
                f'Mimicking joint "{urdf_joint.name}" has type '
                f'{urdf_joint.type} but only prismatic, revolute,'
                ' and continuous types are supported, ignoring "mimic"', False,
            )
            continue
        ros_joint.Mimic = True
        mimicked_urdf_joint = urdf_robot.joint_map.get(urdf_joint.mimic.joint)
        if not mimicked_urdf_joint:
            warn(
                f'Joint "{urdf_joint.name}" mimics the unknown'
                f' joint "{mimicked_urdf_joint}", ignoring', True,
            )
            continue
        mimicked_ros_joint = joint_map.get(mimicked_urdf_joint.name)
        # `mimicked_ros_joint` should not be None.
        ros_joint.MimickedJoint = mimicked_ros_joint
        if urdf_joint.mimic.multiplier is not None:
            ros_joint.Multiplier = urdf_joint.mimic.multiplier
        if urdf_joint.mimic.offset is None:
            offset = 0.0
        else:
            if urdf_joint.type == 'prismatic':
                # Meters (URDF) to millimeters (FreeCAD).
                offset = urdf_joint.mimic.offset * 1000.0
            else:
                # urdf_joint.type in ['revolute', 'continuous']
                # Radians (URDF) to degrees (FreeCAD).
                offset = degrees(urdf_joint.mimic.offset)
        ros_joint.Offset = offset


def _compensate_joint_placement(
        robot: CrossRobot,
        urdf_robot: UrdfRobot,
        joint_map: dict[str, CrossJoint],
) -> None:
    """Make all joints about/around the z axis."""
    chains = robot.Proxy.get_chains()
    already_compensated_joints: set[CrossJoint] = set()
    for chain in chains:
        ros_joints = get_joints(chain)
        previous_rotation_to_z = fc.Rotation()
        for ros_joint in ros_joints:
            name = list(joint_map.keys())[list(joint_map.values()).index(ros_joint)]
            urdf_joint = urdf_robot.joint_map[name]
            rotation_to_z = axis_to_z(urdf_joint)
            if ros_joint in already_compensated_joints:
                previous_rotation_to_z = rotation_to_z
                continue
            already_compensated_joints.add(ros_joint)
            _set_child_placement(robot, urdf_joint, ros_joint)
            ros_joint.Origin = previous_rotation_to_z.inverted() * ros_joint.Origin
            previous_rotation_to_z = rotation_to_z


def _set_child_placement(
        robot: CrossRobot,
        urdf_joint: UrdfJoint,
        ros_joint: CrossJoint,
) -> None:
    """Set Child.MountedPlacement to compensate for joints not along z."""
    if not ros_joint.Child:
        return
    link = robot.Proxy.get_link(ros_joint.Child)
    if link:
        link.MountedPlacement.Rotation = axis_to_z(urdf_joint).inverted()


def _add_visual(
        urdf_link: UrdfLink,
        parts_group: DOG,
        ros_link: CrossLink,
        visual_part: AppPart,
        colors: dict[str, Color],
        convert_mesh_to_solid: bool = False,
) -> tuple[DOList, DOList]:
    """Add the visual geometries to a robot.

    Return the list of objects representing the geometries and the list of
    FreeCAD links.

    Parameters
    ==========

    - All parameters of `_add_geometries` except `name_linked_geom`.

    """
    name_linked_geom = f'{urdf_link.name}_visual'
    return _add_geometries(
        parts_group,
        ros_link,
        visual_part,
        urdf_link.visuals,
        name_linked_geom,
        colors,
        convert_mesh_to_solid,
    )


def _add_real(
        urdf_link: UrdfLink,
        parts_group: DOG,
        ros_link: CrossLink,
        real_part: AppPart,
        colors: dict[str, Color],
        convert_mesh_to_solid: bool = False,
) -> tuple[DOList, DOList]:
    """Add the real geometries to a robot.

    Return the list of objects representing the geometries and the list of
    FreeCAD links.

    Parameters
    ==========

    - All parameters of `_add_geometries` except `name_linked_geom`.

    """
    name_linked_geom = f'{urdf_link.name}_real'
    return _add_geometries(
        parts_group,
        ros_link,
        real_part,
        urdf_link.visuals,
        name_linked_geom,
        colors,
        convert_mesh_to_solid,
    )


def _add_collision(
        urdf_link: UrdfLink,
        parts_group: DOG,
        ros_link: CrossLink,
        collision_part: AppPart,
        colors: dict[str, Color],
        convert_mesh_to_solid: bool = False,
) -> tuple[DOList, DOList]:
    """Add the collision geometries to a robot.

    Return the list of objects representing the geometries and the list of
    FreeCAD links.

    Parameters
    ==========

    - All parameters of `_add_geometries` except `name_linked_geom`.

    """
    name_linked_geom = f'{urdf_link.name}_collision'
    return _add_geometries(
        parts_group,
        ros_link,
        collision_part,
        urdf_link.collisions,
        name_linked_geom,
        colors,
        convert_mesh_to_solid,
    )


def _material(
        visual: UrdfVisual,
        colors: dict[str, Color],
) -> Optional[fc.Material]:
    if not hasattr(visual, 'material'):
        return None

    if not visual.material:
        return None

    # Inline material.
    color = _material_color(visual.material)
    if color:
        return color.to_material()

    # Reference to a material defined externally.
    if not hasattr(visual.material, 'name'):
        # Should not happen, just in case.
        return None

    if visual.material.name not in colors:
        warn(f'Material "{visual.material.name}" not found in URDF description.', gui=False)
        return None

    return colors[visual.material.name].to_material()


def _add_geometries(
        parts_group: DOG,
        ros_link: DOG,
        part: AppPart,
        geometries: [VisualList | CollisionList],
        name_linked_geom: str,
        colors: dict[str, Color],
        convert_mesh_to_solid: bool = False,
) -> tuple[DOList, DOList]:
    """Add the geometries from URDF into `group` and an App::Link to it into `link`.

    `geometries` is either `visuals` or `collisions` and the geometry itself is
    `geometries[?].geometry`.
    If `name_linked_geom` is empty, not FC link is created in `link`.

    Return the list of objects representing the geometries and the list of
    FreeCAD links.

    Parameters
    ==========

    - group: an "App::DocumentObjectGroup" where the generated FreeCAD objects
             will be placed.
    - geometries: list of URDF geometries.
    - link: an "App::Link" object linking to an "App::Part" object representing
            the visual- or collision geometries of a URDF link.
    - name_linked_geom: base pattern for the generated "App::Link" objects
                        generated. The final name may then be
                        `name_linked_geom`, `name_linked_geom`001, ...
    - colors: a dictionary {material_name: Color} with the available colors.

    """
    geom_objs: DOList = []
    geom_containers: DOList = []
    for geometry in geometries:
        # Make the FC object in the group.
        try:
            geom_obj, _ = obj_from_geometry(geometry.geometry, parts_group, convert_mesh_to_solid, min_vol_instead_zero = True)
        except NotImplementedError:
            continue
        if not geom_obj:
            warn(f'Error when importing geometry for {ros_link.Label}')
            continue

        # Set the color.
        material = _material(geometry, colors)
        if material:
            try:
                geom_obj.ViewObject.ShapeAppearance = (material,)
            except (AttributeError, IndexError):
                pass

        geom_obj.Visibility = False
        geom_objs.append(geom_obj)

        # Add a reference to geom_obj to `ros_link.Visual` or
        # `ros_link.Collision`.
        link_to_geom = add_object(part, 'App::Link', name_linked_geom)
        link_to_geom.setLink(geom_obj)

        if hasattr(geometry, 'origin'):
            placement = (
                placement_from_origin(geometry.origin)
                * geom_obj.Placement
            )
        else:
            placement = geom_obj.Placement
        # geom_obj.Placement = fc.Placement()
        link_to_geom.Placement = placement
        geom_containers.append(link_to_geom)
    return geom_objs, geom_containers
