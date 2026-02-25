"""Utility function specific to this workbench."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, List, Optional, Protocol, Union
import os
import subprocess
import typing

import FreeCAD as fc
import FreeCADGui as fcgui

from . import wb_constants
if typing.TYPE_CHECKING:
    try:
        from geometry_msgs.msg import Pose
    except ImportError:
        Pose = Any

from . import wb_globals
from .freecad_utils import get_objs_from_selection_objs, get_param, get_parents_names, get_selected_shape_object, is_link_to_assembly_from_assembly_wb, is_selection_object, parse_freecad_path
from .gui_utils import tr
from .freecad_utils import is_box
from .freecad_utils import is_cylinder
from .freecad_utils import is_sphere
from .freecad_utils import is_lcs
from .freecad_utils import is_part
from .freecad_utils import is_link as is_fc_link
from .freecad_utils import message
from .freecad_utils import set_param
from .freecad_utils import warn
from .freecad_utils import lcs_attachmentsupport_name
from .freecadgui_utils import get_placement, get_progress_bar, gui_process_events
from .ros.utils import get_ros_workspace_from_file
from .ros.utils import without_ros_workspace
from .utils import attr_equals, calc_md5
from .utils import values_from_string
from .utils import get_valid_filename
from .exceptions import NoPartWrapperOfObject
from .freecad_utils import validate_types

# Stubs and typing hints.
from .attached_collision_object import AttachedCollisionObject as CrossAttachedCollisionObject  # A Cross::AttachedCollisionObject, i.e. a DocumentObject with Proxy "AttachedCollisionObject". # noqa: E501
from .joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
from .workcell import Workcell as CrossWorkcell  # A Cross::Workcell, i.e. a DocumentObject with Proxy "Workcell". # noqa: E501
from .xacro_object import XacroObject as CrossXacroObject  # A Cross::XacroObject, i.e. a DocumentObject with Proxy "XacroObject". # noqa: E501
from .controller import Controller as CrossController  # A Cross::Controller, i.e. a DocumentObject with Proxy "Controller". # noqa: E501
from .sensors.sensor import Sensor as CrossSensor  # A Cross::Sensor, i.e. a DocumentObject with Proxy "SensorProxyJoint" or "SensorProxyJoint". # noqa: E501

DO = fc.DocumentObject
CrossBasicElement = Union[CrossJoint, CrossLink]
CrossObject = Union[CrossJoint, CrossLink, CrossRobot, CrossXacroObject, CrossWorkcell, CrossController, CrossSensor]
DOList = List[DO]

# /home/use/.local/share/FreeCAD/Mod/ # this used when RobotCAD installed like Addon Manager do
MOD_PATH_IN_USER_SHARE_DIR = Path(fc.getUserAppDataDir()) / 'Mod/freecad.robotcad'
# near basic modules like Part, BIM # this used when RobotCAD packed with root modules for install via 1 archive
MOD_PATH_IN_ROOT_MODULES_DIR = Path(fc.getResourceDir()) / 'Mod/freecad.robotcad' 
if os.path.exists(MOD_PATH_IN_USER_SHARE_DIR):
    MOD_PATH = MOD_PATH_IN_USER_SHARE_DIR
else:
    MOD_PATH = MOD_PATH_IN_ROOT_MODULES_DIR

RESOURCES_PATH = MOD_PATH / 'resources'
UI_PATH = RESOURCES_PATH / 'ui'
ICON_PATH = RESOURCES_PATH / 'icons'
MODULES_PATH = MOD_PATH / 'modules'

ROS2_CONTROLLERS_PATH = MOD_PATH / 'modules' / 'ros2_controllers'
SDFORMAT_PATH = MOD_PATH / 'modules' / 'sdformat'
SDFORMAT_SDF_TEMPLATES_PATH = MOD_PATH / 'modules' / 'sdformat' / 'sdf'
ROBOT_DESCRIPTIONS_REPO_PATH = MOD_PATH / 'modules' / 'robot_descriptions'
ROBOT_DESCRIPTIONS_MODULE_PATH = ROBOT_DESCRIPTIONS_REPO_PATH / 'robot_descriptions'
DYNAMIC_WORLD_GENERATOR_REPO_PATH = MODULES_PATH / 'Dynamic_World_Generator'
DYNAMIC_WORLD_GENERATOR_MODULE_PATH = DYNAMIC_WORLD_GENERATOR_REPO_PATH / 'code'
DYNAMIC_WORLD_GENERATOR_WORLDS_GAZEBO_PATH = DYNAMIC_WORLD_GENERATOR_REPO_PATH / 'worlds' / 'gazebo'

SENSORS_DATA_PATH = MOD_PATH / 'resources' / 'sensors'
LINK_SENSORS_DATA_PATH = MOD_PATH / 'resources' / 'sensors' / 'link'
JOINT_SENSORS_DATA_PATH = MOD_PATH / 'resources' / 'sensors' / 'joint'
ROBOT_SENSORS_DATA_PATH = MOD_PATH / 'resources' / 'sensors' / 'robot'


class SupportsStr(Protocol):
    def __str__(self) -> str:
        ...


@dataclass
class XacroObjectAttachment:
    xacro_object: CrossXacroObject
    attached_to: Optional[CrossLink] = None
    attached_by: Optional[CrossJoint] = None
    # ROS object `attached_to` belongs to.
    attachement_ros_object: Optional[CrossXacroObject | CrossRobot] = None


def get_workbench_param(
    param_name: str,
    default: Any,
) -> Any:
    """Return the value of a workbench parameter."""
    param_grp = fc.ParamGet(
            f'User parameter:BaseApp/Preferences/Mod/{wb_globals.PREFS_CATEGORY}',
    )
    return get_param(param_grp, param_name, default)


def set_workbench_param(
    param_name: str,
    value: Any,
) -> None:
    """Set the value of a workbench parameter."""
    param_grp = fc.ParamGet(
            f'User parameter:BaseApp/Preferences/Mod/{wb_globals.PREFS_CATEGORY}',
    )
    set_param(param_grp, param_name, value)


def is_attached_collision_object(obj: DO) -> bool:
    """Return True if the object is a Cross::AttachedCollisionObject."""
    return _has_ros_type(obj, 'Cross::AttachedCollisionObject')


def is_robot(obj: DO) -> bool:
    """Return True if the object is a Cross::Robot."""
    return _has_ros_type(obj, 'Cross::Robot')


def is_link(obj: DO) -> bool:
    """Return True if the object is a Cross::Link."""
    return _has_ros_type(obj, 'Cross::Link')


def is_joint(obj: DO) -> bool:
    """Return True if the object is a Cross::Link."""
    return _has_ros_type(obj, 'Cross::Joint')


def is_xacro_object(obj: DO) -> bool:
    """Return True if the object is a Cross::Xacro."""
    return _has_ros_type(obj, 'Cross::XacroObject')


def is_workcell(obj: DO) -> bool:
    """Return True if the object is a Cross::Workcell."""
    return _has_ros_type(obj, 'Cross::Workcell')


def is_controller(obj: DO) -> bool:
    """Return True if the object is a Cross::Controller."""
    return _has_ros_type(obj, 'Cross::Controller')


def is_broadcaster(obj: DO) -> bool:
    """Return True if the object is a Cross::Broadcaster."""
    return _has_ros_type(obj, 'Cross::Broadcaster')


def is_sensor(obj: DO) -> bool:
    """Return True if the object is a Cross::Sensor."""
    return is_sensor_link(obj) or is_sensor_joint(obj) or _has_ros_type(obj, 'Cross::Sensor')


def is_sensor_link(obj: DO) -> bool:
    """Return True if the object is a Cross::SensorLink."""
    return _has_ros_type(obj, 'Cross::SensorLink')


def is_sensor_joint(obj: DO) -> bool:
    """Return True if the object is a Cross::SensorJoint."""
    return _has_ros_type(obj, 'Cross::SensorJoint')


def is_planning_scene(obj: DO) -> bool:
    """Return True if the object is a Cross::PlanningScene."""
    return _has_ros_type(obj, 'Cross::PlanningScene')


def is_simple_joint(obj: DO) -> bool:
    """Return True if prismatic, revolute, or continuous."""
    return (
        is_joint(obj)
        and (obj.Type in ['prismatic', 'revolute', 'continuous'])
    )


def is_primitive(obj: DO) -> bool:
    """Return True if the object is a 'Part::{Box,Cylinder,Sphere}'."""
    return is_box(obj) or is_sphere(obj) or is_cylinder(obj)


def is_controllers_template_for_param_mapping(param_full_name: str) -> bool:
    """Return True param_full_name contains marker of param mapping template."""
    if wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE + wb_constants.ROS2_CONTROLLERS_PARAM_MAP_MARKER in param_full_name:
        return True
    return False


def is_placement(obj: DO) -> bool:
    """Return True if the object is a FreeCAD Placement."""
    return isinstance(obj, fc.Placement)


def return_true(obj: DO) -> bool:
    """Return always True.

    It is dummy for checking of elements without real checking function.
    Used for maintain same workflow as other element with checking function."""

    return True


def is_robot_selected() -> bool:
    """Return True if the first selected object is a Cross::Robot."""
    return is_selected_from_lambda(is_robot)


def is_joint_selected() -> bool:
    """Return True if the first selected object is a Cross::Joint."""
    return is_selected_from_lambda(is_joint)


def is_link_selected() -> bool:
    """Return True if the first selected object is a Cross::Link."""
    return is_selected_from_lambda(is_link)


def is_controller_selected() -> bool:
    """Return True if the first selected object is a Cross::Controller."""
    return is_selected_from_lambda(is_controller)


def is_broadcaster_selected() -> bool:
    """Return True if the first selected object is a Cross::Broadcaster."""
    return is_selected_from_lambda(is_broadcaster)


def is_sensor_selected() -> bool:
    """Return True if the first selected object is a Cross::Sensor."""
    return is_selected_from_lambda(is_sensor)


def is_workcell_selected() -> bool:
    """Return True if the first selected object is a Cross::Workcell."""
    return is_selected_from_lambda(is_workcell)


def is_planning_scene_selected() -> bool:
    """Return True if the first selected object is a Cross::PlanningScene."""
    return is_selected_from_lambda(is_planning_scene)


def get_attached_collision_objects(objs: DOList) -> list[CrossAttachedCollisionObject]:
    """Return only the objects that are Cross::AttachedCollisionObject instances."""
    return [o for o in objs if is_attached_collision_object(o)]


def get_links(objs: DOList) -> list[CrossLink]:
    """Return only the objects that are Cross::Link instances."""
    return [o for o in objs if is_link(o)]


def get_joints(objs: DOList) -> list[CrossJoint]:
    """Return only the objects that are Cross::Joint instances."""
    return [o for o in objs if is_joint(o)]


def get_child_joints(link_or_joint: DO) -> list[CrossJoint] | bool:
    """Return first level child joints of link or joint."""
    if is_joint(link_or_joint):
        link_name = link_or_joint.Child
    elif is_link(link_or_joint):
        link_name = link_or_joint.Name
    else:
        return False

    robot = link_or_joint.Proxy.get_robot()
    robot_joints = robot.Proxy.get_joints()
    child_joints = []
    for joint in robot_joints:
        if link_name == joint.Parent:
            child_joints.append(joint)

    return child_joints


def get_link_sensors(objs: DOList) -> list[CrossSensor]:
    """Return only the objects that are Cross::SensorLink instances."""
    return [o for o in objs if is_sensor_link(o)]


def get_joint_sensors(objs: DOList) -> list[CrossSensor]:
    """Return only the objects that are Cross::SensorJoint instances."""
    return [o for o in objs if is_sensor_joint(o)]


def get_controllers(objs: DOList) -> list[CrossController]:
    """Return only the objects that are Cross::Controller instances."""
    return [o for o in objs if is_controller(o)]


def get_broadcasters(objs: DOList) -> list[CrossController]:
    """Return only the objects that are Cross::Controller instances."""
    return [o for o in objs if is_broadcaster(o)]


def get_xacro_objects(objs: DOList) -> list[CrossXacroObject]:
    """Return only the objects that are Cross::XacroObject instances."""
    return [o for o in objs if is_xacro_object(o)]


def get_chains(
        links: list[CrossLink],
        joints: list[CrossJoint],
        check_kinematics = True,
) -> list[list[CrossBasicElement]]:
    """Return the list of chains.

    A chain starts at the root link, alternates links and joints, and ends
    at the last joint of the chain.

    If the last element of a chain would be a joint, that chain is not
    considered.

    """
    # TODO: Make the function faster.
    base_links: list[CrossLink] = []
    tip_links: list[CrossLink] = []

    for link in links:
        if link.Proxy.may_be_base_link():
            if check_kinematics == False: # avoid silent bug https://github.com/drfenixion/freecad.cross/issues/5
                if len(base_links) < 1:
                    base_links.append(link)
            else:
                base_links.append(link)
        if link.Proxy.is_tip_link():
            tip_links.append(link)
    if len(base_links) > 1:
        # At least two root links found, not supported.
        return []
    chains: list[list[CrossBasicElement]] = []
    for link in tip_links:
        chain = get_chain(link)
        if chain:
            chains.append(chain)
    return chains


def get_chain(link: CrossLink) -> list[CrossBasicElement]:
    """Return the chain from base link to link, excluded.

    The chain starts with the base link, then alternates a joint and a link.
    The last item is the joint that has `link` as child.

    """
    chain: list[CrossBasicElement] = []
    ref_joint = link.Proxy.get_ref_joint()
    if not ref_joint:
        # A root link.
        return [link]
    if not ref_joint.Parent:
        warn(f'Joint `{ros_name(ref_joint)}` has no parent', False)
        # Return only ref_joint to indicate an error.
        return [ref_joint]
    robot = ref_joint.Proxy.get_robot()
    subchain = get_chain(robot.Proxy.get_link(ref_joint.Parent))
    if subchain and is_joint(subchain[0]):
        # Propagate the error of missing joint.Parent.
        return subchain
    chain += subchain + [ref_joint] + [link]
    return chain


def is_subchain(subchain: DOList, chain: DOList) -> bool:
    """Return True if all items in `subchain` are in `chain`."""
    for link_or_joint in subchain:
        if link_or_joint not in chain:
            return False
    return True


def get_parent_link_of_obj(obj: DO) -> bool | CrossLink:
    """Return first parent CrossLink of object or None."""
    for parent in reversed(obj.InListRecursive):
        if is_link(parent):
            return parent
    return None


def get_xacro_object_attachments(
        xacro_objects: list[CrossXacroObject],
        joints: list[CrossJoint],
) -> list[XacroObjectAttachment]:
    """Return attachment details of xacro objects."""
    attachments: list[XacroObjectAttachment] = []
    for xacro_object in xacro_objects:
        attachment = XacroObjectAttachment(xacro_object)
        attachments.append(attachment)
        if not hasattr(xacro_object, 'Proxy'):
            continue
        root_link = xacro_object.Proxy.root_link
        for joint in joints:
            if joint.Child == root_link:
                attachment.attached_by = joint
        if not attachment.attached_by:
            continue
        for xo in [x for x in xacro_objects if x is not xacro_object]:
            parent: str = attachment.attached_by.Parent
            if xo.Proxy.has_link(parent):
                attachment.attached_to = xo.Proxy.get_link(parent)
                attachment.attachement_ros_object = xo
    return attachments


def get_xacro_chains(
        xacro_objects: list[CrossXacroObject],
        joints: list[CrossJoint],
) -> list[list[XacroObjectAttachment]]:
    """Return the list of chains.

    A chain starts at a xacro object that is not attached to any other xacro
    object and contains all xacro objects that form an attachment chain, up to
    the xacro object to which no other xacro object is attached.

    """
    def is_parent(
        xacro_object: CrossXacroObject,
        attachments: list[XacroObjectAttachment],
    ) -> bool:
        for attachment in attachments:
            if attachment.attachement_ros_object is xacro_object:
                return True
        return False

    def get_chain(
        xacro_object: CrossXacroObject,
        attachments: list[XacroObjectAttachment],
    ) -> list[XacroObjectAttachment]:
        for attachment in attachments:
            if attachment.xacro_object is xacro_object:
                break
        if attachment.attachement_ros_object:
            return (
                get_chain(attachment.attachement_ros_object, attachments)
                + [attachment]
            )
        else:
            return [attachment]

    attachments = get_xacro_object_attachments(xacro_objects, joints)
    tip_xacros: list[CrossXacroObject] = []
    for attachment in attachments:
        if not is_parent(attachment.xacro_object, attachments):
            tip_xacros.append(attachment.xacro_object)
    chains: list[list[XacroObjectAttachment]] = []
    for xacro_object in tip_xacros:
        chains.append(get_chain(xacro_object, attachments))
    return chains


def ros_name(obj: DO) -> str:
    """Return in order obj.Label2, obj.Label, obj.Name."""
    if ((
        not hasattr(obj, 'isDerivedFrom')
        or (not obj.isDerivedFrom('App::DocumentObject'))
    )):
        return 'not_a_FreeCAD_object'
    if obj.Label2:
        return obj.Label2
    if obj.Label:
        return obj.Label
    return obj.Name


def get_valid_urdf_name(name: str) -> str:
    if not name:
        return 'no_name'
    # TODO: special XML characters must be escaped.
    return name.replace('"', '&quot;')


def get_rel_and_abs_path(path: str, ask_user_fill_workspace: bool = True) -> tuple[str, Path]:
    """Return the path relative to src and the absolute path.

    Return the path relative to the `src` folder in the  ROS workspace and
    the absolute path to the file.
    The input path can be a path relative to the ROS workspace or an absolute
    path.
    If the input path is relative, it is returned as-is.

    For example, if the file path is `my_package/file.py`, return
    `('my_package/file.py', Path('/home/.../ros2_ws/src/my_package/file.py`)`,
    supposing that `my_package` is a ROS package in the ROS workspace
    `/home/.../ros2_ws`.

    The existence of the given path is not checked.

    If `wb_globals.g_ros_workspace` is not set, ask the user to configure it.

    """
    # Import here to avoid circular import.
    from .wb_gui_utils import get_ros_workspace

    if ask_user_fill_workspace:
        if not wb_globals.g_ros_workspace.name:
            ws = get_ros_workspace()
            wb_globals.g_ros_workspace = ws
    p = without_ros_workspace(path)
    full_path = (
        wb_globals.g_ros_workspace.expanduser() / 'src' / p
    )
    return p, full_path


def remove_ros_workspace(path) -> str:
    """Modify `path` to remove $ROS_WORKSPACE/src.

    Return the path relative to `wb_globals.g_ros_workspace / 'src'`.
    Doesn't use any ROS functionalities and, thus, doesn't support finding any
    underlay workspace.
    Modify `wb_globals.g_ros_workspace` if a workspace was guessed from `path`.

    """
    rel_path = without_ros_workspace(path)
    if wb_globals.g_ros_workspace.samefile(Path()):
        # g_ros_workspace was not defined yet.
        ws = get_ros_workspace_from_file(path)
        if not ws.samefile(Path()):
            # A workspace was found.
            wb_globals.g_ros_workspace = ws
            message(
                'ROS workspace was set to'
                f' {wb_globals.g_ros_workspace},'
                ' change if not correct.'
                ' Note that packages in this workspace will NOT be'
                ' found, though, but only by launching FreeCAD from a'
                ' sourced workspace',
                True,
            )
            rel_path = without_ros_workspace(path)
    return rel_path


def export_templates(
        template_files: list[str],
        package_parent: [Path | str],
        **keys: SupportsStr,
) -> None:
    """Export generated files.

    Parameters
    ----------

    - template_files: list of files to export, relative to
                      `RESOURCES_PATH/templates`, where RESOURCES_PATH is the
                      directory `resources` of this workbench.
    - package_name: the directory containing the directory called
                    `package_name`, usually "$ROS_WORKSPACE/src".
    - keys: dictionary of replacement string in templates.
            - package_name (compulsory): name of the ROS package and its containing
                                         directory.
            - urdf_file: name of the URDF/xacro file without directory.
                         Used in `launch/display.launch.py`.
            - fixed_frame: parameter "Global Options / Fixed Frame" in RViz.

    """
    try:
        package_name: str = keys['package_name']
    except KeyError:
        raise RuntimeError('Parameter "package_name" must be given')

    package_parent = Path(package_parent)

    try:
        meshes_dir
    except NameError:
        meshes_dir = (
            'meshes '
            if _has_meshes_directory(package_parent, package_name)
            else ''
        )

    for f in template_files:
        template_file_path = RESOURCES_PATH / 'templates' / f
        template = template_file_path.read_text()
        txt = template.format(**keys)

        xacro_wrapper_tmpl = 'xacro_wrapper_template.urdf.xacro'
        #replace xacro wrapper file name
        if xacro_wrapper_tmpl in f:
            f = f.replace(xacro_wrapper_tmpl, keys['xacro_wrapper_file'])

        sensors_tmpl = 'sensors_template.urdf.xacro'
        #replace sensors_template file name
        if sensors_tmpl in f:
            f = f.replace(sensors_tmpl, keys['sensors_file'])

        output_path = package_parent / package_name / f
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(txt)


def _has_ros_type(obj: DO, type_: str) -> bool:
    """Return True if the object is an object from this workbench."""
    if not isinstance(obj, DO):
        return False
    return attr_equals(obj, '_Type', type_)


def _has_meshes_directory(
        package_parent: [Path | str],
        package_name: str,
) -> bool:
    """Return True if the directory "meshes" exists in the package."""
    meshes_directory = Path(package_parent) / package_name / 'meshes'
    return meshes_directory.exists()


def is_selected_from_lambda(
        is_type_fun: Callable[DO, bool],
) -> bool:
    """Return True if the first selected object meets the given criteria.

    Return `is_type_fun("first_selected_object")`.

    """
    if not fc.GuiUp:
        return False
    if fc.activeDocument() is None:
        return False
    import FreeCADGui as fcgui
    sel = fcgui.Selection.getSelection()
    if not sel:
        return False
    return is_type_fun(sel[0])


def is_name_used(
        obj: CrossObject,
        container_obj: [CrossRobot | CrossWorkcell],
) -> bool:
    if not is_robot(container_obj) or is_workcell(container_obj):
        raise RuntimeError(
            'Second argument must be a'
            ' CrossRobot or a CrossWorkbench',
        )
    obj_name = ros_name(obj)
    if ((obj is not container_obj)
            and (ros_name(container_obj) == obj_name)):
        return True
    if is_robot(container_obj) and hasattr(container_obj, 'Proxy'):
        for link in container_obj.Proxy.get_links():
            if ((obj is not link)
                    and (ros_name(link) == obj_name)):
                return True
    elif is_workcell(container_obj) and hasattr(container_obj, 'Proxy'):
        for xacro_object in container_obj.Proxy.get_xacro_objects():
            if ((obj is not xacro_object)
                    and (ros_name(xacro_object) == obj_name)):
                return True
            if hasattr(xacro_object, 'Proxy'):
                robot = xacro_object.Proxy.get_robot()
                if robot and is_name_used(obj, robot):
                    return True
    if hasattr(container_obj, 'Proxy'):
        for joint in container_obj.Proxy.get_joints():
            if ((obj is not joint)
                    and (ros_name(joint) == obj_name)):
                return True
    return False


def placement_from_geom_pose(pose: Pose) -> fc.Placement:
    """Return a FreeCAD Placement from a ROS Pose."""
    ros_to_freecad_factor = 1000.0  # ROS uses meters, FreeCAD uses mm.
    return fc.Placement(
        fc.Vector(
            pose.position.x * ros_to_freecad_factor,
            pose.position.y * ros_to_freecad_factor,
            pose.position.z * ros_to_freecad_factor,
        ),
        fc.Rotation(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ),
    )


def placement_from_pose_string(pose: str) -> fc.Placement:
    """Return a FreeCAD Placement from a string pose `x, y, z; qw, qx, qy, qz`.

    The pose is a string of 2 semi-colon-separated groups: 3 floats
    representing the position in meters and 4 floats for the orientation
    as quaternions qw, qx, qy, qz. The values in a group can be separated
    by commas or spaces.
    A string of 7 floats is also accepted.

    """
    if ';' in pose:
        position_str, orientation_str = pose.split(';')
        try:
            x, y, z = values_from_string(position_str)
            qw, qx, qy, qz = values_from_string(orientation_str)
        except ValueError:
            raise ValueError('Pose must have the format `x, y, z; qw, qx, qy, qz`')
    else:
        try:
            x, y, z, qw, qx, qy, qz = values_from_string(pose)
        except ValueError:
            raise ValueError(
                    'Pose must have the format `x, y, z; qw, qx, qy, qz`'
                    ' or `x, y, z, qw, qx, qy, qz`',
            )
    ros_to_freecad_factor = 1000.0  # ROS uses meters, FreeCAD uses mm.
    return fc.Placement(
        fc.Vector(x, y, z) * ros_to_freecad_factor,
        fc.Rotation(qw, qx, qy, qz),
    )


def get_urdf_path(robot: CrossRobot, output_path: str) -> Path:
    """Return URDF file path`.
    """

    file_base = get_valid_filename(ros_name(robot))
    urdf_file = f'{file_base}.urdf'
    urdf_path = output_path / f'urdf/{urdf_file}'
    urdf_path = Path(urdf_path)

    return urdf_path


def get_xacro_wrapper_path(robot: CrossRobot, output_path: str) -> Path:
    """Return xacro wrapper file path`.
    """

    xacro_wrapper_file = get_xacro_wrapper_file_name(ros_name(robot))
    xacro_wrapper_path = output_path / f'urdf/{xacro_wrapper_file}'
    xacro_wrapper_path = Path(xacro_wrapper_path)

    return xacro_wrapper_path


def get_robot_meta_path(path_to_overcross_meta_dir: str) -> Path:
    """Return robot meta file path`.
    """

    return Path(path_to_overcross_meta_dir + 'robot_meta.xml')


def get_controllers_config_path(robot: CrossRobot, path_to_overcross_meta_dir: str) -> Path:
    """Return ros2_control controllers config file path in source dir (overcross)`.
    """

    return Path(path_to_overcross_meta_dir + get_controllers_config_file_name(ros_name(robot)))


def set_placement_by_orienteer(doc: DO, link_or_joint: DO,
    origin_or_mounted_placement_name: str, orienteer1: DO, hold_downstream_chain: bool = False):
    """Set element (joint) placement (Origin) by orienteer.

    Set placement with orienteer placement value
    """

    # set_placement_by_orienteer() does not work for MountedPlacement of link
    # because link conjuction place in many cases not at origin (zero coordinates) of link
    # Instead of this use move_placement()

    placement1 = get_placement_of_orienteer(orienteer1, lcs_concentric_reversed = True)

    origin_or_mounted_placement_value = getattr(link_or_joint, origin_or_mounted_placement_name)
    element_basic_placement = link_or_joint.Placement * origin_or_mounted_placement_value.inverse()
    placement1_diff = element_basic_placement.inverse() * placement1

    old_placement_diff =  origin_or_mounted_placement_value.inverse() * placement1_diff
    
    setattr(link_or_joint, origin_or_mounted_placement_name, getattr(link_or_joint, origin_or_mounted_placement_name) * old_placement_diff)

    if hold_downstream_chain:
        child_joints = get_child_joints(link_or_joint)
        for joint in child_joints:
            joint.Origin = old_placement_diff.inverse() * joint.Origin

        if is_joint(link_or_joint):
            child_link = doc.getObject(link_or_joint.Child)
            child_link.MountedPlacement  = old_placement_diff.inverse() * child_link.MountedPlacement

    doc.recompute()


def move_placement(
    doc: DO, link_or_joint: DO, origin_or_mounted_placement_name: str,
    orienteer1: DO, orienteer2: DO, delete_created_objects:bool = True,
    element_local_placement_inverse: bool = False
):
    """Move element (joint or link) placement (Origin or Mounted placement).

    Move first orienteer to placement of second and second orienteer
    and element to positions relative their bind system (element and orienteers) before.
    This move does not change spatial relation between each other.
    """

    placement1 = get_placement_of_orienteer(orienteer1, delete_created_objects)
    placement2 = get_placement_of_orienteer(orienteer2, delete_created_objects, lcs_concentric_reversed = True)

    origin_or_mounted_placement_value = getattr(link_or_joint, origin_or_mounted_placement_name)
    element_basic_placement = link_or_joint.Placement * origin_or_mounted_placement_value.inverse()
    element_local_placement = getattr(link_or_joint, origin_or_mounted_placement_name)
    origin_placement1_diff = (element_basic_placement * element_local_placement).inverse() * placement1
    origin_placement2_diff = (element_basic_placement * element_local_placement).inverse() * placement2

    # do Origin move
    if element_local_placement_inverse:
        new_local_placement = (origin_placement2_diff * origin_placement1_diff.inverse()) * element_local_placement.inverse()
    else:
        # first orienteer come to second orienteer place and Origin respectively moved
        # in local frame every tool click will result Origin move because both orienteers moved and received new position
        new_local_placement = element_local_placement * origin_placement2_diff * origin_placement1_diff.inverse()
    setattr(link_or_joint, origin_or_mounted_placement_name, new_local_placement)

    doc.recompute()


def get_placement_of_orienteer(orienteer, delete_created_objects:bool = True, lcs_concentric_reversed:bool = False) \
    -> fc.Placement :
    '''Return placement of orienteer.
    If orienteer is not certain types it will make LCS with InertialCS map mode and use it'''

    orienteer_object = orienteer
    if is_selection_object(orienteer):
        orienteer_object = orienteer.Object
        parsed_path = parse_freecad_path(orienteer.SubElementNames, orienteer.Document)
        obj = parsed_path['object']

    if is_lcs(orienteer):
        placement = get_placement(orienteer)
    elif is_selection_object(orienteer) and parsed_path['datum_type'] in ['LCS', 'Point']:
        placement = get_placement(obj)
    # TODO process Vertex
    elif is_link(orienteer_object) or is_joint(orienteer_object):
        placement = orienteer_object.Placement
    elif is_placement(orienteer_object):
        placement = orienteer_object
    else:
        lcs, body_lcs_wrapper, placement = make_lcs_at_link_body(orienteer, delete_created_objects, lcs_concentric_reversed)

    return placement


def make_lcs_at_link_body(
        orienteer,
        delete_created_objects:bool = True,
        lcs_concentric_reversed:bool = False,
        deactivate_after_map_mode:bool = True,
) -> list[fc.DO, fc.DO, fc.Placement] :
    '''Make LCS at face of body of robot link.
    orienteer body must be wrapper by App::Part and be any of Real, Collision, Visual element of robot link'''

    # def getParentsPlacementRecursively(obj, placement = fc.Placement()):
    #     orienteer_parents_reversed = reversed(obj.Parents)
    #     parent_obj = None
    #     for parent in orienteer_parents_reversed:
    #         parent_splited = parent[1].split('.')
    #         if len(parent_splited) > 2:
    #             parent_obj = fc.ActiveDocument.getObject(parent_splited[0])
    #         break

    #     if parent_obj:
    #         placement = placement * parent_obj.Placement
    #         return getParentsPlacementRecursively(parent_obj, placement)
    #     else:
    #         return obj, placement


    link_to_obj = None
    dynamic_link_of_robot_link = None
    original_obj = None
    dynamic_link_of_robot_link = None
    original_obj_wrapper = None
    try:
        # if is_fc_link(orienteer.Object):
        #     # link to object (like Part::Feature)
        #     link_to_obj = orienteer.Object
        #     original_obj = link_to_obj.getLinkedObject(True)
        #     obj_for_getting_parents = link_to_obj
        #     link_to_obj_placement = link_to_obj.Placement
        # else:
        #     # consider fc_link to part design obj. In that case link will be deeper
        #     inListRecursive = orienteer.Object.InListRecursive
        #     inListRecursiveWithoutRobotAndRealLink = list(inListRecursive)[:-2]
        #     inListRecursiveWithoutRobotAndRealLink_dict = {index: value for index, value in enumerate(inListRecursiveWithoutRobotAndRealLink)}
        #     inListRecursiveWithoutRobotAndRealLink_dict_reversed = sorted(inListRecursiveWithoutRobotAndRealLink_dict.items(), reverse=True)
        #     inListRecursiveWithoutRobotAndRealLink_list_reversed = [item[1] for item in inListRecursiveWithoutRobotAndRealLink_dict_reversed]
        #     fc_link = next(
        #         filter(
        #             lambda element: is_fc_link(element),
        #             inListRecursiveWithoutRobotAndRealLink_list_reversed
        #         ),
        #         None
        #     )
        #     robot_link = next(
        #         filter(
        #             lambda element: is_link(element),
        #             inListRecursiveWithoutRobotAndRealLink_list_reversed
        #         ),
        #         None
        #     )
        #     try:
        #         # check correct fc_link is found
        #         if fc_link.LinkedObject.Name != robot_link.Real[0].Group[0].LinkedObject.Name:
        #             fc_link = None
        #     except (IndexError, AttributeError):
        #         fc_link = None
        #         pass

        #     if fc_link:
        #         link_to_obj = fc_link
        #         original_obj = link_to_obj.getLinkedObject(True)
        #         obj_for_getting_parents = link_to_obj
        #         link_to_obj_placement = link_to_obj.Placement                
        #     else:
        #         #no found fc link
        #         original_obj = orienteer.Object
        #         obj_for_getting_parents = original_obj
        #         link_to_obj_placement = fc.Placement() # zero placement because of origin obj is selected

        # works inside SetPlacement tools
        if is_fc_link(orienteer.Object) and orienteer.Object.Name.startswith('real_'):
            dynamic_link_of_robot_link = orienteer.Object
            if is_part(dynamic_link_of_robot_link.LinkedObject):
                original_obj_wrapper = dynamic_link_of_robot_link.LinkedObject
            if is_selection_object(orienteer):
                original_obj = get_selected_shape_object(orienteer)
            link_to_obj_placement = fc.Placement() # zero placement because of origin obj is selected
            
        # # parents_reversed = reversed(obj_for_getting_parents.Parents)
        # parents_reversed = reversed(obj_for_getting_parents.InListRecursive)
        # for p in parents_reversed:
        #     # p = p[0]
        #     if not dynamic_link_of_robot_link and is_fc_link(p) and p.Name.startswith('real_'):
        #         dynamic_link_of_robot_link = p
        #     elif not original_obj_wrapper and is_part(p):
        #         original_obj_wrapper = p

    except (AttributeError, IndexError, RuntimeError):
        pass

    if not original_obj_wrapper:
        message('Can not find object wrapper for adding LCS. Original object or link to original object must be wrapped by App::Part.', gui=True)
        raise RuntimeError()

    if not original_obj:
        message('Can not find original object for getting reference.', gui=True)
        raise RuntimeError()

    if not dynamic_link_of_robot_link:
        message('Can not find dynamic link of (real, visual, collision). Make robot structure first', gui=True)
        raise RuntimeError()

    sub_element_name = ''
    sub_element_type = ''
    sub_element = None
    try:
        sub_element_name = '.'.join(orienteer.SubElementNames[0].split('.')[-1:])
        sub_element_type = orienteer.SubObjects[0].ShapeType
        sub_element = orienteer.SubObjects[0]
    except (AttributeError, IndexError):
        pass

    body_lcs_wrapper = fc.ActiveDocument.addObject("PartDesign::Body", "Body")

    body_lcs_wrapper.Label = wb_constants.lcs_wrapper_prefix + orienteer.Object.Label + '(' + orienteer.Object.Name + ') ' + sub_element_name + ' '

    original_obj_wrapper.addObject(body_lcs_wrapper)

    lcs = fc.ActiveDocument.addObject( 'PartDesign::CoordinateSystem', 'LCS')
    body_lcs_wrapper.addObject(lcs)

    setattr(lcs, lcs_attachmentsupport_name(), [(original_obj, sub_element_name)])   # The X axis.

    if sub_element_type == 'Vertex':
        lcs.MapMode = 'Translate'
    elif sub_element_type == 'Edge' \
        and (
            sub_element.Curve.TypeId == 'Part::GeomCircle' \
            or sub_element.Curve.TypeId == 'Part::GeomBSplineCurve'
        ):

        lcs.MapMode = 'Concentric'
        if lcs_concentric_reversed:
            lcs.MapReversed = True
    else:
        lcs.MapMode = 'InertialCS'

    if deactivate_after_map_mode:
        # prevent automove back to InertialCS rotation
        lcs.MapMode = 'Deactivated'

    # remove placement of origin obj (mean in zero point of original obj)
    lcs.Placement = original_obj.Placement.inverse() * lcs.Placement
    # add placement of link of origin obj (mean in same place at face of link as at original obj)
    lcs.Placement = link_to_obj_placement * lcs.Placement
    # fix Z rotation to 0 in frame of origin
    lcs.Placement = rotate_placement(lcs.Placement, x = None, y = None, z = 0)

    # find placement of lcs at dynamic link of obj. lcs.Placement is placement at original object or it is link if present
    placement = dynamic_link_of_robot_link.Placement * lcs.Placement

    if delete_created_objects:
        fc.ActiveDocument.removeObject(body_lcs_wrapper.Name)
        fc.ActiveDocument.removeObject(lcs.Name)

    fc.activeDocument().recompute()

    return lcs, body_lcs_wrapper, placement


def rotate_placement(
        placement:fc.Placement,
        x:float | None = None, y:float | None = None, z:float | None = None,
        rotation_center: fc.Vector = fc.Vector(0,0,0),
) -> fc.Placement :
    ''' Rotate (incremental) placement in frame of origin or set any axis to zero.

        This func let you rotate object how be you rotate it as it was at origin.

        Params:
            placement - to rotate
            x,y,z - axis value as increment to rotate.
            If the value received is 0, the axis will be set to 0, otherwise the rotation will be incremented
    '''

    ## rotation
    if x is not None:
        rotAxis = fc.Vector(1,0,0)
        placement.rotate(rotation_center, rotAxis, x)
    if y is not None:
        rotAxis = fc.Vector(0,1,0)
        placement.rotate(rotation_center, rotAxis, y)
    if z is not None:
        rotAxis = fc.Vector(0,0,1)
        placement.rotate(rotation_center, rotAxis, z)

    ## setting axis angle to zero
    if x == 0 or y == 0 or z == 0:
        # go to default frame by inverse
        placement_inversed = placement.inverse()
        rotXYZ = placement_inversed.Rotation.toEulerAngles('XYZ')
        rotXYZ = list(rotXYZ)

        if x == 0:
            rotXYZ[0] = 0
        if y == 0:
            rotXYZ[1] = 0
        if z == 0:
            rotXYZ[2] = 0

        # set to zero position by axis in default frame
        placement_inversed.Rotation.setEulerAngles('XYZ', rotXYZ[0], rotXYZ[1], rotXYZ[2])
        # inverse back and get modificated rotation
        placement.Rotation = placement_inversed.inverse().Rotation

    return placement


def rotate_origin(x:float | None = None, y:float | None = None, z:float | None = None) -> bool :
    ''' Rotate joint origin by rotate_placement() func. '''

    doc = fc.activeDocument()
    selection_ok = False
    try:
        orienteer1, = validate_types(
            fcgui.Selection.getSelection(),
            ['Any'],
        )
        selection_ok = True
    except RuntimeError:
        pass

    if not selection_ok:
        message(
            'Select: joint or link or subelement of Real link or LCS of Real link.'
            , gui=True,
        )
        return
    
    orienteer1_sub_obj, *_ = fcgui.Selection.getSelectionEx("", 0)
    subNames = orienteer1_sub_obj.SubElementNames
    subNames = subNames[0] if len(subNames) else ''
    parsed_path = parse_freecad_path(subNames, orienteer1_sub_obj.Document)
    joint = None
    if is_lcs(parsed_path['object']):
        LCS = parsed_path['object']
        LCS.Placement = rotate_placement(LCS.Placement, x, y, z)
    else:
        if is_joint(orienteer1):
            joint = orienteer1
        elif is_link(orienteer1):
            link = orienteer1
        else:
            link = get_parent_link_of_obj(orienteer1_sub_obj.Object)

        if not joint:
            if link == None:
                message('Can not get parent robot link of selected object', gui=True)
                return

            # for subobjects (face, edge, vertex)
            if hasattr(orienteer1_sub_obj, 'Object') and not is_link(orienteer1):
                orienteer2_placement = get_placement_of_orienteer(
                    orienteer1_sub_obj,
                    lcs_concentric_reversed = True,
                    delete_created_objects = True,
                )
                orienteer2_to_link_diff = link.Placement.inverse() * orienteer2_placement
                link.MountedPlacement = rotate_placement(link.MountedPlacement, x, y, z, orienteer2_to_link_diff.Base)
            else:
                link.MountedPlacement = rotate_placement(link.MountedPlacement, x, y, z)
        else:
            joint.Origin = rotate_placement(joint.Origin, x, y, z)
        
    doc.recompute()
    return True

def get_xacro_wrapper_file_name(robot_name: str) -> str:
    """ Return xacro wrapper file name.

    Xacro wrapper file includes URDF description file and other xacro files."""

    return get_valid_filename(robot_name) + '_wrapper.urdf.xacro'


def get_sensors_file_name(robot_name: str) -> str:
    """ Return sensors xacro file name.

    Sensors xacro file contains gazebo sensors declarations"""

    return get_valid_filename(robot_name) + '_sensors.urdf.xacro'


def get_controllers_config_file_name(robot_name: str) -> str:
    """ Return controllers config file name (yaml config of ros2_control).
    """

    return get_valid_filename(robot_name) + '_controllers.yaml'


def git_change_submodule_branch(module_path: str, branch: str):
        message('Git change submodule branch.')
        p = subprocess.run(
            ["git submodule set-branch -b " + branch + ' ' + module_path],
            shell=True,
            capture_output=True,
            cwd=MOD_PATH,
            check=True,
        )
        print('process:', p)
        git_init_submodules(update_from_remote_branch = True, only_first_update = False)


def git_init_submodules(
        only_first_update: bool = True,
        update_from_remote_branch: bool = False,
        submodule_repo_path = ROS2_CONTROLLERS_PATH,
        pip_deps_install_submodule_paths: list = [ROBOT_DESCRIPTIONS_REPO_PATH],
        callback: function = None
):
    """
    Initializes and updates Git submodules.

    Args:
        only_first_update: Update submodules for the first time. Check first time by empty of submodule_repo_path dir.
        update_from_remote_branch: Whether to update submodules from the remote branch.
        submodule_repo_path: The path to the submodule repository.
        pip_deps_install_module_paths: Modules paths where should be made pip install when updates Git submodules.

    Description:
    This function checks if the submodule_repo_path directory is empty and initializes and updates Git submodules if necessary.
    Also does update when .gitmodules file changed.
    """

    def git_deinit_submodules():
        message('Deinit git submodules.')
        p = subprocess.run(
            ["git submodule deinit -f ."],
            shell=True,
            capture_output=True,
            cwd=MOD_PATH,
            check=True,
        )
        print('process:', p)


    def git_update_submodules(update_from_remote_branch_param: str = ''):
        message('Update git submodules.')
        p = subprocess.run(
            ["git submodule update --init " + update_from_remote_branch_param],
            shell=True,
            capture_output=True,
            cwd=MOD_PATH,
            check=True,
        )
        print('process:', p)

        for pip_deps_installmodule_path in pip_deps_install_submodule_paths:
            pip_install_dependencies_of_module(pip_deps_installmodule_path)


    def pip_install_dependencies_of_module(target_module_path: str):
        message('Pip install dependencies of module.')
        p = subprocess.run(
            ["pip install ."],
            shell=True,
            capture_output=True,
            cwd=target_module_path,
            check=True,
        )
        print('process:', p)


    progressBar = get_progress_bar(
        title = "Git submodules download and update...",
        min = 0,
        max = 100,
        show_percents = False,
    )
    progressBar.show()
    progressBar.setValue(0)
    gui_process_events()


    update_from_remote_branch_param = ''
    if update_from_remote_branch:
        update_from_remote_branch_param = '--remote'


    files_and_dirs = os.listdir(submodule_repo_path)
    # update if dir is empty or .gitmodules file was changed
    is_gitsubmodules_updated = os.path.expanduser('~/.is_robotcad_gitsubmodules_updated')
    if only_first_update:
        gitmodules_changed = is_gitmodules_changed()
        # update if empty dir
        if not len(files_and_dirs):
            git_update_submodules(update_from_remote_branch_param)
        # update when changed
        elif gitmodules_changed:
            git_deinit_submodules()
            git_update_submodules(update_from_remote_branch_param)
        # first update in clear container
        elif not os.path.exists(is_gitsubmodules_updated):
            with open(is_gitsubmodules_updated, 'w+') as f:
                git_update_submodules(update_from_remote_branch_param)
                f.write("git submodules updated")
                f.close()                
    else:
        git_update_submodules(update_from_remote_branch_param)

    progressBar.setValue(100)
    progressBar.close()
    gui_process_events()

    if callable(callback):
        callback()


def is_gitmodules_changed(workbench_path: Path = MOD_PATH) -> bool:
    """Check .gitmodules file for changes by backup file with md5 if .gitmodules"""
    gitmodules_md5_filepath = workbench_path / '.gitmodules_md5'
    gitmodules_changed = True
    gitmodules_md5_backup = False
    try:
        gitmodules_md5 = calc_md5(workbench_path / ".gitmodules")
        f = open(gitmodules_md5_filepath, "r")
        gitmodules_md5_backup = f.read()
        f.close()
    except (FileNotFoundError, IOError) as e:
        if isinstance(e, FileNotFoundError):
            pass
        else:
            print(f'Error reading file {gitmodules_md5_filepath}: {e}')

    if gitmodules_md5 != gitmodules_md5_backup:
        try:
            f = open(gitmodules_md5_filepath, "w")
            f.write(gitmodules_md5)
            f.close()
        except (FileNotFoundError, IOError) as e:
            print(f'Error writing file {gitmodules_md5_filepath}: {e}')
    else:
        gitmodules_changed = False

    return gitmodules_changed


def find_link_real_in_obj_parents(obj: fc.DocumentObject, link: CrossLink) -> fc.DocumentObject:
    """Find real object (Real of link) presents in parents of object.
    Usefull when need to know root parent of obj in Real of robot link
    if object`s parent is present in robot Link as Real"""
    parents_names = get_parents_names(obj)
    for parents_name in parents_names:
        for real in link.Real:
            if parents_name == real.Name:
                return real
    return None


def set_placement_fast(
        joint_origin: bool = True,
        link_mounted_placement: bool = True,
        parent_tree_to_child_branch: bool = False,
        child_branch_to_parent_tree: bool = False,
    ) -> bool |  tuple[DO, DO, DO]:
    doc = fc.activeDocument()
    selection_ok = False
    try:
        orienteer1, orienteer2 = validate_types(
            fcgui.Selection.getSelectionEx("", 0),
            ['Any', 'Any'],
            respect_count = True,
        )
        selection_ok = True
    except RuntimeError:
        pass

    if not selection_ok:
        message(
            'Select at Real objects of robot links (Parent, Child): \n'
            '1) face or edge or vertex or LCS of Parent,\n'
            '2) face or edge or vertex or LCS of Child.\n'
            '\n'
            'Robot links must be near to each other (parent, child) and have joint between.\n'
            '\n'
            'Check you dont select redundant objects in Project Tree.\n'  \
            , gui=True,
        )
        return False

    link1 = get_parent_link_of_obj(orienteer1)
    link2 = get_parent_link_of_obj(orienteer2)

    if link1 == None:
        message('Can not get parent robot link of first selected object', gui=True)
        return False

    if link2 == None:
        message('Can not get parent robot link of second selected object', gui=True)
        return False

    sel = fcgui.Selection.getSelectionEx("", 0)

    orienteer1_sub_obj = sel[0]
    orienteer2_sub_obj = sel[1]

    chain1 = get_chain(link1)
    chain2 = get_chain(link2)
    chain1_len = len(chain1)
    chain2_len = len(chain2)
    parent_link = None # same link as parent in both orienteers

    if chain1_len > chain2_len:
        parent_link = link2
        child_link = link1
        chain = chain1

        if is_lcs(orienteer2):
            parent_orienteer = orienteer2
        else:
            parent_orienteer = orienteer2_sub_obj

        if is_lcs(orienteer1):
            child_orienteer = orienteer1
        else:
            child_orienteer = orienteer1_sub_obj

    elif chain1_len < chain2_len:
        parent_link = link1
        child_link = link2
        chain = chain2
        parent_orienteer = orienteer1
        child_orienteer = orienteer2

        if is_lcs(orienteer1):
            parent_orienteer = orienteer1
        else:
            parent_orienteer = orienteer1_sub_obj

        if is_lcs(orienteer2):
            child_orienteer = orienteer2
        else:
            child_orienteer = orienteer2_sub_obj
    elif chain1_len == chain2_len == 1:
        message('Links must be connected by joints first.', gui=True)
        return False

    if not parent_link:
        message(
            'Tool works only for parent and child link orienteers.\n'
            'One orienteer must be in parent link and one in child link.', gui=True,
        )
        return False

    joint = chain[-2]
    if not is_joint(joint):
        message('Can not get joint between parent links of selected objects', gui=True)
        return False

    if parent_tree_to_child_branch or child_branch_to_parent_tree:
        if parent_tree_to_child_branch:
            doc.openTransaction(tr("Set placement - fast - parent tree to child branch"))
        else:
            doc.openTransaction(tr("Set placement - fast - child branch to parent tree"))

        robot = link1.Proxy.get_robot()
        root_link = robot.Proxy.get_root_link()
        root_link_child_joints = root_link.Proxy.get_ref_child_joints()
        child_link_child_joints = child_link.Proxy.get_ref_child_joints()
        # parent_joint_of_parent_link = parent_link.Proxy.get_ref_joint()
        parent_joint_of_child_link = child_link.Proxy.get_ref_joint()
        
        child_orienteer_placement_backup = get_placement_of_orienteer(child_orienteer, delete_created_objects=True)
        parent_orienteer_placement_backup = get_placement_of_orienteer(parent_orienteer, delete_created_objects=True)
        
        root_link_child_joints_backup = []
        for child_joint in root_link_child_joints:
            root_link_child_joints_backup.append({'Name': child_joint.Name, 'Origin': child_joint.Origin})

        parent_joint_of_child_link_origin_backup = parent_joint_of_child_link.Origin
        set_placement_by_orienteer(doc, parent_joint_of_child_link, 'Origin', parent_orienteer)
        move_placement(doc, child_link, 'MountedPlacement', child_orienteer, parent_orienteer)

        if child_link_child_joints:
            for child_joint in child_link_child_joints:
                move_placement(doc, child_joint, 'Origin', parent_joint_of_child_link.Origin, parent_joint_of_child_link_origin_backup)
                move_placement(doc, child_joint, 'Origin', child_orienteer_placement_backup, parent_orienteer_placement_backup)
        
        if parent_tree_to_child_branch:
            for child_joint in root_link_child_joints:
                move_placement(doc, child_joint, 'Origin', parent_orienteer_placement_backup, child_orienteer_placement_backup)

            move_placement(doc, root_link, 'MountedPlacement', parent_orienteer_placement_backup, child_orienteer_placement_backup)
        
        doc.recompute()
        doc.commitTransaction()
    else:
        doc.openTransaction(tr("Set placement - fast"))

        if joint_origin:
            set_placement_by_orienteer(doc, joint, 'Origin', parent_orienteer)
        if link_mounted_placement:
            move_placement(doc, child_link, 'MountedPlacement', child_orienteer, parent_orienteer)

        doc.commitTransaction()

    return joint, child_link, parent_link


def get_first_lcs_or_link(obj_name_list: list) -> DO | None:
    """Return first lcs from FreeCAD link or first link if lcs is not exist"""
    lcs = None
    link = None
    for obj_name in obj_name_list:
        obj = fc.ActiveDocument.getObject(obj_name)
        if is_fc_link(obj):
            link = obj
        elif is_lcs(obj):
            lcs = obj

    if lcs:
        return lcs
    else:
        return link
    

def get_first_link(obj_name_list: list) -> DO | None:
    """Return first FreeCAD link"""
    link = None
    for obj_name in obj_name_list:
        obj = fc.ActiveDocument.getObject(obj_name)
        if is_fc_link(obj):
            link = obj
            break

    return link


def get_last_link_to_assembly(obj_name_list: list) -> DO | None:
    """Return last FreeCAD assembly from Assembly WB"""
    assembly = None
    obj_name_list_reversed = list(reversed(obj_name_list))
    for obj_name in obj_name_list_reversed:
        obj = fc.ActiveDocument.getObject(obj_name)
        if is_link_to_assembly_from_assembly_wb(obj):
            assembly = obj
            break

    return assembly


def get_comulative_assemblies_placement(obj_name_list: list) -> DO | None:
    """Return comulative assemblies placement of FreeCAD assembly from Assembly WB included one in other.
    Parent assembly placement to child assembly placement and etc"""
    comulative_assemblies_placement = fc.Placement()
    for obj_name in obj_name_list:
        obj = fc.ActiveDocument.getObject(obj_name)
        if is_link_to_assembly_from_assembly_wb(obj):
            comulative_assemblies_placement = comulative_assemblies_placement * obj.Placement
        else:
            break

    return comulative_assemblies_placement
