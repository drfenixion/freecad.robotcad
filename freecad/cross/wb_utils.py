"""Utility function specific to this workbench."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, List, Optional, Protocol, Union
import os
import yaml
import xml.etree.ElementTree as ET
import re

import FreeCAD as fc
import FreeCADGui as fcgui

from . import wb_constants
from . import wb_globals
from .freecad_utils import get_param
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
from .freecadgui_utils import get_placement
from .ros.utils import get_ros_workspace_from_file
from .ros.utils import without_ros_workspace
from .utils import attr_equals
from .utils import values_from_string
from .utils import get_valid_filename
from .exceptions import NoPartWrapperOfObject
from .freecad_utils import validate_types

# Stubs and typing hints.
from .joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
from .workcell import Workcell as CrossWorkcell  # A Cross::Workcell, i.e. a DocumentObject with Proxy "Workcell". # noqa: E501
from .xacro_object import XacroObject as CrossXacroObject  # A Cross::XacroObject, i.e. a DocumentObject with Proxy "XacroObject". # noqa: E501
from .controller import Controller as CrossController  # A Cross::Controller, i.e. a DocumentObject with Proxy "Controller". # noqa: E501
DO = fc.DocumentObject
CrossBasicElement = Union[CrossJoint, CrossLink]
CrossObject = Union[CrossJoint, CrossLink, CrossRobot, CrossXacroObject, CrossWorkcell, CrossController]
DOList = List[DO]

MOD_PATH = Path(fc.getUserAppDataDir()) / 'Mod/freecad.robotcad'
RESOURCES_PATH = MOD_PATH / 'resources'
UI_PATH = RESOURCES_PATH / 'ui'
ICON_PATH = RESOURCES_PATH / 'icons'
ROS2_CONTROLLERS_PATH = MOD_PATH / 'modules' / 'ros2_controllers'


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


def is_workcell_selected() -> bool:
    """Return True if the first selected object is a Cross::Workcell."""
    return is_selected_from_lambda(is_workcell)


def is_planning_scene_selected() -> bool:
    """Return True if the first selected object is a Cross::PlanningScene."""
    return is_selected_from_lambda(is_planning_scene)


def get_links(objs: DOList) -> list[CrossLink]:
    """Return only the objects that are Cross::Link instances."""
    return [o for o in objs if is_link(o)]


def get_joints(objs: DOList) -> list[CrossJoint]:
    """Return only the objects that are Cross::Joint instances."""
    return [o for o in objs if is_joint(o)]


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
        check_kinematics = True
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


def get_rel_and_abs_path(path: str) -> tuple[str, Path]:
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
    - keys: dictionnary of replacement string in templates.
            - package_name (compulsary): name of the ROS package and its containing
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
    return fc.Placement(fc.Vector(x, y, z) * ros_to_freecad_factor,
                        fc.Rotation(qw, qx, qy, qz))


def get_urdf_path(robot: CrossRobot, output_path: str) -> Path:
    """Return URDF file path`.
    """
        
    file_base = get_valid_filename(ros_name(robot))
    urdf_file = f'{file_base}.urdf'
    urdf_path = output_path / f'urdf/{urdf_file}'
    urdf_path = Path(urdf_path)

    return urdf_path


def set_placement_by_orienteer(doc: DO, link_or_joint: DO, origin_or_mounted_placement_name: str, orienteer1: DO):
    """Set element (joint) placement (Origin) by orienteer.

    Set placement with orienteer placement value
    """

    # set_placement_by_orienteer() does not work for MountedPlacement of link
    # because link conjuction place in many cases not at origin (zero coordinates) of link
    # Instead of this use move_placement()

    placement1 = get_placement_of_orienteer(orienteer1, lcs_concentric_reversed = True)

    # prepare data
    origin_or_mounted_placement_name__old = getattr(link_or_joint, origin_or_mounted_placement_name)
    setattr(link_or_joint, origin_or_mounted_placement_name, fc.Placement(fc.Vector(0,0,0), fc.Rotation(0,0,0), fc.Vector(0,0,0)))  # set zero Origin
    doc.recompute() # trigger compute element placement based on zero Origin
    element_basic_placement = getattr(link_or_joint, 'Placement')
    setattr(link_or_joint, origin_or_mounted_placement_name, origin_or_mounted_placement_name__old)
    doc.recompute()

    ## prepare data
    placement1_diff = element_basic_placement.inverse() * placement1

    # Set Origin or Mounted placement
    setattr(link_or_joint, origin_or_mounted_placement_name, placement1_diff)


def move_placement(doc: DO, link_or_joint: DO, origin_or_mounted_placement_name: str, 
                   orienteer1: DO, orienteer2: DO, delete_created_objects:bool = True):
    """Move element (joint or link) placement (Origin or Mounted placement).

    Move first orienteer to placement of second and second orienteer
    and element to positions relative their bind system (element and orienteers) before.
    This move does not change spatial relation between each other.
    """

    placement1 = get_placement_of_orienteer(orienteer1, delete_created_objects)
    placement2 = get_placement_of_orienteer(orienteer2, delete_created_objects, lcs_concentric_reversed = True)
    
    # prepare data
    origin_or_mounted_placement_name__old = getattr(link_or_joint, origin_or_mounted_placement_name)
    # set zero Origin
    setattr(link_or_joint, origin_or_mounted_placement_name, fc.Placement(fc.Vector(0,0,0), fc.Rotation(0,0,0), fc.Vector(0,0,0)))
    doc.recompute() # trigger compute element placement based on zero Origin
    element_basic_placement = getattr(link_or_joint, 'Placement')
    setattr(link_or_joint, origin_or_mounted_placement_name, origin_or_mounted_placement_name__old)
    doc.recompute()

    ## prepare data
    element_local_placement = getattr(link_or_joint, origin_or_mounted_placement_name)
    origin_placement1_diff = (element_basic_placement * element_local_placement).inverse() * placement1
    origin_placement2_diff = (element_basic_placement * element_local_placement).inverse() * placement2

    # do Origin move
    # first orienteer come to second orienteer place and Origin respectively moved
    # in local frame every tool click will result Origin move because both orienteers moved and received new position
    new_local_placement = element_local_placement * origin_placement2_diff * origin_placement1_diff.inverse()
    setattr(link_or_joint, origin_or_mounted_placement_name, new_local_placement)



def get_placement_of_orienteer(orienteer, delete_created_objects:bool = True, lcs_concentric_reversed:bool = False) \
    -> fc.Placement :
    '''Return placement of orienteer. 
    If orienteer is not certain types it will make LCS with InertialCS map mode and use it'''

    # TODO process Vertex
    if is_lcs(orienteer) :
        placement = get_placement(orienteer)
    elif is_link(orienteer) or is_joint(orienteer):
        placement = orienteer.Placement
    else:
        lcs, body_lcs_wrapper, placement = make_lcs_at_link_body(orienteer, delete_created_objects, lcs_concentric_reversed)

    return placement


def make_lcs_at_link_body(orienteer, delete_created_objects:bool = True, lcs_concentric_reversed:bool = False) \
    -> list[fc.DO, fc.DO, fc.Placement] :
    '''Make LCS at face of body of robot link. 
    orienteer body must be wrapper by part and be Real element of robot link'''
    
    link = None
    # trying get link from subelement
    try:
        orienteer_parents_reversed = reversed(orienteer.Object.Parents)
        for parent in orienteer_parents_reversed:
            parent = parent[0]
            if is_fc_link(parent):
                link = parent
    except (AttributeError, IndexError, RuntimeError):
        pass
        
    # orienteer not a subelement (face, edge, etc)
    if not link:
        link = orienteer

    obj = link.getLinkedObject(True)
    obj_placement = obj.Placement
    link_to_obj_placement = link.Placement

    if not is_part(obj):
        message('Can not get Part-wrapper of object. Real object of robot link must have Part as wrapper of body.', gui=True)
        raise NoPartWrapperOfObject()
    
    sub_element_name = ''
    sub_element_type = ''
    sub_element = None
    try:
        sub_element_name = orienteer.SubElementNames[0]
        sub_element_type = orienteer.SubObjects[0].ShapeType
        sub_element = orienteer.SubObjects[0]
    except (AttributeError, IndexError):
        pass

    body_lcs_wrapper = fc.ActiveDocument.addObject("PartDesign::Body", "Body")

    body_lcs_wrapper.Label = "LCS wrapper " + orienteer.Object.Label + '(' + orienteer.Object.Name + ') ' + sub_element_name + ' '

    obj.addObject(body_lcs_wrapper)

    lcs = fc.ActiveDocument.addObject( 'PartDesign::CoordinateSystem', 'LCS' )
    body_lcs_wrapper.addObject(lcs)

    setattr(lcs, lcs_attachmentsupport_name(), [(orienteer.Object, sub_element_name)])   # The X axis.

    if sub_element_type == 'Vertex':
        lcs.MapMode = 'Translate'
    elif sub_element_type == 'Edge' \
        and (sub_element.Curve.TypeId == 'Part::GeomCircle' \
            or sub_element.Curve.TypeId == 'Part::GeomBSplineCurve'):
        
        lcs.MapMode = 'Concentric'
        if lcs_concentric_reversed:
            lcs.MapReversed = True
    else:
        lcs.MapMode = 'InertialCS'
    
    # prevent automove back to InertialCS rotation
    lcs.MapMode = 'Deactivated'

    # fix Z rotation to 0 in frame of origin
    lcs.Placement = rotate_placement(lcs.Placement, x = None, y = None, z = 0)

    # find placement of lcs at link of obj. lcs.Placement is placement at support obj
    placement = link_to_obj_placement * lcs.Placement

    if delete_created_objects:
        fc.ActiveDocument.removeObject(body_lcs_wrapper.Name)
        fc.ActiveDocument.removeObject(lcs.Name)

    fc.activeDocument().recompute()

    return lcs, body_lcs_wrapper, placement


def rotate_placement(placement:fc.Placement, x:float | None = None, y:float | None = None, z:float | None = None) -> fc.Placement :
    ''' Rotate (incremental) placement in frame of origin or set any axis to zero. 

        This func let you rotate object how be you rotate it as it was at origin.

        Params:
            placement - to rotate
            x,y,z - axis value as increment to rotate. 
            If the value received is 0, the axis will be set to 0, otherwise the rotation will be incremented
    '''

    ## rotation
    if x is not None:
        placement.rotate(fc.Vector(0,0,0), fc.Vector(1,0,0), x)
    if y is not None:
        placement.rotate(fc.Vector(0,0,0), fc.Vector(0,1,0), y)
    if z is not None:
        placement.rotate(fc.Vector(0,0,0), fc.Vector(0,0,1), z)

    ## setting axis angle to zero
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
            ['Any'])
        selection_ok = True
    except RuntimeError:
        pass

    if not selection_ok:
        message('Select: subobject of robot link or link.'
                , gui=True)
        return

    joint = None
    if is_joint(orienteer1):
        joint = orienteer1
    if is_link(orienteer1):
        link = orienteer1
    else:
        link = get_parent_link_of_obj(orienteer1)

    # get parent joint from link
    if not joint:
        if link == None:
            message('Can not get parent robot link of selected object', gui=True)
            return      
        
        chain = get_chain(link)

        if len(chain) < 2:
            message('Link must be in chain (joint to link).', gui=True)
            return                


        joint = chain[-2]
        if not is_joint(joint):
            message('Can not get parent joint of link', gui=True)
            return
    
    joint.Origin = rotate_placement(joint.Origin, x, y, z)

    doc.recompute()


def get_controllers_data(ROS2_CONTROLLERS_PATH: Path) -> dict :
    ''' Get controllers data. '''

    def collect_controllers_parameters(controllers: dict) -> dict :
        ''' Adding to controllers their collected parameters. '''

        for controller_name in controllers:
            controller = controllers[controller_name]

            for path in list(controller['parameters_path'].values()):
                with open(path) as stream:
                    if 'parameters' in controllers[controller_name]:
                        controllers[controller_name]['parameters'].update(yaml.safe_load(stream))
                    else:
                        controllers[controller_name]['parameters'] = yaml.safe_load(stream)
        
        return controllers
    

    def collect_controllers_plugin_data(controllers: dict) -> dict :
        ''' Adding to controllers their plugin data. '''

        for dir_name in controllers:
            controller = controllers[dir_name]

            for path in list(controller['plugin_path'].values()):
                root = ET.parse(path).getroot()

                for type_tag in root.findall('class'):
                    full_class_name = type_tag.get('name')
                    class_name = full_class_name.split('/')[-1]
                    camelCaseToList = re.findall(r'[A-Z](?:[a-z]+|[A-Z]*(?=[A-Z]|$))', class_name)
                    controller_name = '_'.join(camelCaseToList).lower()
                    plugin = {full_class_name: 
                                {
                                  'name': controller_name,
                                  'description': type_tag.find('description').text.strip()
                                }
                              }

                    if 'plugins_data' in controllers[dir_name]:
                        controllers[dir_name]['plugins_data'].update(plugin)
                    else:
                        controllers[dir_name]['plugins_data'] = plugin
        
        return controllers

    controllers = get_controllers_root_dirs(ROS2_CONTROLLERS_PATH)
    controllers['controllers_dirs'] = collect_controllers_config_files(controllers['controllers_dirs'])
    controllers['broadcasters_dirs'] = collect_controllers_config_files(controllers['broadcasters_dirs'])
    controllers['controllers_dirs'] = collect_controllers_parameters(controllers['controllers_dirs'])
    controllers['broadcasters_dirs'] = collect_controllers_parameters(controllers['broadcasters_dirs'])
    controllers['controllers_dirs'] = collect_controllers_plugin_data(controllers['controllers_dirs'])
    controllers['broadcasters_dirs'] = collect_controllers_plugin_data(controllers['broadcasters_dirs'])

    controllers['controllers'] = separate_controllers_from_dirs(controllers['controllers_dirs'])
    controllers['broadcasters'] = separate_controllers_from_dirs(controllers['broadcasters_dirs'])

    return controllers

def separate_controllers_from_dirs(controllers_dirs: dict) -> dict :
    ''' Separate controllers from controllers directories dictionaries. 
    
    Directory can contains several controllers.
    This make controllers as first level properties of dict.
    '''

    def add_fc_types_based_on_params_types(params: dict, param_name_prefix: list = []) -> dict:
        ''' Return params with FreeCAD types in addition to ros2_controllers parameters types
        '''

        # some types required to be replaced to more suited because can be more convenient to use
        replacement = wb_constants.ROS2_CONTROLLERS_PARAMS_TYPES_REPLACEMENTS
        for param_name, param in params.items():
            try:
                # param['type'] - KeyError trigger to recursion because type present only in leaf element
                params[param_name]['type_fc'] = wb_constants.ROS2_CONTROLLERS_PARAMS_TO_FRECAD_PROP_MAP[param['type']]
                params[param_name]['type_fc_origin'] = param['type_fc']
                if 'default_value' in param:
                    params[param_name]['default_value_origin'] = param['default_value']
                
                # full_param_name means param_name + all parents prefixes
                full_param_name = '__'.join(param_name_prefix + [param_name]) 
                if full_param_name in replacement:
                    params[param_name]['type_fc'] = replacement[full_param_name]['replace']
                    # some types of replaced params must have other default value
                    if 'default_value_replace' in replacement[full_param_name]:
                        params[param_name]['default_value'] = replacement[full_param_name]['default_value_replace']
            except KeyError:
                param_name_prefix.append(param_name)
                params[param_name] = add_fc_types_based_on_params_types(param, param_name_prefix)
                param_name_prefix.pop()

        return params


    def exlude_params(params: dict, exluded_params: list) -> dict:
        ''' Return params without exluded params
        '''

        params_without_exluded = {}
        for param_name, param in params.items():
            if param_name not in exluded_params:
                params_without_exluded[param_name] = param

        return params_without_exluded
    

    def add_validation_rules_str_to_params(params: dict) -> dict:
        ''' Add validation rules string to every param with validation. 
        
        It convert rules to string and to every param where present validation.
        '''

        for param_name, param in params.items():
            validation_str = ''
            if 'validation' in param:
                for validation_rule, validation_value in param['validation'].items():

                    rule_desc = 'Validation rule: '
                    try:
                        rule_desc += wb_constants.ROS2_CONTROLLERS_VALIDATION_RULES_DESCRIPTIONS[validation_rule.rstrip("<>")]
                    except KeyError:
                        rule_desc += validation_rule.rstrip("<>")

                    validation_value_str = ''
                    if validation_value:
                        validation_value_str += '. Validation value: '
                        
                        try:
                            validation_value_str += ', '.join(str(el) for el in validation_value)
                        except:
                            validation_value_str += str(validation_value)

                    validation_str += rule_desc + validation_value_str + '\n'

                params[param_name]['validation_str'] = validation_str

        return params
    

    special_cases = {
        'joint_group_velocity_controller': {
            'params_based_on_controller_dir': 'forward_command_controller',
            'params_based_on_controller_name': 'forward_command_controller',
            'excluded_params': ['interface_name'],
        },
        'joint_group_effort_controller': {
            'params_based_on_controller_dir': 'forward_command_controller',
            'params_based_on_controller_name': 'forward_command_controller',
            'excluded_params': ['interface_name'],
        },
        'joint_group_position_controller': {
            'params_based_on_controller_dir': 'forward_command_controller',
            'params_based_on_controller_name': 'forward_command_controller',
            'excluded_params': ['interface_name'],
        },        
    }

    controllers = {}
    for controller_dir_name, controller_dir in controllers_dirs.items():
        if 'plugins_data' in controller_dir:
            for plugin_class_name, plugin_data in controller_dir['plugins_data'].items():

                plugin_data_name = plugin_data['name']

                # prepare controller parameters
                parameters = {}
                try:
                    parameters = controller_dir['parameters'][plugin_data_name]
                except KeyError:
                    # special cases for controllers that does not have their own params and uses params from other controllers
                    params_based_on_controller_dir = special_cases[plugin_data_name]['params_based_on_controller_dir']
                    params_based_on_controller_name = special_cases[plugin_data_name]['params_based_on_controller_name']
                    params_to_exlude = special_cases[plugin_data_name]['excluded_params']
                    parameters = exlude_params(
                        controllers_dirs[params_based_on_controller_dir]['parameters'][params_based_on_controller_name],
                        params_to_exlude
                        )
                if plugin_data_name in ['ackermann_steering_controller', 'bicycle_steering_controller', 'tricycle_steering_controller']:
                    # must add steering_controllers_library params to steering controllers
                    parameters.update(controllers_dirs['steering_controllers_library']['parameters']['steering_controllers_library'])
                
                parameters = add_fc_types_based_on_params_types(parameters)

                parameters = add_validation_rules_str_to_params(parameters)


                # assembly controller dict
                controllers[plugin_data_name] = {
                    'name': plugin_data_name,
                    'description': plugin_data['description'],
                    'controller_path': controller_dir['dir'],
                    'plugin_path': controller_dir['plugin_path'],
                    'parameters_path': controller_dir['parameters_path'],
                    'parameters': parameters,
                    'controller_dir_name': controller_dir_name,
                    'controller_plugin_class_name': plugin_class_name,
                }

    return controllers


def get_controllers_root_dirs(ROS2_CONTROLLERS_PATH: Path) -> dict :
    ''' Get controllers root dirs. '''
    controllers_and_broadcusters = {}

    controller_key_words = ['controller']
    controller_key_words_blacklist = ['test', 'ros2_controllers', 'rqt']
    controllers = get_files_or_dirs_by_filter(ROS2_CONTROLLERS_PATH, 
                                              files_or_dirs = 'dirs', 
                                              key_words = controller_key_words, 
                                              key_words_blacklist = controller_key_words_blacklist, 
                                              attr_name_for_found_result = 'dir')

    controller_key_words = ['broadcaster']
    broadcasters = get_files_or_dirs_by_filter(ROS2_CONTROLLERS_PATH, 
                                               files_or_dirs = 'dirs', 
                                               key_words = controller_key_words, 
                                               key_words_blacklist = controller_key_words_blacklist, 
                                               attr_name_for_found_result = 'dir')

    controllers_and_broadcusters['controllers_dirs'] = controllers
    controllers_and_broadcusters['broadcasters_dirs'] = broadcasters

    return controllers_and_broadcusters


def collect_controllers_config_files(controllers: dict) -> dict :
    ''' Get controllers plugin xml file and parameters yaml file. '''

    for controller_folder_name, controller_value in controllers.items():

        plugin = get_files_or_dirs_by_filter(
            controller_value['dir'], 
            key_words = ['.xml'], 
            key_words_blacklist = ['package.xml'])
        controllers[controller_folder_name].update({'plugin_path': plugin})

        parameters = get_files_or_dirs_by_filter(
            controller_value['dir'] / 'src', 
            key_words = ['.yaml'])
        controllers[controller_folder_name].update({'parameters_path': parameters})

    return controllers


def get_files_or_dirs_by_filter(dir: Path, 
                                files_or_dirs:str = 'files',
                                key_words:list = [],
                                key_words_blacklist:list = [],
                                attr_name_for_found_result:str | None = None) -> dict :
    ''' Get files or dirs in path by included keywords in file name. '''

    files_or_dirs_result = {}
    for root, dirs, files in os.walk(dir):
        for name in eval(files_or_dirs):
            # check key words
            for key_word in key_words:
                if key_word in name:
                    
                    # check blacklist
                    name_in_blacklist = False
                    for key_word_bl in key_words_blacklist:
                        if key_word_bl in name:
                            name_in_blacklist = True
                            break
                    
                    # forming result
                    if not name_in_blacklist:  
                        if name not in files_or_dirs_result:
                            if not attr_name_for_found_result:
                                files_or_dirs_result[name] = {Path(root) / name}
                            else:
                                files_or_dirs_result[name] = {}

                        if not attr_name_for_found_result:
                            files_or_dirs_result.update({name: Path(root) / name})  
                        else:
                            files_or_dirs_result.update({name: {attr_name_for_found_result: Path(root) / name}})
        
        break

    return files_or_dirs_result
