"""Proxy for Cross::Robot FreeCAD objects

A robot is a combination of Cross::Link and Cross::Joint objects that can be
exported as URDF file.

"""

from __future__ import annotations

from math import radians
from typing import ForwardRef, List, Optional, Union, cast
import xml.etree.ElementTree as et
from copy import deepcopy

import FreeCAD as fc

from PySide.QtWidgets import QFileDialog  # FreeCAD's PySide
from PySide.QtWidgets import QMenu # FreeCAD's PySide

from .freecad_utils import ProxyBase, is_assembly_from_assembly_wb, is_grounded_join_from_assembly_wb, is_join_from_assembly_wb, is_lcs, is_link_to_assembly_from_assembly_wb, is_part
from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import get_properties_of_category
from .freecad_utils import get_valid_property_name
from .freecad_utils import is_origin
from .freecad_utils import label_or
from .freecad_utils import quantity_as
from .freecad_utils import warn
from .freecad_utils import is_link as is_fc_link
from .gui_utils import tr
from .ros.utils import split_package_path
from .ui.file_overwrite_confirmation_dialog import FileOverwriteConfirmationDialog
from .urdf_utils import xml_comment_element
from .utils import get_valid_filename, replace_substring_in_keys
from .utils import grouper
from .utils import save_xml
from .utils import save_yaml
from .utils import save_file
from .utils import warn_unsupported
from .wb_utils import ICON_PATH, get_chain
from .wb_utils import export_templates
from .wb_utils import get_attached_collision_objects
from .wb_utils import get_chains
from .wb_utils import get_joints
from .wb_utils import get_links
from .wb_utils import get_controllers
from .wb_utils import get_broadcasters
from .wb_utils import get_rel_and_abs_path
from .wb_utils import get_valid_urdf_name
from .wb_utils import is_attached_collision_object
from .wb_utils import is_joint
from .wb_utils import is_link
from .wb_utils import is_robot
from .wb_utils import is_controller
from .wb_utils import is_broadcaster
from .wb_utils import is_controllers_template_for_param_mapping
from .wb_utils import remove_ros_workspace
from .wb_utils import ros_name
from .wb_utils import _has_meshes_directory
from .wb_utils import get_urdf_path
from .wb_utils import git_init_submodules
from xml.dom.minidom import parseString
from pathlib import Path
from .joint_proxy import make_robot_joint_filled, make_robot_joints_filled
from .link_proxy import make_robot_link_filled, make_robot_links_filled
from . import wb_constants
from .wb_utils import get_xacro_wrapper_file_name
from .wb_utils import get_sensors_file_name
from .wb_utils import get_controllers_config_file_name
from .sdf.sdf_parser.sdf_tree import sdf_dict_to_xml

# Stubs and type hints.
from .attached_collision_object import AttachedCollisionObject as CrossAttachedCollisionObject  # A Cross::AttachedCollisionObject, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
from .joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
from .controller import Controller as CrossController  # A Cross::Controller, i.e. a DocumentObject with Proxy "Controller". # noqa: E501
BasicElement = Union[CrossJoint, CrossLink]
DO = fc.DocumentObject
DOList = List[DO]
VPDO = ForwardRef('FreeCADGui.ViewProviderDocumentObject')  # Don't want to import FreeCADGui here. # noqa: E501
AppLink = DO  # TypeId == 'App::Link'.


def _add_joint_variable(
        robot: CrossRobot,
        joint: CrossJoint,
        category: str,
) -> str:
    """Add a property for the actuator value to `robot` and return its name.

    Add a property starting with "joint.Label" to represent the actuation
    value of a joint. Supported types are 'prismatic', 'revolute', and
    'continuous'. There is no actuation value for mimicking joints.

    """
    if not is_joint(joint):
        warn(
            f'Wrong object type. {joint.Name} ({joint.Label})'
            ' is not a Cross::Joint',
        )
        return ''
    if joint.Mimic:
        # No actuator for mimic joints.
        return ''
    if joint.Type == 'prismatic':
        unit = 'mm'
    elif joint.Type in ['revolute', 'continuous']:
        unit = 'deg'
    else:
        # Non-simple joints not supported yet.
        return ''
    rname = ros_name(joint)
    # e.g. name_candidate = "q0_deg" or "q0".
    name_candidate = f'{rname}{f"_{unit}" if unit else ""}'
    var_name = get_valid_property_name(name_candidate)
    # e.g. help_txt = "q0 in deg" or "q0".
    label = joint.Label
    id_ = rname if rname == label else f'{rname} ({label})'
    help_txt = f'{id_}{f" in {unit}" if unit else ""}'
    if joint.Type in ['prismatic', 'revolute']:
        prop_type = 'App::PropertyFloatConstraint'
    else:
        prop_type = 'App::PropertyFloat'

    # recreate prop in case of type was changed
    prop = getattr(robot, var_name, False)
    if prop is not False:
        robot.removeProperty(var_name)

    _, used_var_name = add_property(
        robot,
        prop_type,
        var_name,
        category,
        help_txt,
    )

    try:
        # try to set old value in case of type was changed
        setattr(robot, used_var_name, prop)
    except:
        pass
    
    if joint.Type in ['prismatic', 'revolute']:
        # Set the default value to the current value to set min/max.
        value = robot.getPropertyByName(used_var_name)
        if (joint.LowerLimit == 0.0) and (joint.UpperLimit == 0.0):
            # Properties are not set.
            min_, max_ = -1e999, 1e999
        else:
            min_, max_ = joint.LowerLimit, joint.UpperLimit
            if value > max_:
                warn('Joint (' + var_name + ') value can not be more Upper limit. Value set as Upper limit.')
                value = max_
            if value < min_:
                warn('Joint (' + var_name + ') value can not be less Lower limit. Value set as Lower limit.')
                value = min_
            if max_ < min_:
                error('Joint (' + var_name + ') Upper limit can not be less LowerLimit limit')
        # Deactivate callback on change.
        robot.setPropertyStatus(used_var_name, ['NoRecompute'])
        setattr(robot, used_var_name, (value, min_, max_, 1.0))
        robot.setPropertyStatus(used_var_name, [])
    value: Optional[float] = None
    if joint.Type == 'prismatic':
        value = robot.getPropertyByName(used_var_name) * 0.001
    elif joint.Type in ['revolute', 'continuous']:
        value = radians(robot.getPropertyByName(used_var_name))
    if ((value is not None)
            and (joint.Position != value)
            and (not joint.Mimic)):
        # Avoid recursive recompute.
        joint.Position = value
    return used_var_name


def _dispatch_to_joint_view_objects(
        robot: CrossRobot,
        prop: str,
        joint_prop: str,
) -> None:
    """Dispatch a property to the view objects of the joints of `robot`."""
    if not is_robot(robot):
        return
    if (not hasattr(robot, 'Proxy')) or (not robot.Proxy.is_execute_ready()):
        return
    prop_value = getattr(robot.ViewObject, prop)
    for joint in get_joints(robot.Group):
        if hasattr(joint.ViewObject, 'Proxy') and (joint.ViewObject.Proxy is not None):
            joint.ViewObject.Proxy.update_prop(joint_prop, prop_value)


def _dispatch_to_link_view_objects(
        robot: CrossRobot,
        prop: str,
        link_prop: str,
) -> None:
    if not is_robot(robot):
        return
    if (not hasattr(robot, 'Proxy')) or (not robot.Proxy.is_execute_ready()):
        return
    prop_value = getattr(robot.ViewObject, prop)
    for link in robot.Proxy.get_links():
        if hasattr(link.ViewObject, 'Proxy') and (link.ViewObject.Proxy is not None):
            link.ViewObject.Proxy.update_prop(link_prop, prop_value)


class RobotProxy(ProxyBase):
    """The proxy for CROSS::Robot objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::Robot'

    # Name of the category (or group) for the joint values, which are saved as
    # properties of `self.robot`.
    _category_of_joint_values = 'JointValues'

    def __init__(self, obj: CrossRobot):
        # Implementation note: 'Group' is not required because
        # DocumentObjectGroupPython.
        super().__init__(
            'robot', [
                'CreatedObjects',
                'OutputPath',
                'MaterialCardName',
                'MaterialCardPath',
                'MaterialDensity',
                'RobotType',
                'GenerateCodeForRosVersion',
                '_Type',
                'Mass',
            ],
        )

        if obj.Proxy is not self:
            obj.Proxy = self
        self.robot = obj

        # List of objects created for the robot.
        # Used for example by `robot_from_urdf` to keep track of imported
        # meshes.
        # This class doesn't add any object to this list itself.
        # TODO: remove `_created_objects` and use `robot.CreatedObjects` instead.
        self._created_objects: DOList = []

        # Map of CROSS joints to joint variable names.
        # Only for actuated non-mimicking joints.
        self._joint_variables: dict[CrossJoint, str] = {}

        # Map of ROS names to joint variable names.
        # Used to restore the joint variables from the dumped state because
        # DocumentObject instances cannot be saved.
        # Defined in onDocumentRestored().
        # TODO: Maybe save as two lists (App::PropertyLinkListGlobal and App::PropertyStringList).
        self._joint_variables_ros_map: dict[str, str]

        # Save the children to speed-up get_links() and get_joints().
        self._attached_collision_objects: Optional[list[CrossAttachedCollisionObject]] = None
        self._links: Optional[list[CrossLink]] = None
        self._joints: Optional[list[CrossJoint]] = None
        self._joints_old: Optional[list[CrossJoint]] = []

        self._controllers: Optional[list[CrossController]] = None
        self._broadcasters: Optional[list[CrossController]] = None

        self._init_properties(obj)

    @property
    def created_objects(self) -> DOList:
        """List of objects created for the robot."""
        return self._created_objects

    @property
    def joint_variables(self) -> dict[CrossJoint, str]:
        """Map of CROSS joints to joint variable names."""
        return self._joint_variables

    def _init_properties(self, obj: CrossRobot):
        add_property(
            obj, 'App::PropertyString', '_Type', 'Internal',
            'The type of object',
        )
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        # store created objects to PropertyLinkListGlobal leads to error - 'Object can only be in a single Group'
        # looks like PropertyLinkListGlobal perceived as the group and brokes other object`s group
        # used PropertyLinkListHidden instead
        add_property(
            obj, 'App::PropertyLinkListHidden', 'CreatedObjects', 'Internal',
            'Objects created for the robot',
        )

        add_property(
            obj, 'App::PropertyQuantity', 'Mass', 'Inertial Parameters',
            'Mass of the link',
        )
        obj.Mass = fc.Units.Mass

        add_property(
            obj, 'App::PropertyPlacement', 'CenterOfMassGlobalCoords', 'Inertial Parameters',
            'Center of mass of the link, with orientation determining the principal axes of inertia \
                        in global coordinates.',
        )

        # Managed in self.reset_group().
        obj.setPropertyStatus('Group', 'ReadOnly')

        add_property(
            obj, 'App::PropertyPath', 'OutputPath', 'Export',
            'The path to the ROS package to export files to,'
            ' relative to $ROS_WORKSPACE/src',
        )

        add_property(
                obj,
                'App::PropertyString',
                'MaterialCardName',
                'Material',
                (
                    'Default material of robot. Used to calculate mass and inertia'
                    ' if link has not its own material. Use "Set material" tool to'
                    ' change'
                ),
        )
        obj.setPropertyStatus('MaterialCardName', ['ReadOnly'])
        add_property(
            obj, 'App::PropertyPath', 'MaterialCardPath', 'Material',
            'Default material of robot. Used to calculate mass and inertia',
        )
        obj.setPropertyStatus('MaterialCardPath', ['Hidden', 'ReadOnly'])
        add_property(
                obj,
                'App::PropertyString',
                'MaterialDensity',
                'Material',
                (
                    'Density of material. Used to calculate mass. May be outdated'
                    ' if you updated the material density outside RobotCAD workbench.'
                    ' Actual density will taken from material (material editor) at'
                    ' mass calculation moment.'
                ),
        )
        obj.setPropertyStatus('MaterialDensity', ['ReadOnly'])

        add_property(
            obj, 'App::PropertyEnumeration', 'RobotType', 'Robot',
            'Used by extended code generator. For multicopter use ROS2 Iron code generation because PX4 does not released for Ubuntu 24.04 yet',
        )
        obj.RobotType=["nonspecific","multicopter"]

        add_property(
            obj, 'App::PropertyEnumeration', 'GenerateCodeForRosVersion', 'Robot',
            'Generate code for choosed ROS version.',
        )
        obj.GenerateCodeForRosVersion=["jazzy", "iron"]


        # The `Placement` is not used directly by the robot but it is used to
        # transform the pose of its links.
        add_property(
            obj, 'App::PropertyPlacement', 'Placement',
            'Base', 'Placement',
        )

    def execute(self, obj: CrossRobot) -> None:
        self._cleanup_group()
        self.set_joint_enum()
        self.add_joint_variables()
        self.compute_poses()
        # self.reset_group()

    def onChanged(self, obj: CrossRobot, prop: str) -> None:
        # print(f'{obj.Name}.onChanged({prop})') # DEBUG
        if not self.is_execute_ready():
            return
        if prop in ['Group']:
            # Reset _links and _joints to provoke a recompute.
            self._links = None
            self._joints_old = self._joints
            self._joints = None
            self._controllers = None
            self._broadcasters = None
            self.execute(obj)
        if prop == 'OutputPath':
            rel_path = remove_ros_workspace(obj.OutputPath)
            if rel_path != obj.OutputPath:
                obj.OutputPath = rel_path
        if prop == 'Placement':
            self.compute_poses()

    def onDocumentRestored(self, obj):
        """Handle the object after a document restore.

        Required by FreeCAD.

        """
        # `self.__init__()` is not called on document restore, do it manually.
        self.__init__(obj)
        self._created_objects = obj.CreatedObjects
        # Rebuild self._joint_variables from the map {ros_name: joint_variable_name}.
        self._joint_variables = {
            self.get_joint(name): var
            for name, var in self._joint_variables_ros_map.items()
        }

    def dumps(self):
        self.robot.CreatedObjects = self._created_objects
        # Map {ros_name: joint_variable_name}.
        var_map = {ros_name(joint): var for joint, var in self.joint_variables.items()}
        return self.Type, var_map

    def loads(self, state) -> None:
        if state:
            self.Type, self._joint_variables_ros_map = state

    def _reset_group(self) -> None:
        """Add FreeCAD links in CrossLinks for Real, Visual, and Collision."""
        if ((not self.is_execute_ready())
                or (not hasattr(self.robot.ViewObject, 'Proxy'))
                or (not self.robot.ViewObject.Proxy.is_execute_ready())):
            return

        links: list[CrossLink] = self.get_links()
        for link in links:
            link.Proxy.update_fc_links()

    def _cleanup_group(self) -> Optional[DO]:
        """Remove the objects not supported by CROSS::Robot.

        Recursion provoked by modifying `Group` will take care of removing
        the remaining unsupported objects.

        """
        if not self.is_execute_ready():
            return None
        for o in self.robot.Group[::-1]:
            if is_link(o) or is_joint(o) or is_controller(o) or is_broadcaster(o) or is_attached_collision_object(o):
                # Supported.
                continue
            warn_unsupported(o, by='CROSS::Robot', gui=True)
            return self.robot.removeObject(o)
        return None

    def _is_exclusive_to_robot(self, obj: DO) -> bool:
        """Return True if `obj` was created for the Robot."""
        if not self.is_execute_ready():
            return False

        # Special case for Origin objects that are auto-generated by FreeCAD.
        if is_origin(obj):
            return self._is_exclusive_to_robot(obj.InList[0])

        show_objects: DOList = []
        for link in self.get_links():
            if not link.Proxy.is_execute_ready():
                continue
            for o in link.Group:
                show_objects.append(o)
        robot_objects = (
            self.get_links()
            + self.get_joints()
            + self.created_objects
            + show_objects
        )
        return (obj in robot_objects) and len(set(obj.InList) - set(robot_objects)) == 0

    def delete_created_objects(self) -> None:
        """Delete all objects created for the Robot object.

        Objects that are used somewhere else should not be deleted but they are
        removed from `created_objects`.

        Calling this method may create broken links in FreeCAD links added for
        the Real, Visual, and Collision objects. Setting Robot.ViewObject.ShowReal
        and similars to False fixes the issue (deletes the objects) and should
        ideally be done before calling this method.

        This methods does not use any FreeCAD transaction.

        """
        for obj in self.created_objects[::-1]:
            try:
                name = obj.Name
            except RuntimeError:
                # Already deleted.
                continue
            if (
                self._is_exclusive_to_robot(obj)
                and all([self._is_exclusive_to_robot(o) for o in obj.OutList])
            ):
                # Object without "external" parent in the dependency graph.
                # "External" means not in self.created_objects.
                self.robot.Document.removeObject(name)
        self.created_objects.clear()

    def set_joint_enum(self) -> None:
        """Set the enum for Child and Parent of all joints."""
        def get_possible_parent_links(joint: CrossJoint) -> list[str]:
            links: list[str] = []
            for link in self.get_links():
                link_name = ros_name(link)
                if ((joint.Parent == link_name)
                    or (
                        hasattr(link, 'Proxy')
                        and link.Proxy.is_execute_ready()
                    )):
                    links.append(link_name)
            return links

        def get_possible_child_links(joint: CrossJoint) -> list[str]:
            links: list[str] = []
            for link in self.get_links():
                link_name = ros_name(link)
                if ((joint.Child == link_name)
                    or (
                        hasattr(link, 'Proxy')
                        and link.Proxy.is_execute_ready()
                        and link.Proxy.may_be_base_link()
                        and (not link.Proxy.is_in_chain_to_joint(joint))
                        and (joint.Parent != link_name)
                    )):
                    links.append(link_name)
            return links

        sym_diff_joints = set(self.get_joints()).symmetric_difference(set(self.get_joints_old()))
        for joint in sym_diff_joints:
            # We add the empty string to show that the child or parent
            # was not set yet.
            parent_links: list[str] = ['']
            parent_links += get_possible_parent_links(joint)
            child_links: list[str] = ['']
            child_links += get_possible_child_links(joint)
            # Implementation note: setting to a list sets the enumeration.
            if joint.getEnumerationsOfProperty('Parent') != parent_links:
                # Avoid recursive recompute.
                # Doesn't change the value if in the new enum.
                joint.Parent = parent_links
            if joint.getEnumerationsOfProperty('Child') != child_links:
                # Avoid recursive recompute.
                # Doesn't change the value if in the new enum.
                joint.Child = child_links

    def set_joint_values(
            self,
            joint_values: dict[CrossJoint, [float | fc.Units.Quantity]],
    ) -> None:
        """Set the joint values from values in meters and radians.

        Set the joint values of the robot from values in meters and radians or
        from FreeCAD's `Quantity` objects.

        """
        joint_variables: dict[CrossJoint, str] = self.joint_variables
        source_units = {
                'Length': 'm',
                'Angle': 'rad',
        }
        target_units = {
                'Length': 'mm',
                'Angle': 'deg',
        }
        for joint, value in joint_values.items():
            var_name = joint_variables[joint]
            unit_type = joint.Proxy.get_unit_type()
            if isinstance(value, float):
                value = fc.Units.Quantity(f'{value} {source_units[unit_type]}')
            value = quantity_as(value, target_units[unit_type])
            self.update_prop(var_name, value)

    def add_joint_variables(self) -> None:
        """Add a property for each actuated joint."""
        if not self.is_execute_ready():
            return
        self._joint_variables.clear()
        # Get all old variables.
        old_vars: set[str] = set(
            get_properties_of_category(
                self.robot,
                self._category_of_joint_values,
            ),
        )
        # Add a variable for each actuated (supported) joint.
        for joint in self.get_joints():
            var = _add_joint_variable(
                self.robot,
                joint,
                self._category_of_joint_values,
            )
            if var:
                self._joint_variables[joint] = var
        # Remove obsoleted variables.
        for p in old_vars - set(self._joint_variables.values()):
            self.robot.removeProperty(p)
        return

    def compute_poses(self) -> None:
        """Set `Placement` of all joints, links, and attached collision objects.

        Compute and set the pose of all joints, links, and attached collision
        objects in the same frame as the robot.

        """
        joint_cache: dict[CrossJoint, fc.Placement] = {}
        for chain in self.get_chains():
            placement = self.robot.Placement  # A copy.
            for link, joint in grouper(chain, 2):
                if joint in joint_cache:
                    placement = joint_cache[joint]
                    # No need to update the link's placement because we support
                    # only tree structures.
                    continue
                if hasattr(link, 'MountedPlacement'):
                    new_link_placement = placement * link.MountedPlacement
                else:
                    # TODO: find out why `MountedPlacement` is not set.
                    new_link_placement = link.Placement
                if link.Placement != new_link_placement:
                    # Avoid recursive recompute.
                    link.Placement = new_link_placement
                if joint:
                    new_joint_placement = placement * joint.Origin
                    if joint.Placement != new_joint_placement:
                        # Avoid recursive recompute.
                        joint.Placement = new_joint_placement
                    # For next link.
                    placement = (
                        new_joint_placement
                        * joint.Proxy.get_actuation_placement()
                    )
                    joint_cache[joint] = placement
        for aco in self.get_attached_collision_objects():
            if not aco.Link:
                continue
            if aco.Placement != aco.Link.Placement:
                # Avoid recursive recompute.
                aco.Placement = aco.Link.Placement

    def get_attached_collision_objects(self) -> list[CrossAttachedCollisionObject]:
        # TODO: as property.
        if self._attached_collision_objects is not None:
            return list(self._attached_collision_objects)  # A copy.
        if not self.is_execute_ready():
            return []
        self._attached_collision_objects = get_attached_collision_objects(
                self.robot.Group,
        )
        return list(self._attached_collision_objects)  # A copy.

    def get_links(self) -> list[CrossLink]:
        """Return the list of CROSS links in the order of creation."""
        # TODO: as property.
        if self._links is not None:
            return list(self._links)  # A copy.
        if not self.is_execute_ready():
            return []
        self._links = get_links(self.robot.Group)
        return list(self._links)  # A copy.

    def get_joints(self) -> list[CrossJoint]:
        """Return the list of CROSS joints in the order of creation."""
        # TODO: as property.
        if self._joints is not None:
            # self._joints is updated in self.onChanged().
            return list(self._joints)  # A copy.
        if not self.is_execute_ready():
            return []
        self._joints = get_joints(self.robot.Group)
        return list(self._joints)  # A copy.

    def get_joints_old(self) -> list[CrossJoint]:
        """Return the list of CROSS old (before onChanged()) joints in the order of creation."""
        return self._joints_old

    def get_controllers(self) -> list[CrossController]:
        """Return the list of CROSS controllers in the order of creation."""
        # TODO: as property.
        if (self._controllers is not None):
            # self._controllers is updated in self.onChanged().
            return list(self._controllers)  # A copy.
        if not self.is_execute_ready():
            return []
        self._controllers = get_controllers(self.robot.Group)
        return list(self._controllers)  # A copy.


    def get_broadcasters(self) -> list[CrossController]:
        """Return the list of CROSS broadcasters in the order of creation."""
        # TODO: as property.
        if (self._broadcasters is not None):
            # self._broadcasters is updated in self.onChanged().
            return list(self._broadcasters)  # A copy.
        if not self.is_execute_ready():
            return []
        self._broadcasters = get_broadcasters(self.robot.Group)
        return list(self._broadcasters)  # A copy.


    def get_actuated_joints(self) -> list[CrossJoint]:
        """Return the list of CROSS actuated joints in the order of creation."""
        return [j for j in self.get_joints() if j.Type != 'fixed']

    def get_link(self, name: str) -> Optional[CrossLink]:
        """Return the link with ROS name `name`.

        The ROS name is the object's description, or its label if the
        description is empty.

        """
        if not name:
            # Shortcut.
            return None
        for link in self.get_links():
            if ros_name(link) == name:
                return link
        return None

    def get_joint(self, name: str) -> Optional[CrossJoint]:
        """Return the joint with ROS name `name`.

        The ROS name is the object's description, or its label if the
        description is empty.

        """
        if not name:
            # Shortcut.
            return None
        for joint in self.get_joints():
            if ros_name(joint) == name:
                return joint
        return None

    def get_root_link(self) -> Optional[CrossLink]:
        """Return the root link of the robot."""
        chains = self.get_chains(check_kinematics=True)
        if not chains or not chains[0]:
            return None
        return chains[0][0]

    def get_chains(self, check_kinematics=False) -> list[list[BasicElement]]:
        """Return the list of chains.

        A chain starts at the root link, alternates links and joints, and ends
        at the last link of the chain.

        If the last element of a chain would be a joint, that chain is not
        considered.

        """
        if not self.is_execute_ready():
            return []
        if not is_robot(self.robot):
            warn(f'Internal error, {label_or(self.robot)} is not a CROSS::Robot', True)
            return []
        links = self.get_links()
        joints = self.get_joints()
        return get_chains(links, joints, check_kinematics)

    def get_links_fixed_with(self, link_name: str) -> list[CrossLink]:
        """Return the list of links fixed with the specified link.

        The order of the links can be considered as arbitrary.
        If not empty, the returned list contains the specified link itself.
        An empty list indicated an error (link not found).

        """
        if not self.is_execute_ready():
            return []
        link = self.get_link(link_name)
        if not link:
            return []
        chains = self.get_chains()
        out_links: set[CrossLink] = set([link])
        for chain in chains:
            if link not in chain:
                # Shortcut.
                continue
            subchain: list[CrossLink] = []
            # Iterate over joints.
            joints = cast(list[CrossJoint], chain[1::2])
            for joint in joints:
                # No need to check the parent- and child link validity because
                # the joint is part of a chain.
                parent = cast(CrossLink, self.get_link(joint.Parent))
                child = cast(CrossLink, self.get_link(joint.Child))
                if not subchain:
                    # Add the first link.
                    subchain.append(parent)
                if joint.Proxy.is_fixed():
                    subchain.append(child)
                    if (joint is chain[-2]):
                        # Last joint of the chain.
                        out_links.update(subchain)
                        # Next subchain.
                else:
                    if link in subchain:
                        out_links.update(subchain)
                        # Next subchain.
                        break
                    # Link not in subchain, wrong subchain, start a new one.
                    subchain.clear()
                # Next element in the chain.
            # Next chain.
        return list(out_links)

    def get_transform(self, from_link: str, to_link: str) -> Optional[fc.Placement]:
        """Return the current transform between two links.

        Return the current transform, i.e. with the current joint values,
        from link `from_link` to link `to_link`.

        """
        if not self.is_execute_ready():
            return None
        from_ = self.get_link(from_link)
        if not from_:
            return None
        to = self.get_link(to_link)
        if not to:
            return None
        if from_ is to:
            return fc.Placement()
        chains = self.get_chains()
        for chain in chains:
            if from_ not in chain:
                continue
            if to not in chain:
                continue
            from_or_to_found = False
            first_transform = fc.Placement()
            second_transform = fc.Placement()
            for link, joint in grouper(chain, 2):
                if ((not ((link is from_) or (link is to)))
                        and (not from_or_to_found)):
                    continue
                if not from_or_to_found:
                    first_transform = link.Placement
                if (link is from_) and (not from_or_to_found):
                    second = to
                if (link is to) and (not from_or_to_found):
                    second = from_
                from_or_to_found = True
                if link is second:
                    first_to_second = first_transform.inverse() * second_transform
                    if second is to:
                        return first_to_second
                    else:
                        return first_to_second.inverse()
                child = self.get_link(joint.Child)
                if not child:
                    return None
                parent = self.get_link(joint.Parent)
                if not parent:
                    return None
                if (child is second) or (parent is second):
                    second_transform = (
                        joint.Placement
                        * joint.Proxy.get_actuation_placement()
                    )
        return None

    def export_urdf(self, interactive: bool = False) -> Optional[et.Element]:
        """Export the robot as URDF, writing files."""
        if not self.is_execute_ready():
            return None
        if not self.robot.OutputPath:
            # TODO: ask the user for OutputPath.
            warn('Property `OutputPath` cannot be empty', True)
            return
        # TODO: also accept OutputPath as package name in $ROS_WORKSPACE/src.
        p, output_path = get_rel_and_abs_path(self.robot.OutputPath)
        if p != self.robot.OutputPath:
            self.robot.OutputPath = p

        template_files = [
            'package.xml',
            'CMakeLists.txt',
            'launch/description.launch.py',
            'launch/display.launch.py',
            'launch/gazebo.launch.py',
            'rviz/robot_description.rviz',
            'urdf/xacro_wrapper_template.urdf.xacro',
            'urdf/sensors_template.urdf.xacro',
            'worlds/cars_and_trees.sdf',
            'worlds/empty.sdf',
        ]

        write_files = template_files + [
                'meshes/',
                'urdf/',
                'overcross/',
                'worlds/',
        ]

        if interactive and fc.GuiUp:
            diag = FileOverwriteConfirmationDialog(
                    output_path, write_files,
            )
            ignore, write, overwrite = diag.exec_()
            diag.close()
        if set(ignore) == set(write_files):
            # No files to write.
            return
        elif set(write + overwrite) != set(write_files):
            warn(tr('Partial selection of files not supported yet'), True)
            return
        project_path, package_name, description_package_path = split_package_path(output_path)
        # TODO: warn if package name doesn't end with `_description`.
        xml = et.fromstring('<robot/>')
        xml.attrib['name'] = get_valid_urdf_name(self.robot.Label)
        xml.append(
            xml_comment_element(
            'Generated by RobotCAD, a ROS Workbench for FreeCAD ('
            'https://github.com/drfenixion/freecad.robotcad)',
            ),
        )

        for link in self.get_links():
            if not hasattr(link, 'Proxy'):
                error(
                    f"Internal error with '{link.Label}', has no 'Proxy' attribute",
                    True,
                )
                return
            xml.append(link.Proxy.export_urdf(project_path, package_name))

        for joint in self.get_joints():
            if not joint.Parent:
                error(f"Joint '{joint.Label}' has no parent link", True)
                continue
            if not joint.Child:
                error(f"Joint '{joint.Label}' has no child link", True)
                continue
            if hasattr(joint, 'Proxy') and joint.Proxy:
                xml.append(joint.Proxy.export_urdf())
            else:
                error(
                    f"Internal error with joint '{joint.Label}'"
                    ", has no 'Proxy' attribute", True,
                )

        # Save the xml into a file.
        description_package_path.mkdir(parents=True, exist_ok=True)
        urdf_path = get_urdf_path(self.robot, description_package_path)
        urdf_file = urdf_path.name
        root_link=self.get_root_link()

        if root_link == None:
            error('Bad kinematics. Double root link or link not in joint ralationship or empty kinematic chain detected. Fix it and repeat.', True)
            return

        meshes_dir = (
            'meshes '
            if _has_meshes_directory(Path(project_path), package_name)
            else ''
        )

        robot_name = get_valid_urdf_name(ros_name(self.robot))
        controllers_config_file_name = get_controllers_config_file_name(robot_name)
        xacro_wrapper_file = get_xacro_wrapper_file_name(robot_name)
        sensors_file = get_sensors_file_name(robot_name)

        sensors_urdf = self.get_sensors_xml()

        # robot meta for external code generator
        robot_meta = self.get_robot_meta(package_name, urdf_file, meshes_dir, controllers_config_file_name)
        save_file(robot_meta, output_path / f'overcross/robot_meta.xml')

        robot_controllers_yaml = self.get_robot_controllers_yaml()
        save_yaml(robot_controllers_yaml, output_path / f'overcross/{controllers_config_file_name}')

        save_xml(xml, urdf_path)
        export_templates(
            template_files,
            project_path,
            meshes_dir=meshes_dir,
            package_name=package_name,
            urdf_file=urdf_file,
            fixed_frame=ros_name(self.get_root_link()),
            xacro_wrapper_file=xacro_wrapper_file,
            sensors_file=sensors_file,
            sensors_urdf=sensors_urdf,
            robot_name=robot_name,
            cameras_topics_comma_sep=self.get_sensors_topics_by_types(sensor_types = ['camera','depth_camera','wideanglecamera','rgbd_camera'])
        )

        return xml

    def get_sensors_xml(self) -> str:

        sensors_xml = ''
        for sensor_xml_as_dict in self.get_sensors_data().values():
            sensors_xml += '\n\n' + sdf_dict_to_xml(sensor_xml_as_dict, full_document = False, pretty = True)

        return sensors_xml
    
    def get_sensors_topics_by_types(self, sensor_types: list) -> str:
        """Return comma separated sensors topics by sensors type"""

        topics = []
        for sensor_xml_as_dict in self.get_sensors_data().values():
            for sensor_type in sensor_types:
                try:
                    type = sensor_xml_as_dict['gazebo']['sensor']['@type']
                    if type == sensor_type:
                        topic = sensor_xml_as_dict['gazebo']['sensor']['topic']
                        if type == 'rgbd_camera':
                            topics.append("'"+topic+"/image'")
                            topics.append("'"+topic+"/depth_image'")
                        else:
                            topics.append("'"+topic+"'")
                except KeyError:
                    pass                    

        return ','.join(topics)

    def get_sensors_data(self, parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE) -> dict:
        """Get sensors data as sensors dictionaries from all elements (links, joints) of robot"""
        def construct_dict_branch(sensor_data: dict, param_name_level: str, param_full_name_stlited: list, param_value):
            """Construct dict branch dict branch line by recursive travese dict deep levels."""
            if len(param_full_name_stlited):
                param_name_level_deeper = param_full_name_stlited.pop(0)
                if param_name_level in sensor_data:
                    param_level_data = sensor_data[param_name_level]
                else:
                    param_level_data = {}
                sensor_data[param_name_level] = construct_dict_branch(
                    deepcopy(param_level_data), param_name_level_deeper, param_full_name_stlited, param_value,
                )
            else:
                sensor_data[param_name_level] = param_value

            return sensor_data

        def get_sensors(elem: CrossJoint | CrossLink | CrossRobot) -> dict:
            """Get sensors data from element (CrossJoint, CrossLink, CrossRobot)"""
            sensors = {}
            for sensor in elem.Proxy.get_sensors():
                sensor_data = {}
                for param_full_name in sensor.sensor_parameters_fullnames_list:
                    param_full_name_stlited = param_full_name.split(parameter_full_name_glue)
                    param = getattr(sensor, param_full_name)

                    # f.e. level1_name/level1_name/level1_name nested sctruct names in dict
                    param_name_level = param_full_name_stlited.pop(0)
                    sensor_data = construct_dict_branch(sensor_data, param_name_level, param_full_name_stlited, param)

                # sensor_data['@type'] = sensor.sensor_type
                sensor_data['@name'] = get_valid_urdf_name(ros_name(sensor))

                # special case of contact sensor
                # construct collision name (link_name_collision is default collision name)
                if sensor_data[wb_constants.XMLTODICT_ATTR_PREFIX_FIXED_FOR_PROP_NAME + 'type'] == 'contact':
                    sensor_data['contact']['collision'] = get_valid_urdf_name(ros_name(elem)) + '_collision'

                # assemble sensors
                sensors[get_valid_urdf_name(ros_name(sensor))] = {
                    'gazebo':
                        {
                            '@reference': get_valid_urdf_name(ros_name(elem)),
                            'sensor': sensor_data,
                        },
                }

                # revert replace attr prefix to it is origin symbol (@)
                sensors = replace_substring_in_keys(
                    sensors,
                    wb_constants.XMLTODICT_ATTR_PREFIX_FIXED_FOR_PROP_NAME,
                    wb_constants.XMLTODICT_ATTR_PREFIX_ORIGIN,
                )

            return sensors

        sensors = {}
        for link in self.get_links():
            sensors.update(get_sensors(link))
        for joint in self.get_joints():
            sensors.update(get_sensors(joint))

        return sensors


    def get_robot_controllers_yaml(
        self,
        parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
        parameter_full_name_glue_yaml: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE_YAML,
    ) -> dict:
        """Make robot controllers data in yaml format"""

        def add_controllers_types_to_yaml(controller: CrossController, yaml_data: dict):
            plugin_class_name = getattr(controller, 'plugin_class_name')
            yaml_data['controller_manager']['ros__parameters'][get_valid_urdf_name(ros_name(controller))] = {'type': plugin_class_name}
            return yaml_data


        def separate_controllers_from_other_elements(param: list) -> tuple[list, list]:
            """ Separate controllers and broadcasters from other elements (joints) in param elements list"""

            controllers_broadcasters = []
            others = []
            for el in param:
                if is_controller(el) or is_broadcaster(el):
                    controllers_broadcasters.append(el)
                else:
                    others.append(el)

            return controllers_broadcasters, others


        def add_controllers_data_to_yaml(controller: CrossController, yaml_data: dict):
            yaml_data[get_valid_urdf_name(ros_name(controller))] = {'ros__parameters': {}}
            for param_full_name in controller.controller_parameters_fullnames_list:
                param = getattr(controller, param_full_name)

                if not is_controllers_template_for_param_mapping(param_full_name):
                    param_full_name_yaml = param_full_name.replace(parameter_full_name_glue, parameter_full_name_glue_yaml)

                    if isinstance(param, (float, int, str, type(None))):
                        yaml_data[get_valid_urdf_name(ros_name(controller))]['ros__parameters'][param_full_name_yaml] = param

                        # case of parallel_gripper_action_controller
                        # clear empty interfaces for use some default value for interface if posible (don`t sure there is any default)
                        if param_full_name_yaml in ['max_effort_interface', 'max_velocity_interface'] \
                        and yaml_data[get_valid_urdf_name(ros_name(controller))]['ros__parameters'][param_full_name_yaml] == '':
                            del yaml_data[get_valid_urdf_name(ros_name(controller))]['ros__parameters'][param_full_name_yaml]

                    elif isinstance(param, DO):
                        yaml_data[get_valid_urdf_name(ros_name(controller))]['ros__parameters'][param_full_name_yaml] = get_valid_urdf_name(ros_name(param))
                    else:
                        yaml_data[get_valid_urdf_name(ros_name(controller))]['ros__parameters'][param_full_name_yaml] = []
                        controllers_broadcasters, others = separate_controllers_from_other_elements(param)

                        # make chain reference names - ex: controller_name/joint_name
                        if len(controllers_broadcasters):
                            for cont_broad in controllers_broadcasters:
                                cont_broad_name = cont_broad
                                if isinstance(cont_broad, DO):
                                    cont_broad_name = get_valid_urdf_name(ros_name(cont_broad))
                                for other_el in others:
                                    other_el_name = other_el
                                    if isinstance(other_el, DO):
                                        other_el_name = get_valid_urdf_name(ros_name(other_el))
                                    chain_name = cont_broad_name + '/' + other_el_name
                                    yaml_data[get_valid_urdf_name(ros_name(controller))]['ros__parameters'][param_full_name_yaml].append(chain_name)

                        # make regular names - ex: joint_name
                        else:
                            for other_el in others:
                                other_el_name = other_el
                                if isinstance(other_el, DO):
                                    other_el_name = get_valid_urdf_name(ros_name(other_el))
                                yaml_data[get_valid_urdf_name(ros_name(controller))]['ros__parameters'][param_full_name_yaml].append(other_el_name)

                        # clear empty list
                        if not len(yaml_data[get_valid_urdf_name(ros_name(controller))]['ros__parameters'][param_full_name_yaml]):
                            # control manager throw error when detect empty list
                            # delete empty element
                            del yaml_data[get_valid_urdf_name(ros_name(controller))]['ros__parameters'][param_full_name_yaml]

            return yaml_data


        git_init_submodules()

        yaml_data = {}
        yaml_data['controller_manager'] = {'ros__parameters': {}}
        # controller manager and controllers plugin types
        yaml_data['controller_manager']['ros__parameters'] = {'update_rate': 250}
        for controller in self.get_controllers():
            yaml_data = add_controllers_types_to_yaml(controller, yaml_data)
        for controller in self.get_broadcasters():
            yaml_data = add_controllers_types_to_yaml(controller, yaml_data)

        # controllers and their params
        for controller in self.get_controllers():
            yaml_data = add_controllers_data_to_yaml(controller, yaml_data)
        for controller in self.get_broadcasters():
            yaml_data = add_controllers_data_to_yaml(controller, yaml_data)
        return yaml_data


    def get_robot_meta(self, package_name: str, urdf_file_name: str, meshes_dir: str, controllers_config_file_name: str) -> str:
        """Return robot meta info as xml. It can be used by external code generators"""

        robotMetaXml = f"""<?xml version="1.0" ?>
<robotMeta>
    <robotName>{get_valid_urdf_name(ros_name(self.robot))}</robotName>
    <urdfFileName>{urdf_file_name}</urdfFileName>
    <packageName>{package_name}</packageName>
    <meshesDir>{meshes_dir}</meshesDir>
    <robotType>{self.robot.RobotType}</robotType>
    <generateCodeForRosVersion>{self.robot.GenerateCodeForRosVersion}</generateCodeForRosVersion>
    <rootLinkName>{get_valid_urdf_name(ros_name(self.get_root_link()))}</rootLinkName>
    <xacroWrapperFileName>{get_xacro_wrapper_file_name(ros_name(self.robot))}</xacroWrapperFileName>
    <controllersConfigFileName>{controllers_config_file_name}</controllersConfigFileName>
</robotMeta>
        """

        jointsXml = f"""
    <joints>
        """

        for joint in self.get_joints():

            jointsXml += f"""
        <joint>
            <name>{get_valid_urdf_name(ros_name(joint))}</name>
            <jointSpecific>{joint.JointSpecific}</jointSpecific>
            <jointRotationDirection>{joint.JoinRotationDirection}</jointRotationDirection>
            <jointRelTotalCenterOfMass_x>{quantity_as(fc.Units.Quantity(str(joint.PlacementRelTotalCenterOfMass.Base.x) + ' mm'), 'm')}</jointRelTotalCenterOfMass_x>
            <jointRelTotalCenterOfMass_y>{quantity_as(fc.Units.Quantity(str(joint.PlacementRelTotalCenterOfMass.Base.y) + ' mm'), 'm')}</jointRelTotalCenterOfMass_y>
            <jointRelTotalCenterOfMass_z>{quantity_as(fc.Units.Quantity(str(joint.PlacementRelTotalCenterOfMass.Base.z) + ' mm'), 'm')}</jointRelTotalCenterOfMass_z>
        </joint>
        """

        jointsXml += f"""
    </joints>
        """


        robotMetaXmlDoc = parseString(robotMetaXml)
        jointsXmlDoc = parseString(jointsXml)
        robotMetaXmlDoc.getElementsByTagName("robotMeta").item(0).appendChild(jointsXmlDoc.getElementsByTagName("joints").item(0))
        robotMetaXml = robotMetaXmlDoc.toprettyxml(indent=' ', newl='\n', encoding=None, standalone=None)

        return robotMetaXml


class _ViewProviderRobot(ProxyBase):
    """A view provider for the Robot container object """

    def __init__(self, vobj: VPDO):
        super().__init__(
            'view_object',
            [
                'JointAxisLength',
                'ShowCollision',
                'ShowJointAxes',
                'ShowReal',
                'ShowVisual',
                'Visibility',
            ],
        )
        if vobj.Proxy is not self:
            # Implementation note: triggers `self.attach`.
            vobj.Proxy = self
        self._init(vobj)

    def _init(self, vobj: VPDO) -> None:
        self.view_object = vobj
        self.robot = vobj.Object
        self._init_properties(vobj)

    def _init_properties(self, vobj: VPDO) -> None:
        # Level of detail.
        add_property(
            vobj, 'App::PropertyBool', 'ShowReal', 'ROS Display Options',
            'Whether to show the real parts',
        )
        add_property(
            vobj, 'App::PropertyBool', 'ShowVisual', 'ROS Display Options',
            'Whether to show the parts for URDF visual',
        )
        add_property(
            vobj, 'App::PropertyBool', 'ShowCollision', 'ROS Display Options',
            'Whether to show the parts for URDF collision',
        )

        # Joint display options.
        add_property(
            vobj, 'App::PropertyBool', 'ShowJointAxes',
            'ROS Display Options',
            'Toggle the display of the Z-axis for all child joints',
            True,
        )
        add_property(
            vobj, 'App::PropertyLength', 'JointAxisLength',
            'ROS Display Options',
            "Length of the arrow for the joints axes",
            250.0,
        )

    def getIcon(self):
        # Implementation note: "return 'robot.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'robot.svg')

    def attach(self, vobj: VPDO):
        # `self.__init__()` is not called on document restore, do it manually.
        self.__init__(vobj)

        # vobj.addExtension('Gui::ViewProviderGeoFeatureGroupExtensionPython')

    def updateData(self, obj: CrossRobot, prop: str):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        robot: CrossRobot = vobj.Object

        if prop == 'ShowJointAxes':
            _dispatch_to_joint_view_objects(robot, prop, 'ShowAxis')
        if prop == 'JointAxisLength':
            _dispatch_to_joint_view_objects(robot, prop, 'AxisLength')
        if prop in ['ShowReal', 'ShowVisual', 'ShowCollision']:
            _dispatch_to_link_view_objects(robot, prop, prop)
            # robot.Proxy.execute(robot)

    def setupContextMenu(self, vobj: VPDO, menu: QMenu) -> None:
        action = menu.addAction("Load trajectories from YAML...")
        action.triggered.connect(
                lambda function=self.load_trajectories_from_yaml, argument=vobj: function(argument),
        )

    def doubleClicked(self, vobj: VPDO):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj: VPDO, mode):
        return False

    def unsetEdit(self, vobj: VPDO, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()

    def dumps(self):
        return None

    def loads(self, state) -> None:
        pass

    def load_trajectories_from_yaml(self, vobj: VPDO) -> None:
        # Import late to avoid slowing down workbench start-up.
        import FreeCADGui as fcgui
        import yaml
        from .trajectory_proxy import make_trajectory

        dialog = QFileDialog(
            fcgui.getMainWindow(),
            'Select a multi-doc YAML file to import trajectories from',
        )
        dialog.setNameFilter('YAML *.yaml;;All files (*.*)')
        if dialog.exec_():
            filename = str(dialog.selectedFiles()[0])
        else:
            return

        display_trajs = yaml.safe_load_all(open(filename))

        fcgui.Selection.clearSelection()
        for display_traj in display_trajs:
            if not display_traj:
                continue
            traj_obj = make_trajectory('Trajectory', vobj.Object.Document)
            traj_obj.Robot = vobj.Object
            traj_obj.Proxy.load_display_trajectory_dict(display_traj)
            fcgui.Selection.addSelection(traj_obj)

        vobj.Object.Document.recompute()
        fcgui.runCommand('Std_ToggleFreeze', 0)


def make_robot(name, doc: Optional[fc.Document] = None) -> CrossRobot:
    """Add a Cross::Robot to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    robot: CrossRobot = doc.addObject('App::DocumentObjectGroupPython', name)
    # robot = doc.addObject('Part::FeaturePython', name)
    RobotProxy(robot)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderRobot(robot.ViewObject)
        robot.ViewObject.ShowReal = True
        robot.ViewObject.ShowVisual = False
        robot.ViewObject.ShowCollision = True

    doc.recompute()
    return robot


def make_filled_robot(name:str = 'Robot') -> CrossRobot:
    """Add a Cross::Robot to the current document and fill it with links and joints."""

    robot = make_robot(name)
    links = make_robot_links_filled(robot = robot)
    fc.ActiveDocument.recompute()

    if len(links):
        make_robot_joints_filled(links, robot)

    fc.ActiveDocument.recompute()
    return robot


def get_assembly_elements(assembly:DO, root_grounded_joint_of_root_assembly:DO = None, assembly_link:DO = None) -> tuple:
    """Get assemble links, joints, grounded_joints from
    from assembly (default Assembly WB)."""

    if not is_assembly_from_assembly_wb(assembly) and not is_link_to_assembly_from_assembly_wb(assembly):
        raise TypeError('Not an assembly from Assembly WB')

    doc = assembly.Document

    # separate assembly sub elements
    assembly_links = []
    assembly_joints = []
    grounded_joints = []

    # Assembly in other assembly does not included joints 
    # it need to get linked assembly for it joints.
    for el in assembly.Group:
        if is_grounded_join_from_assembly_wb(el) and not root_grounded_joint_of_root_assembly:
            root_grounded_joint_of_root_assembly = el
            break

    for el in assembly.Group:
        if is_fc_link(el):
            assembly_links.append(el)
        elif is_join_from_assembly_wb(el):
            r1 = el.Reference1[1][0]
            r1_name_path = r1.split('.')
            r1_obj0_link = doc.getObject(r1_name_path[0])

            r2 = el.Reference2[1][0]
            r2_name_path = r2.split('.')
            r2_obj0_link = doc.getObject(r2_name_path[0])

            is_link2_root_assembly_link = False
            is_link1_root_assembly_link = False
            if root_grounded_joint_of_root_assembly.ObjectToGround.Name == r2_name_path[0]:
                is_link2_root_assembly_link = True
            if root_grounded_joint_of_root_assembly.ObjectToGround.Name == r1_name_path[1]:
                is_link1_root_assembly_link = True
            
            r2_obj_link = r2_obj0_link
            r1_obj_link = r1_obj0_link
            new_assembly_branch_r2 = None
            new_assembly_branch_r1 = None
            if is_link_to_assembly_from_assembly_wb(r1_obj0_link):
                new_assembly_branch_r1 = r1_obj0_link.LinkedObject
                r1_obj1_link = doc.getObject(r1_name_path[1])
                # get child link (root of new assembly branch) from it source assembly not from assembly link
                r1_obj1_link = r1_obj1_link.getLinkedObject(False)
                r1_obj_link = r1_obj1_link
            if is_link_to_assembly_from_assembly_wb(r2_obj0_link):
                new_assembly_branch_r2 = r2_obj0_link.LinkedObject
                r2_obj1_link = doc.getObject(r2_name_path[1])
                # get child link (root of new assembly branch) from it source assembly not from assembly link
                r2_obj1_link = r2_obj1_link.getLinkedObject(False)
                r2_obj_link = r2_obj1_link

            assembly_joints.append({
                    'joint': el,
                    'assembly': assembly,
                    'assembly_link': assembly_link,
                    'is_root_joint_to_new_assembly_branch': True,
                    'new_assembly_branch_r2': new_assembly_branch_r2,
                    'new_assembly_branch_r1': new_assembly_branch_r1,
                    'link2': r2_obj_link,
                    'link1': r1_obj_link,
                    'link2_name': r2_obj_link.Name,
                    'link1_name': r1_obj_link.Name,
                    'is_link2_root_assembly_link': is_link2_root_assembly_link,
                    'is_link1_root_assembly_link': is_link1_root_assembly_link,
                    'chain_direction': None,
                })

        elif is_grounded_join_from_assembly_wb(el):
            grounded_joints.append(el)
        elif is_link_to_assembly_from_assembly_wb(el):
            assembly_links_rec, assembly_joints_rec, grounded_joint_rec = get_assembly_elements(el.LinkedObject, root_grounded_joint_of_root_assembly, el)
            assembly_links = assembly_links + assembly_links_rec
            assembly_joints = assembly_joints + assembly_joints_rec
            grounded_joints = grounded_joints + grounded_joint_rec
    
    return assembly_links, assembly_joints, grounded_joints


def make_filled_robot_from_assembly(assembly:DO, robot:CrossRobot = None) -> CrossRobot:
    """Add a Cross::Robot to the current document and fill it with links and joints
    from assembly (default Assembly WB)."""

    if not robot:
        robot = make_robot(ros_name(assembly))

    assembly_links, assembly_joints, grounded_joints = get_assembly_elements(assembly)

    # create robot links based on assembly links
    robot_links:list[CrossLink] = []
    for link in assembly_links:
        obj = link.getLinkedObject(True)
        res = make_robot_link_filled(obj, True)
        if is_link(res):
            robot_link = res
            # robot_link.Placement = link.Placement
            robot_links.append(robot_link)
            if robot:
                robot_link.adjustRelativeLinks(robot)
                robot.addObject(robot_link)

    # make root joint on top
    assembly_joints_sorted = sorted(
        assembly_joints, 
        key=lambda x: x['is_link2_root_assembly_link'] or x['is_link1_root_assembly_link'],
        reverse=True
    )

    # make chain
    assembly_joints_chain = []
    root_joint = assembly_joints_sorted[0]
    assembly_joints_sorted.remove(root_joint) # we will use root_joint separated
    
    def get_next_child_joint(joint) -> DO:
        for i in assembly_joints_sorted:
            if joint['link1'].Name == i['link1'].Name or joint['link2'].Name == i['link2'].Name \
            or joint['link2'].Name == i['link1'].Name:
                # revers chain direction
                assembly_joints_sorted.remove(i)
                i['chain_direction'] = 'reverse'
                yield i
                break
            elif joint['link1'].Name == i['link2'].Name:
            # or joint['link2'].Name in [i['link1'].Name, i['link2'].Name]:
                # forward chain direction
                assembly_joints_sorted.remove(i)
                i['chain_direction'] = 'forward'
                yield i
                break
                

    def get_next_branch_root_joint(joint) -> DO:
        for i in assembly_joints_sorted:
            #TODO check direction of chain
            if joint['link1'].Name == i['link1'].Name or joint['link2'].Name == i['link2'].Name \
            or joint['link2'].Name == i['link1'].Name:
                # revers chain direction
                assembly_joints_sorted.remove(i)
                i['chain_direction'] = 'reverse'
                yield i
                break
            elif joint['link1'].Name == i['link2'].Name:
            # or joint['link2'].Name in [i['link1'].Name, i['link2'].Name]:
                # forward chain direction
                assembly_joints_sorted.remove(i)
                i['chain_direction'] = 'forward'
                yield i
                break

    while len(assembly_joints_sorted):
        if root_joint:
            assembly_joints_chain.append(root_joint)
            child_joint = next(get_next_child_joint(root_joint))
            assembly_joints_chain.append(child_joint)
            root_joint = None
        else:
            try:
                child_joint = next(get_next_child_joint(child_joint))
                assembly_joints_chain.append(child_joint)
            except StopIteration:
                for chain_joint in assembly_joints_chain:
                    try:
                        child_joint = next(get_next_branch_root_joint(chain_joint))
                        assembly_joints_chain.append(child_joint)
                        break
                    except StopIteration:
                        pass

    # create robot joints based on assembly joints
    for joint in assembly_joints_chain:
        # prepare data
        r1 = joint['joint'].Reference1[1][0]
        r2 = joint['joint'].Reference2[1][0]
        p1 = joint['joint'].Placement1
        p2 = joint['joint'].Placement2
        o1 = joint['joint'].Offset1
        o2 = joint['joint'].Offset1
        # joint['chain_direction']
        r1_name_path = r1.split('.')
        r2_name_path = r2.split('.')
        r1_obj_link = assembly.Document.getObject(r1_name_path[0])
        r2_obj_link = assembly.Document.getObject(r2_name_path[0])
        if is_link_to_assembly_from_assembly_wb(r1_obj_link):
            r1_obj_link = assembly.Document.getObject(r1_name_path[1])
        if is_link_to_assembly_from_assembly_wb(r2_obj_link):
            r2_obj_link = assembly.Document.getObject(r2_name_path[1])
        r1_obj = r1_obj_link.getLinkedObject(True)
        r2_obj = r2_obj_link.getLinkedObject(True)
        parent_robot_link = None
        child_robot_link = None
        for robot_link in robot_links:
            if is_part(robot_link.Real[0]):
                for real_sub_el in robot_link.Real[0].Group:
                    if is_fc_link(real_sub_el):
                        real_sub_el = real_sub_el.getLinkedObject(True)

                    if r1_obj.Name == real_sub_el.Name:
                        if not joint['chain_direction'] or joint['chain_direction'] == 'forward':
                            child_robot_link = robot_link
                        else:
                            parent_robot_link = robot_link
                    elif r2_obj.Name == real_sub_el.Name:
                        if joint['chain_direction'] == 'reverse':
                            child_robot_link = robot_link
                        else:
                            parent_robot_link = robot_link
        
        if not child_robot_link:
            break

        # calc robot link MountedPlacement 
        mounted_placement = fc.Placement()
        link_assembly_placement = fc.Placement()
        # it need to adding assembly link placement to it`s elements (links)
        # when assembly in other assembly case
        # for get placement of elements relative to assembly link

        # if not joint['chain_direction'] or joint['chain_direction'] == 'forward':
        elem0 = assembly.Document.getObject(r1_name_path[0])
        if is_link_to_assembly_from_assembly_wb(elem0):
            link_assembly_placement = elem0.Placement
        sub_el = assembly.Document.getObject(r1_name_path[1])
        if is_lcs(sub_el):
            mounted_placement = sub_el.Placement * o1
        else: # face of obj case
            mounted_placement = p1 * o1

        # else:
        elem0 = assembly.Document.getObject(r2_name_path[0])
        if is_link_to_assembly_from_assembly_wb(elem0):
            link_assembly_placement = elem0.Placement
        sub_el = assembly.Document.getObject(r2_name_path[1])
        if is_lcs(sub_el):
            mounted_placement_r2 = sub_el.Placement * o2
        else: # face of obj case
            mounted_placement_r2 = p2 * o2


        if not joint['chain_direction'] or joint['chain_direction'] == 'forward':
            child_robot_link.MountedPlacement = mounted_placement.inverse()
            origin_mounted_placement_correction = mounted_placement
            origin_obj_link_correction = r1_obj_link.Placement
        else:
            # child_robot_link.MountedPlacement = mounted_placement.inverse()
            child_robot_link.MountedPlacement = mounted_placement_r2.inverse()
            origin_mounted_placement_correction = mounted_placement_r2
            origin_obj_link_correction = r2_obj_link.Placement

        # calc joint Origin
        robot_joint = make_robot_joint_filled(parent_robot_link, child_robot_link, robot)
        chain = get_chain(child_robot_link)
        comulative_joint_placement = fc.Placement()
        for el in chain:
            if is_joint(el):
                comulative_joint_placement = comulative_joint_placement * el.Origin

        assembly_link_placement = fc.Placement()
        if joint['assembly_link']:
            assembly_link_placement = joint['assembly_link'].Placement
        
        robot_joint.Origin = comulative_joint_placement.inverse() * assembly_link_placement * link_assembly_placement \
            * origin_obj_link_correction * origin_mounted_placement_correction
    
    return robot
