from __future__ import annotations

from pathlib import Path
from typing import NewType, List, Optional, cast
import xml.etree.ElementTree as et

import FreeCAD as fc
import FreeCADGui as fcgui
from freecad.cross.freecadgui_utils import get_sorted_concated_names

from .freecad_utils import ProxyBase, is_body, volume_mm3
from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import is_link as is_freecad_link
from .freecad_utils import warn
from .freecad_utils import message
from .freecad_utils import add_object
from .freecad_utils import is_part
from .freecad_utils import is_derived_from
from .mesh_utils import save_mesh_dae
from .urdf_utils import XmlForExport
from .urdf_utils import urdf_collision_from_object
from .urdf_utils import urdf_inertial
from .urdf_utils import urdf_visual_from_object
from .utils import attr_equals
from .utils import warn_unsupported
from .wb_utils import ICON_PATH, get_link_sensors
from .wb_utils import get_vacuum_grippers
from .wb_utils import get_chain
from .wb_utils import get_joints
from .wb_utils import get_links
from .wb_utils import get_valid_urdf_name
from .wb_utils import is_joint
from .wb_utils import is_link
from .wb_utils import is_sensor_link
from .wb_utils import is_vacuum_gripper
from .wb_utils import is_name_used
from .wb_utils import is_primitive
from .wb_utils import is_robot
from .wb_utils import ros_name
from .wb_utils import get_parent_link_of_obj
from . import wb_constants

# Stubs and typing hints.
from .joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
from .sensors.sensor import Sensor as CrossSensor  # A Cross::Sensor, i.e. a DocumentObject with Proxy "SensorProxyJoint" or "SensorProxyJoint". # noqa: E501
DO = fc.DocumentObject
CrossVacuumGripper = DO  # A Cross::VacuumGripper.
DOList = List[DO]
VPDO = NewType('FreeCADGui.ViewProviderDocumentObject', DO)  # Don't want to import FreeCADGui here. # noqa: E501
AppLink = DO  # TypeId == 'App::Link'.


def _add_fc_links_lod(
        link: CrossLink,
        objects: DOList,
        lod: str,
) -> list[AppLink]:
    """Create FreeCAD links to real, visual or collision elements.

    Return the list of created FreeCAD link objects.
    The objects are not added to the CROSS::link (it's a group), just to the
    document.

    Parameters
    ----------
    - link: a FreeCAD object of type Cross::Link.
    - objects: the list of objects to potentially add.
    - lod: string describing the level of details, {'real', 'visual',
            'collision'}.

    """
    doc = link.Document
    fc_links: DOList = []
    for o in objects:
        name = f'{lod}_{link.Label}_'
        lod_link = doc.addObject('App::Link', name)
        lod_link.Label = name
        lod_link.LinkPlacement = link.Placement
        lod_link.setLink(o)
        lod_link.adjustRelativeLinks(link)
        fc_links.append(lod_link)
    return fc_links


def _skim_links_joints_from(group) -> tuple[DOList, DOList]:
    """Remove all Cross::Link and Cross::Joint from the list.

    `group` is a property that looks like a list but behaves differently
    (behaves like a tuple and is a copy of the original property content,
    so cannot be changed here).

    Return (kept_objects, removed_objects).

    """
    removed_objects: DOList = []
    kept_objects: DOList = list(group)
    # Implementation note: reverse order required.
    for i, o in reversed(list(enumerate(kept_objects))):
        if is_link(o) or is_joint(o):
            warn_unsupported(o, by='CROSS::Link', gui=True)
            # Implementation note: cannot use `kept_objects.remove`, this would
            # lose the object.
            removed_objects.append(kept_objects.pop(i))
    return kept_objects, removed_objects


def _get_xmls_and_export_meshes(
        obj,
        urdf_function,
        placement,
        package_parent: [Path | str] = Path(),
        package_name: str = '',
) -> list[et.Element]:
    """
    Save the meshes as dae files.

    Parameters
    ----------
    - obj: object to create the URDF for
    - urdf_function: {urdf_visual_from_object, urdf_collision_from_object}
    - placement: placement of the object relative to the joint
                 (MountedPlacement)
    - package_parent: where to find the ROS package
    - package_name: name of the ROS package, also name of the directory where
                    to save the package.

    """
    export_data: list[XmlForExport] = urdf_function(
        obj,
        package_name=str(package_name),
        placement=placement,
    )
    xmls: list[et.Element] = []
    for export_datum in export_data:
        if is_body(export_datum.object) and volume_mm3(export_datum.object) <= 0.0:
            # dont create visuals and meshes for LCS_wrapper. If create Gazebo will error about empty mesh.
            continue
        if is_part(export_datum.object):
            any_with_volume_inside = False
            for el in export_datum.object.OutListRecursive:
                if el.TypeId not in ['App::Line', 'App::Plane', 'App::Origin']:
                    any_with_volume_inside = True
            # dont create visuals and meshes for empty App::Part
            if not any_with_volume_inside:
                continue
        if not is_primitive(export_datum.object):
            mesh_path = (
                package_parent / package_name
                / 'meshes' / export_datum.mesh_filename
            )
            save_mesh_dae(export_datum.object, mesh_path)
        xmls.append(export_datum.xml)
    return xmls


class LinkProxy(ProxyBase):
    """Proxy for CROSS::Link objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::Link'

    def __init__(self, obj: CrossLink):
        super().__init__(
            'link',
            [
                'Collision',
                'Group',
                'Mass',
                'MountedPlacement',
                'Placement',
                'Real',
                'Visual',
                'MaterialCardName',
                'MaterialCardPath',
                'MaterialDensity',
                'MaterialNotCalculate',
                'CalculateInertiaBasedOnMass',
                'AssemblyReference',                
                '_Type',
            ],
        )
        if obj.Proxy is not self:
            obj.Proxy = self
        self.link = obj

        # Lists to keep track of the objects that were added to the
        # CROSS::link.
        self._fc_links_real: DOList = []
        self._fc_links_visual: DOList = []
        self._fc_links_collision: DOList = []

        # Used to recover a valid and unique name on change of `Label` or
        # `Label2`.
        # Updated in `onBeforeChange` and potentially used in `onChanged`.
        self.old_ros_name: str = ''

        # Prevent recursion when this proxy updates `Real`, `Visual` or
        # `Collision` itself.
        self._is_wrapping_elements: bool = False

        # True while the document object is being restored, when property
        # values must not be interpreted as manual user edits.
        self._is_restoring: bool = False

        # Snapshot of `Real`, `Visual` and `Collision` values before a change
        # (filled in `onBeforeChange`).
        self._elements_before_change: dict = {}

        # Save the robot to speed-up self.get_robot().
        self._robot: Optional[CrossRobot] = None

        # Save the parent joint to speed-up self.get_ref_joint().
        self._ref_joint: Optional[CrossJoint] = None
        # Save the child joints to speed-up self.get_ref_child_joints().
        self._ref_child_joints: Optional[list[CrossJoint]] = []

        self._sensors: Optional[list[CrossSensor]] = None
        self._vacuum_grippers: Optional[list[CrossVacuumGripper]] = None

        self.init_extensions(obj)
        self.init_properties(obj)

    def init_extensions(self, obj: CrossLink) -> None:
        # Need a group to put the generated FreeCAD links in.
        obj.addExtension('App::GroupExtensionPython')

    def init_properties(self, obj: CrossLink):
        add_property(
            obj, 'App::PropertyString', '_Type', 'Internal',
            'The type of object',
        )
        add_property(
            obj, 'App::PropertyString', 'AssemblyReference', 'Internal',
            'The type of object',
        )        
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type
        obj.setPropertyStatus('AssemblyReference', ['Hidden', 'ReadOnly'])
        

        add_property(
            obj, 'App::PropertyLinkListGlobal', 'Real', 'Elements',
            'The real part objects of this link, optional',
        )
        add_property(
            obj, 'App::PropertyLinkListGlobal', 'Visual', 'Elements',
            'The part objects this link that constitutes the URDF'
            ' visual elements, optional',
        )
        add_property(
            obj, 'App::PropertyLinkListGlobal', 'Collision', 'Elements',
            'The part objects this link that constitutes the URDF'
            ' collision elements, optional',
        )

        add_property(
            obj, 'App::PropertyQuantity', 'Mass', 'Inertial Parameters',
            'Mass of the link',
        )
        obj.Mass = fc.Units.Mass
        add_property(
            obj, 'App::PropertyPlacement', 'CenterOfMass', 'Inertial Parameters',
            'Center of mass of the link, with orientation determining the principal axes of inertia',
        )
        add_property(
            obj, 'App::PropertyPlacement', 'CenterOfMassGlobalCoords', 'Inertial Parameters',
            'Center of mass of the link, with orientation determining the principal axes of inertia \
                        in global coordinates.',
        )
        # Implementation note: App.Units.MomentOfInertia is not a valid unit in
        # FC v0.21.
        add_property(
            obj, 'App::PropertyFloat', 'Ixx', 'Inertial Parameters',
            'Moment of inertia around the x axis, in kg m^2',
        )
        add_property(
            obj, 'App::PropertyFloat', 'Ixy', 'Inertial Parameters',
            'Moment of inertia around the y axis when rotating around the x axis, in kg m^2',
        )
        add_property(
            obj, 'App::PropertyFloat', 'Ixz', 'Inertial Parameters',
            'Moment of inertia around the z axis when rotating around the x axis, in kg m^2',
        )
        add_property(
            obj, 'App::PropertyFloat', 'Iyy', 'Inertial Parameters',
            'Moment of inertia around the y axis, in kg m^2',
        )
        add_property(
            obj, 'App::PropertyFloat', 'Iyz', 'Inertial Parameters',
            'Moment of inertia around the z axis when rotating around the y axis, in kg m^2',
        )
        add_property(
            obj, 'App::PropertyFloat', 'Izz', 'Inertial Parameters',
            'Moment of inertia around the z axis, in kg m^2',
        )

        add_property(
            obj, 'App::PropertyPlacement', 'Placement', 'Internal',
            'Placement of elements in the robot frame',
        )

        add_property(
            obj, 'App::PropertyString', 'MaterialCardName', 'Material',
            'Material of element. Used to calculate mass and inertia. Use "Set material" tool to change',
        )
        obj.setPropertyStatus('MaterialCardName', ['ReadOnly'])
        add_property(
            obj, 'App::PropertyPath', 'MaterialCardPath', 'Material',
            'Material of element. Used to calculate mass and inertia',
        )
        obj.setPropertyStatus('MaterialCardPath', ['Hidden', 'ReadOnly'])
        add_property(
            obj, 'App::PropertyString', 'MaterialDensity', 'Material',
            (
                'Density of the material. Used to calculate the mass. May be'
                ' outdated if you updated the material density outside the'
                ' RobotCAD workbench. The actual density will taken from the'
                ' material (material editor) at the moment the mass is'
                ' calculated.'
            ),
        )
        obj.setPropertyStatus('MaterialDensity', ['ReadOnly'])
        add_property(
            obj, 'App::PropertyBool', 'MaterialNotCalculate', 'Material',
            (
                'If true this material will be not used to calculate mass and'
                ' inertia of this link. If true, the filled mass and inertia'
                ' will not be changed.'
            ),
        )
        add_property(
            obj, 'App::PropertyBool', 'CalculateInertiaBasedOnMass', 'Inertial Parameters',
            'If true and the Mass property is greater than 0, the Mass property will override the material data and be used to calculate the element\'s inertia',
        )

        # Used when adding a link which shape in located at the origin but
        # looks correctly placed. For example, when opening a STEP file or a
        # mesh with all links at the mounted position.
        # This placement is the transform from origin to the location of the
        # joint that is parent of this link.
        add_property(
            obj, 'App::PropertyPlacement', 'MountedPlacement',
            'ROS Parameters', 'Shapes placement',
        )

        self._set_property_modes()

    def execute(self, obj: CrossLink) -> None:
        pass

    def onBeforeChange(self, obj: CrossLink, prop: str) -> None:
        """Called before a property of `obj` is changed."""
        # TODO: save the old ros_name and update all joints that used it.
        if not hasattr(self, '_elements_before_change'):
            # Old proxy instances, or a proxy not yet initialized (e.g. while
            # restoring a document created by an older version, properties can
            # be set before `__init__` has run).
            self._elements_before_change = {}
        if prop in ['Label', 'Label2']:
            robot = self.get_robot()
            if (robot and is_name_used(obj, robot)):
                self.old_ros_name = ''
            else:
                self.old_ros_name = ros_name(obj)
        if prop in ('Real', 'Visual', 'Collision'):
            self._elements_before_change[prop] = list(getattr(obj, prop, []))

    def onChanged(self, obj: CrossLink, prop: str) -> None:
        if prop == 'Group':
            self._sensors = None
            self._vacuum_grippers = None
            self._cleanup_children()
        if prop in ('Real', 'Visual', 'Collision'):
            self._wrap_manually_added_elements(prop)
            self.update_fc_links()
            self._cleanup_children()
        if prop in ('Label', 'Label2'):
            robot = self.get_robot()
            
            if robot and hasattr(robot, 'Proxy'):
                robot.Proxy.set_joint_enum()

            if (
                robot
                and is_name_used(obj, robot)
                and getattr(obj, prop) != self.old_ros_name
            ):
                setattr(obj, prop, self.old_ros_name)
            else:
                # Update Parent and Child joints that reference the old Label
                if robot and self.old_ros_name:
                    old_label = self.old_ros_name
                    new_label = getattr(obj, prop)
                    joints = get_joints(robot.Group)
                    for joint in joints:
                        # Update Parent if it matches old Label
                        if hasattr(joint, 'Parent'):
                            if joint.Parent == old_label:
                                joint.Parent = new_label
                        # Update Child if it matches old Label
                        if hasattr(joint, 'Child'):
                            if joint.Child == old_label:
                                joint.Child = new_label

        if prop == 'Placement':
            if not self.is_execute_ready():
                return
            for fclink in obj.Group:
                if self.get_robot():
                    new_placement = obj.Placement
                else:
                    new_placement = obj.Placement * obj.MountedPlacement
                if (
                    is_freecad_link(fclink)
                    and (fclink.LinkPlacement != new_placement)
                ):
                    fclink.LinkPlacement = new_placement
        if prop == 'MountedPlacement':
            robot = self.get_robot()
            if robot:
                # The placement of FreeCAD links is managed by the robot.
                return
            new_placement = obj.Placement * obj.MountedPlacement
            for fclink in obj.Group:
                if (
                    is_freecad_link(fclink)
                    and (fclink.LinkPlacement != new_placement)
                ):
                    fclink.LinkPlacement = new_placement
        self._set_property_modes()

    def onDocumentRestored(self, obj: CrossLink) -> None:
        self.__init__(obj)
        self._fix_lost_fc_links()
        self._fill_fc_link_lists()

    def dumps(self):
        return self.Type,

    def loads(self, state) -> None:
        if state:
            self.Type, = state
        # Property values are restored through `onChanged` right after `loads`
        # when a document is opened. Those must not be treated as manual user
        # edits, so remember that we are restoring. The flag is cleared in
        # `onDocumentRestored()` (which calls `__init__`).
        self._is_restoring = True

    def _cleanup_children(self) -> DOList:
        """Remove and return all objects not supported by CROSS::Link."""
        if not self.is_execute_ready():
            return []
        removed_objects: set[DO] = set()
        # Group is managed by us and the containing robot.
        for o in self.link.Group:
            if is_freecad_link(o) or is_sensor_link(o) or is_vacuum_gripper(o):
                # Supported, and managed by us.
                continue
            warn_unsupported(o, by='CROSS::Link', gui=True)
            # implementation note: removeobject doesn't raise any exception
            # and `o` exists even if already removed from the group.
            removed_objects.update(self.link.removeObject(o))

        # Clean-up `Real`.
        kept, removed = _skim_links_joints_from(self.link.Real)
        if self.link.Real != kept:
            # Implementation note: the "if" avoids recursion.
            self.link.Real = kept
        warn_unsupported(removed, by='CROSS::Link', gui=True)
        removed_objects.update(removed)

        # Clean-up `Visual.
        kept, removed = _skim_links_joints_from(self.link.Visual)
        if self.link.Visual != kept:
            # Implementation note: the "if" avoids recursion.
            self.link.Visual = kept
        warn_unsupported(removed, by='CROSS::Link', gui=True)
        removed_objects.update(removed)

        # Clean-up `Collision`.
        kept, removed = _skim_links_joints_from(self.link.Collision)
        if self.link.Collision != kept:
            # Implementation note: the "if" avoids recursion.
            self.link.Collision = kept
        warn_unsupported(removed, by='CROSS::Link', gui=True)
        removed_objects.update(removed)

        return list(removed_objects)

    def get_robot(self) -> Optional[CrossRobot]:
        """Return the Cross::Robot this link belongs to."""
        # TODO: as property.
        if (
            hasattr(self, '_robot')
            and self._robot
            and hasattr(self._robot, 'Group')
            and (self.link in self._robot.Group)
        ):
            return self._robot
        if not self.is_execute_ready():
            return None
        for o in self.link.InList:
            if is_robot(o):
                self._robot = cast(CrossRobot, o)
                return self._robot
        return None

    def get_ref_joint(self) -> Optional[CrossJoint]:
        """Return the joint this link is the child of."""
        # TODO: as property.
        robot = self.get_robot()
        if robot is None:
            return None
        if (
            self._ref_joint
            and attr_equals(self._ref_joint, 'Child', ros_name(self.link))
            and hasattr(self._ref_joint, 'Proxy')
            and robot == self._ref_joint.Proxy.get_robot()
        ):
            return self._ref_joint
        joints = get_joints(robot.Group)
        for joint in joints:
            if joint.Child == ros_name(self.link):
                # Parallel mechanisms are not supported, there should be only
                # one joint that has `link` as child.
                self._ref_joint = joint
                return joint
        return None
    

    def get_ref_child_joints(self) -> Optional[list[CrossJoint]]:
        """Return the joint(s) this link is the parent of."""
        # TODO: as property.
        robot = self.get_robot()
        if robot is None:
            return None
        if (
            len(self._ref_child_joints)
            and attr_equals(self._ref_child_joints[0], 'Parent', ros_name(self.link))
            and hasattr(self._ref_child_joints[0], 'Proxy')
            and robot == self._ref_child_joints[0].Proxy.get_robot()
        ):
            return self._ref_child_joints
        joints = get_joints(robot.Group)
        for joint in joints:
            if joint.Parent == ros_name(self.link):
                self._ref_child_joints.append(joint)
        if len(self._ref_child_joints):
            return self._ref_child_joints
        return None
    

    def may_be_base_link(self) -> bool:
        """Return True if the link is child of no joint."""
        return self.get_ref_joint() is None

    def is_tip_link(self) -> bool:
        """Return True if the link is parent of no joint."""
        robot = self.get_robot()
        if robot is None:
            # Not attached to any robot.
            return True
        joints = robot.Proxy.get_joints()
        for joint in joints:
            if joint.Parent == ros_name(self.link):
                return False
        return True

    def is_in_chain_to_joint(self, joint: CrossJoint) -> bool:
        """Return True if `link` is in the chain from base to joint.

        Return True if the link is in the chain from the base link to
        `joint.Parent`.

        """
        if ((not self.is_execute_ready())
                or (not hasattr(joint, 'Proxy'))
                or (not joint.Proxy.is_execute_ready())
                or (not joint.Parent)):
            return False
        robot = joint.Proxy.get_robot()
        if robot is None:
            return False
        parent_link = robot.Proxy.get_link(joint.Parent)
        if parent_link is None:
            return False
        chain = get_chain(parent_link)
        for chain_link in get_links(chain):
            if chain_link is self.link:
                return True
        return False

    def _wrap_manually_added_elements(self, prop: str) -> None:
        """Wrap elements that were added manually to `prop`.

        When a user binds geometry objects to a `Cross::Link` by editing the
        `Real`, `Visual` or `Collision` properties in the FreeCAD property
        editor, each newly added raw geometry object is wrapped the same way
        as in `make_robot_link_filled()`: an `App::Part` wrapper containing an
        `App::Link` to the original object is created and stored into the
        property, and the original object is hidden.

        Wrapping is intentionally not applied when the document is being
        restored, to avoid converting pre-existing data when opening a file.
        Recursion is prevented with `_is_wrapping_elements`. Programmatic
        fills (e.g. `make_robot_links_filled`, URDF/KK import, assembly
        conversion, collision tools) store `App::Part`/`App::Link`-to-part
        wrappers and are therefore kept as-is by `_wrap_robot_link_element()`,
        so they are never wrapped again.

        Note: a user edit in the property editor also runs inside an undo
        transaction, so `doc.Transacting` must not be used to distinguish
        manual edits from programmatic ones.
        """
        if not self.is_execute_ready():
            return
        if not hasattr(self, '_is_wrapping_elements'):
            # Old proxy instances before document reload.
            self._is_wrapping_elements = False
        if not hasattr(self, '_is_restoring'):
            self._is_restoring = False
        if not hasattr(self, '_elements_before_change'):
            self._elements_before_change = {}
        if self._is_wrapping_elements or self._is_restoring:
            return

        link = self.link
        doc = link.Document

        new_values = list(getattr(link, prop, []))
        if not new_values:
            return
        if (not hasattr(link, 'ViewObject')) or (link.ViewObject is None):
            return

        old_values = list(self._elements_before_change.get(prop, []))
        # Wrap only objects that were not there before the change.
        to_wrap = [o for o in new_values if o not in old_values]
        if not to_wrap:
            return

        self._is_wrapping_elements = True
        try:
            for element in to_wrap:
                wrapped = _wrap_robot_link_element(link, element)
                if wrapped is None:
                    continue
                new_values[new_values.index(element)] = wrapped
            if new_values != list(getattr(link, prop, [])):
                setattr(link, prop, new_values)
        finally:
            self._is_wrapping_elements = False

    def update_fc_links(self) -> None:
        """Update the FreeCAD link according to the level of details."""
        # Implementation note: must be public because it is called by the ViewProxy.

        if not self.is_execute_ready():
            return

        link = self.link
        doc = link.Document
        # deactivate update links in undo/redo
        # updating links in undo/redo phase leads to errors
        if doc.Transacting == True:
            return

        if not hasattr(link, 'ViewObject'):
            # No need to change `Group` without GUI.
            return
        vlink = link.ViewObject
        if vlink is None:
            return

        links_real = get_sorted_concated_names(self._fc_links_real)
        reals = get_sorted_concated_names(link.Real)
        links_visual = get_sorted_concated_names(self._fc_links_visual)
        visuals = get_sorted_concated_names(link.Visual)
        links_collision = get_sorted_concated_names(self._fc_links_collision)
        collision = get_sorted_concated_names(link.Collision)

        # compare created links and their source objects
        update_real = links_real != reals
        update_visual = links_visual != visuals
        update_collision = links_collision != collision

        # Old objects that will be removed after having been excluded from
        # `Group`, to avoid recursive calls.
        old_fc_links: DOList = []
        if update_real or not vlink.ShowReal:
            old_fc_links += self._fc_links_real
        if update_visual or not vlink.ShowVisual:
            old_fc_links += self._fc_links_visual
        if update_collision or not vlink.ShowCollision:
            old_fc_links += self._fc_links_collision

        # remove links
        for o in old_fc_links:
            try:
                # Free the labels in case something wrong with removal.
                o.Label = 'to_be_removed'
                o.Visibility = False
                doc.removeObject(o.Name)
            except (ReferenceError, AttributeError, TypeError):
                pass

        # Clear the lists that are regenerated right after and create new
        # objects.
        if update_real or not vlink.ShowReal:
            self._fc_links_real.clear()
        if update_visual or not vlink.ShowVisual:
            self._fc_links_visual.clear()
        if update_collision or not vlink.ShowCollision:
            self._fc_links_collision.clear()

        # Create new objects.
        if update_real and vlink.ShowReal:
            self._fc_links_real = _add_fc_links_lod(link, link.Real, 'real')
        if update_visual and vlink.ShowVisual:
            self._fc_links_visual = _add_fc_links_lod(
                    link, link.Visual, 'visual',
            )
        if update_collision and vlink.ShowCollision:
            self._fc_links_collision = _add_fc_links_lod(
                    link, link.Collision, 'collision',
            )

        # Reset the group.
        new_group = (
            self._fc_links_real
            + self._fc_links_visual
            + self._fc_links_collision
            + self.get_sensors()
            + self.get_vacuum_grippers()
        )
        if new_group != link.Group:
            link.Group = new_group


    def export_urdf(
        self,
        package_parent: Path,
        package_name: [Path | str],
    ) -> et.ElementTree:
        """Return the xml for this link.

        Parameters
        ----------
        - package_parent: the parent directory of the package where the URDF
                          will be saved.
        - package_name: the name of the exported package (also the name of the
                        directory).

        """

        link_xml = et.fromstring(
            f'<link name="{get_valid_urdf_name(ros_name(self.link))}" />',
        )
        for obj in self.link.Visual:
            for xml in _get_xmls_and_export_meshes(
                    obj,
                    urdf_visual_from_object,
                    self.link.MountedPlacement,
                    package_parent,
                    package_name,
            ):
                link_xml.append(xml)
        for obj in self.link.Collision:
            for xml in _get_xmls_and_export_meshes(
                    obj,
                    urdf_collision_from_object,
                    self.link.MountedPlacement,
                    package_parent,
                    package_name,
            ):
                link_xml.append(xml)
        # link with zero mass and inertia can leads to error ("pose must be finite") in Gazebo
        if self.link.Mass.Value > 0:
            link_xml.append(
                urdf_inertial(
                    mass=self.link.Mass.Value,
                    center_of_mass=self.link.CenterOfMass,
                    ixx=self.link.Ixx,
                    ixy=self.link.Ixy,
                    ixz=self.link.Ixz,
                    iyy=self.link.Iyy,
                    iyz=self.link.Iyz,
                    izz=self.link.Izz,
                ),
            )
        return link_xml

    def _fix_lost_fc_links(self) -> None:
        """Fix linked objects in CROSS links lost on restore.

        Probably because these elements are restored before the CROSS links.

        """
        if not self.is_execute_ready():
            return
        link = self.link
        for obj in link.Document.Objects:
            if (not hasattr(obj, 'InList')) or (len(obj.InList) != 1):
                continue
            potential_self = obj.InList[0]
            if ((obj is link)
                    or (potential_self is not link)
                    or (obj in link.Group)
                    or (obj in link.Real)
                    or (obj in link.Visual)
                    or (obj in link.Collision)):
                continue
            link.addObject(obj)

    def _fill_fc_link_lists(self) -> None:
        """Fill the lists of FreeCAD links.

        The lists `_fc_links_real` and similar are empty on document restore
        and need to be filled up.

        Must be called after `_fix_lost_fc_links`.
        Not very elegant but the lists cannot be serialized easily.

        """
        if not self.is_execute_ready():
            return
        for o in self.link.Group:
            if o.Label.startswith('real'):
                self._fc_links_real.append(o)
            elif o.Label.startswith('visual'):
                self._fc_links_visual.append(o)
            elif o.Label.startswith('collision'):
                self._fc_links_collision.append(o)

    def _set_property_modes(self) -> None:
        """Set the modes of the properties."""
        if not self.is_execute_ready():
            return
        if self.get_robot():
            # Placement is managed by the robot.
            self.link.setEditorMode('Placement', ['ReadOnly'])
        else:
            self.link.setEditorMode('Placement', [])

    def get_sensors(self) -> list[CrossSensor]:
        """Return the list of CROSS sensors in the order of creation."""
        # TODO: as property.
        if self._sensors is not None:
            # self._sensors is updated in self.onChanged().
            return list(self._sensors)  # A copy.
        if not self.is_execute_ready():
            return []
        self._sensors = get_link_sensors(self.link.Group)
        return list(self._sensors)  # A copy.

    def get_vacuum_grippers(self) -> list[CrossVacuumGripper]:
        """Return the list of CROSS vacuum grippers in the order of creation."""
        if self._vacuum_grippers is not None:
            return list(self._vacuum_grippers)
        if not self.is_execute_ready():
            return []
        self._vacuum_grippers = get_vacuum_grippers(self.link.Group)
        return list(self._vacuum_grippers)


class _ViewProviderLink(ProxyBase):
    """A view provider for the Cross::Link object."""

    def __init__(self, vobj: VPDO):
        super().__init__(
            'view_object',
            [
                'Visibility',
            ],
        )
        if vobj.Proxy is not self:
            # Implementation note: triggers `self.attach`.
            vobj.Proxy = self
        self._init(vobj)

    def _init(self, vobj: VPDO) -> None:
        self.view_object = vobj
        self.link = vobj.Object
        self._init_extensions(vobj)
        self._init_properties(vobj)

    def getIcon(self):
        # Implementation note: "return 'link.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'link.svg')

    def attach(self, vobj: VPDO):
        # `self.__init__()` is not called on document restore, do it manually.
        self.__init__(vobj)

    def _init_extensions(self, vobj: VPDO):
        vobj.addExtension('Gui::ViewProviderGroupExtensionPython')

    def _init_properties(self, vobj: VPDO):
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

        vobj.ShowCollision = True
        vobj.ShowReal = True

        self._old_show_real = vobj.ShowReal
        self._old_show_visual = vobj.ShowVisual
        self._old_show_collision = vobj.ShowCollision

    def updateData(self, obj: CrossLink, prop):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        if prop in ['ShowReal', 'ShowVisual', 'ShowCollision']:
            vobj.Object.Proxy.update_fc_links()
        if prop == 'Visibility':
            for o in vobj.Object.Group:
                o.ViewObject.Visibility = vobj.Visibility

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
        return

    def dumps(self):
        return None

    def loads(self, state) -> None:
        pass


def make_link(name, doc: Optional[fc.Document] = None, recompute_after: bool = True) -> CrossLink:
    """Add a Cross::Link to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document and cannot create a new document, doing nothing', True)
        return
    cross_link: CrossLink = doc.addObject('App::FeaturePython', name)
    LinkProxy(cross_link)
    cross_link.Label2 = name

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderLink(cross_link.ViewObject)

        # Make `obj` part of the selected `Cross::Robot`.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if is_robot(candidate):
                cross_link.adjustRelativeLinks(candidate)
                candidate.addObject(cross_link)
                if candidate.ViewObject:
                    cross_link.ViewObject.ShowReal = candidate.ViewObject.ShowReal
                    cross_link.ViewObject.ShowVisual = candidate.ViewObject.ShowVisual
                    cross_link.ViewObject.ShowCollision = candidate.ViewObject.ShowCollision
            elif is_joint(candidate):
                robot = candidate.Proxy.get_robot()
                if robot:
                    cross_link.adjustRelativeLinks(robot)
                    robot.addObject(cross_link)
                    link_name = ros_name(cross_link)
                    if link_name in candidate.getEnumerationsOfProperty('Child'):
                        candidate.Child = ros_name(cross_link)
                    if robot.ViewObject:
                        cross_link.ViewObject.ShowReal = robot.ViewObject.ShowReal
                        cross_link.ViewObject.ShowVisual = robot.ViewObject.ShowVisual
                        cross_link.ViewObject.ShowCollision = robot.ViewObject.ShowCollision
    if recompute_after:
        doc.recompute()
    return cross_link


def _get_robot_parts_container(doc: fc.Document, name: str) -> DO:
    """Return the hidden group container `name`, creating it if needed."""
    container = doc.getObject(name)
    if not container:
        container = add_object(doc, 'App::DocumentObjectGroup', name)
        container.Visibility = False
    return container


def _store_robot_link_element_part(
        part: DO,
        doc: Optional[fc.Document] = None,
) -> None:
    """Hide the wrapper `part` and add it to the `robot_parts` container.

    Parameters
    ----------
    - part: the wrapper `App::Part` to store.
    - doc: the document that contains the `robot_parts` container. Defaults to
           `part.Document`.

    """
    if doc is None:
        doc = part.Document
    container = _get_robot_parts_container(doc, 'robot_parts')
    part.Visibility = False
    container.addObject(part)


def _store_robot_link_element_origin(
        element: DO,
        doc: Optional[fc.Document] = None,
) -> None:
    """Hide the original `element` and add it to `robot_parts_origins`.

    An element living in another document than `doc` cannot be moved into a
    group of `doc`; in that case it is only hidden.

    Parameters
    ----------
    - element: the original object that was bound to a link.
    - doc: the document that contains the `robot_parts_origins` container.
           Defaults to `element.Document`.

    """
    if doc is None:
        doc = element.Document
    if hasattr(element, 'Visibility'):
        try:
            element.Visibility = False
        except Exception:
            pass
    if element.Document is not doc:
        # Cannot be moved into a group of another document.
        return
    container = _get_robot_parts_container(doc, 'robot_parts_origins')
    container.addObject(element)


def _is_wrappable_object(obj: DO) -> bool:
    """Return True if `obj` carries geometry usable for a link element.

    An `App::GeoFeature` has geometry directly. An `App::Link` is wrappable
    when its (recursively resolved) linked object is a geometry feature; the
    link itself is then wrapped, preserving its placement and the external
    reference.

    Note: `App::Part` containers and `App::Link` to an `App::Part` are also
    derived from `App::GeoFeature`/carry geometry; whether they are wrapped or
    kept as-is is decided by `_resolve_wrappable_target()`, not here.

    """
    if is_derived_from(obj, 'App::GeoFeature'):
        return True
    if is_freecad_link(obj):
        # `getLinkedObject(True)` resolves the whole link chain, so no
        # recursion is needed here.
        try:
            linked = obj.getLinkedObject(True)
        except ReferenceError:
            return False
        if (linked is None) or (linked is obj):
            return False
        return is_derived_from(linked, 'App::GeoFeature')
    return False


def _make_robot_link_element_wrapper(
        obj: fc.DO,
        doc: Optional[fc.Document] = None,
) -> DO | False:
    """Create an `App::Part` wrapper with an `App::Link` to `obj`.

    The wrapper is the object stored in the `Real`, `Visual` or `Collision`
    property of a `Cross::Link`. The original `obj` is not modified.

    Parameters
    ----------
    - obj: the geometry object to wrap. It may live in another document than
           `doc` (e.g. an external document referenced through an `App::Link`).
    - doc: the document in which the wrapper must be created. Defaults to the
           document of `obj`, like `make_robot_link_filled()` did historically.

    Return the created wrapper part, or `False` if `obj` is not a geometry
    object that can be wrapped.

    """
    if not _is_wrappable_object(obj):
        return False

    if doc is None:
        doc = obj.Document
    part = add_object(doc, 'App::Part', ros_name(obj))
    fc_link_to_obj = add_object(doc, 'App::Link', ros_name(obj))
    fc_link_to_obj.LinkedObject = obj
    fc_link_to_obj.adjustRelativeLinks(part)
    part.addObject(fc_link_to_obj)
    return part


def _get_link_wrapper_for_object(element: DO) -> DO | None:
    """Return the existing `App::Part` wrapper of `element`, if any.

    A wrapper is an `App::Part` that contains exactly one `App::Link` to
    `element`. Such a wrapper is created by `make_robot_link_filled()` and by
    `_wrap_robot_link_element()`.

    """
    for ref in getattr(element, 'InList', []):
        if not is_freecad_link(ref):
            continue
        try:
            if ref.getLinkedObject(True) is not element:
                continue
        except ReferenceError:
            continue
        # `ref` is an `App::Link` to `element`. Find the `App::Part` that
        # contains only this link (our wrapper).
        for parent in getattr(ref, 'InList', []):
            if not is_part(parent):
                continue
            children = list(getattr(parent, 'Group', []))
            if len(children) == 1 and children[0] is ref:
                return parent
    return None


def _find_or_create_link_element_wrapper(
        real_object: DO,
        doc: Optional[fc.Document] = None,
) -> tuple[DO | None, bool]:
    """Return the wrapper `App::Part` of `real_object`, reusing it or creating.

    The wrapper is looked up first with `_get_link_wrapper_for_object()`, so a
    body that was already wrapped (by `make_robot_link_filled()` or by
    `_wrap_robot_link_element()`) is never wrapped a second time. This lookup
    is shared by both callers to avoid duplicating the "find existing wrapper"
    logic.

    Parameters
    ----------
    - real_object: the geometry object (or `App::Link`) that the wrapper
                   `App::Part` must link to.
    - doc: the document in which a new wrapper must be created. Defaults to the
           document of `real_object`.

    Return a tuple `(wrapper, created)`:
    - `wrapper`: the existing or new `App::Part`, or `None` if `real_object`
                 is not a geometry object that can be wrapped.
    - `created`: `True` if a new wrapper was created, `False` if an existing
                 one was reused or if wrapping is not possible.

    """
    existing_wrapper = _get_link_wrapper_for_object(real_object)
    if existing_wrapper is not None:
        return existing_wrapper, False
    part = _make_robot_link_element_wrapper(real_object, doc)
    if not part:
        return None, False
    return part, True


def _resolve_wrappable_target(element: DO) -> tuple[DO, DO] | None:
    """Resolve `element` to (real_object, object_to_hide).

    `real_object` is what the wrapper `App::Part` will link to. For an
    `App::Link` that does not point to an `App::Part`, the wrapper keeps the
    `App::Link` itself so that its placement and the linked object are
    preserved, exactly like `make_robot_link_filled()` does for a selected
    object.

    Return None when the element is not wrappable (already a wrapper
    container, an `App::Link` to an `App::Part`, a `Cross::*` object, or not
    a geometry object).

    """
    if is_link(element) or is_joint(element) or is_sensor_link(element):
        # CROSS objects must not be wrapped.
        return None
    if is_part(element):
        # `App::Part` containers (wrappers and parts stored by the
        # programmatic flows like URDF/KK import) are kept as-is.
        return None
    if is_freecad_link(element):
        # An `App::Link` to an `App::Part` is the internal link of a wrapper
        # (or a reference to an already managed part): keep as-is.
        linked = element.getLinkedObject(True)
        if is_part(linked):
            return None
        # Any other `App::Link` must be wrapped too: it carries geometry
        # through its `LinkedObject` (resolved recursively), so the wrapper
        # keeps the `App::Link` itself, preserving its placement and the
        # reference to the (possibly external) linked object.
        if not _is_wrappable_object(element):
            return None
        return element, element
    if not _is_wrappable_object(element):
        return None
    return element, element


def _wrap_robot_link_element(link: CrossLink, element: DO) -> DO | None:
    """Wrap a raw geometry element manually added to `link` properties.

    This is the same mechanism as in `make_robot_link_filled()`: the element
    is wrapped into an `App::Part` containing an `App::Link` to it, the
    wrapper is hidden and stored into the `robot_parts` container, and the
    original element is hidden and moved to `robot_parts_origins`.

    Return the wrapper part, or `None` if the element should be kept as-is
    (already a wrapper, an `App::Part`, an `App::Link` to a part, a
    `Cross::*` object, or not a geometry object).

    """
    resolved = _resolve_wrappable_target(element)
    if resolved is None:
        return None
    real_object, object_to_hide = resolved

    # Reuse the existing wrapper of the geometry if it was already wrapped
    # (e.g. when bound to another link), otherwise create a new one. The
    # lookup is shared with `make_robot_link_filled()`.
    doc = link.Document
    part, created = _find_or_create_link_element_wrapper(real_object, doc)
    if part is None:
        return None

    if created:
        # Only a newly created wrapper must be stored (hidden into
        # `robot_parts`) and its original element hidden (moved into
        # `robot_parts_origins`), the same way as
        # `make_robot_link_filled()` with `create_parts_group=True` and
        # `make_robot_links_filled()` do. An existing wrapper was already
        # stored and its original element already hidden when it was first
        # created.
        _store_robot_link_element_part(part, doc)
        _store_robot_link_element_origin(object_to_hide, doc)

    return part


def make_robot_link_filled(obj:fc.DO, create_parts_group:bool = False, assembly_reference:str = '') -> CrossLink | False :
    ''' Make robot link and fill Real and Visual of it by selected objects  '''

    # The wrapper must be created in the active document, like the CROSS::Link
    # itself. `obj` may live in another (external) document, e.g. an object
    # linked from an external assembly document.
    doc = fc.ActiveDocument
    # Reuse the existing wrapper of `obj` if it was already wrapped (e.g. the
    # body was bound to another link before), like `_wrap_robot_link_element()`
    # does for manually bound elements. The find-or-create logic is shared and
    # must not be duplicated here.
    part, _created = _find_or_create_link_element_wrapper(obj, doc)
    if not part:
        message(
            f'Not suited object ({ros_name(obj)}) to create robot link.',
            True,
        )
        return False

    link = make_link('l_' + ros_name(part))
    link.Real = part
    link.Visual = part
    if assembly_reference:
        link.AssemblyReference = assembly_reference
    link.ViewObject.ShowReal = False
    link.ViewObject.ShowReal = True

    if create_parts_group:
        _store_robot_link_element_part(part, doc)

    doc.recompute()

    return link


def make_robot_links_filled(objects:list[fc.DO] = [], robot:CrossRobot | None = None, create_parts_group:bool = True) -> list[CrossLink] | False :
    ''' Make robot links and fill Real and Visual of it by selected objects  '''

    if len(objects):
        selection = objects
    else:
        selection = fcgui.Selection.getSelection()

    links:list[CrossLink] = []
    for el in selection:
        res = make_robot_link_filled(el, create_parts_group)
        if is_link(res):
            link = res
            links.append(link)
            el.Visibility = False
            
            if robot:
                link.adjustRelativeLinks(robot)
                robot.addObject(link)

            if create_parts_group:
                _store_robot_link_element_origin(el, fc.ActiveDocument)
    return links


def explode_link(orienteer: fc.DO, offset: float) -> bool:
    ''' Move link for see hiden faces (explode view)  '''

    if not is_link(orienteer):
        link = get_parent_link_of_obj(orienteer)
    else:
        link = orienteer

    if not link:
        # there is no parent link
        return False

    link.MountedPlacement.Base.x = link.MountedPlacement.Base.x + offset
    
    # link.ViewObject.ShowReal = False
    # link.ViewObject.ShowReal = True

    return True
