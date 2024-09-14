from __future__ import annotations

from pathlib import Path
from typing import ForwardRef, List, Optional, cast
import xml.etree.ElementTree as et

import FreeCAD as fc

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import is_link as is_freecad_link
from .freecad_utils import warn
from .freecad_utils import add_object
from .freecad_utils import is_part
from .mesh_utils import save_mesh_dae
from .urdf_utils import XmlForExport
from .urdf_utils import urdf_collision_from_object
from .urdf_utils import urdf_inertial
from .urdf_utils import urdf_visual_from_object
from .utils import attr_equals
from .utils import warn_unsupported
from .wb_utils import ICON_PATH
from .wb_utils import get_chain
from .wb_utils import get_joints
from .wb_utils import get_links
from .wb_utils import get_valid_urdf_name
from .wb_utils import is_joint
from .wb_utils import is_link
from .wb_utils import is_name_used
from .wb_utils import is_primitive
from .wb_utils import is_robot
from .wb_utils import ros_name

# Stubs and typing hints.
from .joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
DO = fc.DocumentObject
DOList = List[DO]
VPDO = ForwardRef('FreeCADGui.ViewProviderDocumentObject')  # Don't want to import FreeCADGui here. # noqa: E501
AppLink = DO  # TypeId == 'App::Link'.


def _add_fc_links_lod(
        link: CrossLink,
        objects: DOList,
        lod: str,
) -> list[AppLink]:
    """Create links to real, visual or collision elements.

    Return the list of created FreeCAD link objects.
    The objects are not added to the link.

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
                '_Type',
            ],
        )
        obj.Proxy = self
        self.link = obj

        # Lists to keep track of the objects that were added to the link.
        self._fc_links_real: DOList = []
        self._fc_links_visual: DOList = []
        self._fc_links_collision: DOList = []

        # Used to recover a valid and unique name on change of `Label` or
        # `Label2`.
        # Updated in `onBeforeChange` and potentially used in `onChanged`.
        self.old_ros_name: str = ''

        # Save the robot to speed-up self.get_robot().
        self._robot: Optional[CrossRobot] = None

        # Save the parent joint to speed-up self.get_ref_joint().
        self._ref_joint: Optional[CrossJoint] = None

        self.init_extensions(obj)
        self.init_properties(obj)

    def init_extensions(self, obj: CrossLink) -> None:
        # Need a group to put the generated FreeCAD links in.
        obj.addExtension('App::GroupExtensionPython')

    def init_properties(self, obj: CrossLink):
        add_property(
            obj, 'App::PropertyString', '_Type', 'Internal',
            'The type',
        )
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(
            obj, 'App::PropertyLinkList', 'Real', 'Elements',
            'The real part objects of this link, optional',
        )
        add_property(
            obj, 'App::PropertyLinkList', 'Visual', 'Elements',
            'The part objects this link that constitutes the URDF'
            ' visual elements, optional',
        )
        add_property(
            obj, 'App::PropertyLinkList', 'Collision', 'Elements',
            'The part objects this link that constitutes the URDF'
            ' collision elements, optional',
        )

        add_property(
            obj, 'App::PropertyQuantity', 'Mass', 'Inertial Parameters',
            'Mass of the link',
        )
        obj.Mass = fc.Units.Mass
        add_property(obj, 'App::PropertyPlacement', 'CenterOfMass', 'Inertial Parameters',
                     'Center of mass of the link, with orientation determining the principal axes of inertia')
        add_property(obj, 'App::PropertyPlacement', 'CenterOfMassGlobalCoords', 'Inertial Parameters',
                     'Center of mass of the link, with orientation determining the principal axes of inertia \
                        in global coordinates.')
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
            'Density of material. Used to calculate mass. May be outdated if you updated the material density outside CROSS workbench. Actual density will taken from material (material editor) at mass calculation moment.',
        )
        obj.setPropertyStatus('MaterialDensity', ['ReadOnly'])
        add_property(
            obj, 'App::PropertyBool', 'MaterialNotCalculate', 'Material',
            'If true this material will be not used to calculate mass and inertia of element. In this case you can use manually filled mass and inertia for some elements and auto calculation for others.',
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
        if prop in ['Label', 'Label2']:
            robot = self.get_robot()
            if (robot and is_name_used(obj, robot)):
                self.old_ros_name = ''
            else:
                self.old_ros_name = ros_name(obj)

    def onChanged(self, obj: CrossLink, prop: str) -> None:
        if prop == 'Group':
            self.cleanup_children()
        if prop in ('Real', 'Visual', 'Collision'):
            self.cleanup_children()
            self.update_fc_links()
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

    def cleanup_children(self) -> DOList:
        """Remove and return all objects not supported by CROSS::Link."""
        if not self.is_execute_ready():
            return []
        removed_objects: set[DO] = set()
        # Group is managed by us and the containing robot.
        for o in self.link.Group:
            if is_freecad_link(o):
                # Supported, and managed by us.
                continue
            warn_unsupported(o, by='CROSS::Link', gui=True)
            # implementation note: removeobject doesn't raise any exception
            # and `o` exists even if already removed from the group.
            removed_objects.update(self.link.removeObject(o))

        # Clean-up `Real`.
        kept, removed = _skim_links_joints_from(self.link.Real)
        if self.link.Real != kept:
            self.link.Real = kept
        warn_unsupported(removed, by='CROSS::Link', gui=True)
        removed_objects.update(removed)

        # Clean-up `Visual.
        kept, removed = _skim_links_joints_from(self.link.Visual)
        if self.link.Visual != kept:
            self.link.Visual = kept
        warn_unsupported(removed, by='CROSS::Link', gui=True)
        removed_objects.update(removed)

        # Clean-up `Collision`.
        kept, removed = _skim_links_joints_from(self.link.Collision)
        if self.link.Collision != kept:
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

    def update_fc_links(self) -> None:
        """Update the FreeCAD link according to the level of details."""
        if not self.is_execute_ready():
            return
        link = self.link
        if not hasattr(link, 'ViewObject'):
            # No need to change `Group` without GUI.
            return
        vlink = link.ViewObject
        if vlink is None:
            return

        old_show_real = len(self._fc_links_real) != 0
        old_show_visual = len(self._fc_links_visual) != 0
        old_show_collision = len(self._fc_links_collision) != 0
        update_real = old_show_real != vlink.ShowReal
        update_visual = old_show_visual != vlink.ShowVisual
        update_collision = old_show_collision != vlink.ShowCollision

        # Old objects that will be removed after having been excluded from
        # `Group`, to avoid recursive calls.
        old_fc_links: DOList = []
        if update_real:
            old_fc_links += self._fc_links_real
        if update_visual:
            old_fc_links += self._fc_links_visual
        if update_collision:
            old_fc_links += self._fc_links_collision

        # Free the labels because of delayed removal.
        for o in old_fc_links:
            # Free the label.
            o.Label = 'to_be_removed'

        # Clear the lists that are regenerated right after and create new
        # objects.
        if update_real:
            self._fc_links_real.clear()
        if update_visual:
            self._fc_links_visual.clear()
        if update_collision:
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
        )
        if new_group != link.Group:
            link.Group = new_group

        # Remove old objects.
        doc = link.Document
        for o in old_fc_links:
            doc.removeObject(o.Name)

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


class _ViewProviderLink(ProxyBase):
    """A view provider for the Cross::Link object """

    def __init__(self, vobj: VPDO):
        super().__init__(
            'view_object',
            [
                'Visibility',
            ],
        )
        vobj.Proxy = self

    def getIcon(self):
        # Implementation note: "return 'link.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'link.svg')

    def attach(self, vobj: VPDO):
        self.view_object = vobj
        self.init_extensions(vobj)
        self.init_properties(vobj)

    def init_extensions(self, vobj: VPDO):
        vobj.addExtension('Gui::ViewProviderGroupExtensionPython')

    def init_properties(self, vobj: VPDO):
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

        self._old_show_real = vobj.ShowReal
        self._old_show_visual = vobj.ShowVisual
        self._old_show_collision = vobj.ShowCollision

    def updateData(self, obj: VPDO, prop):
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


def make_link(name, doc: Optional[fc.Document] = None) -> CrossLink:
    """Add a Cross::Link to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    cross_link: CrossLink = doc.addObject('App::FeaturePython', name)
    LinkProxy(cross_link)
    cross_link.Label2 = cross_link.Label

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
    doc.recompute()
    return cross_link


def make_robot_link_filled(obj:fc.DO) -> CrossLink :
    ''' Make robot link and fill Real and Visual of it by obj param  '''

    if not is_part(obj):
        part = add_object(fc.ActiveDocument, 'App::Part', obj.Label + '_part')
        
        parent_of_obj = None
        try:
            parent_of_obj = obj.Parents[0][0]
        except (KeyError, AttributeError):
            pass

        part.addObject(obj)
        
        #add created part-wrapper as child to parent of object 
        if parent_of_obj:
            parent_of_obj.addObject(part)
    else:
        part = obj

    ros_link = make_link(obj.Label)
    ros_link.Real = part
    ros_link.Visual = part

    return ros_link
