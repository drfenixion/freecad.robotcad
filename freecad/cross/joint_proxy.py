from __future__ import annotations

from math import degrees, radians
from typing import Optional, Union, cast
import xml.etree.ElementTree as et
from typing import NewType, List, Optional, cast

import FreeCAD as fc
import FreeCADGui as fcgui

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import warn
from .freecad_utils import message
from .urdf_utils import urdf_origin_from_placement
from .wb_utils import ICON_PATH, get_joint_sensors
from .wb_utils import get_valid_urdf_name
from .wb_utils import is_link
from .wb_utils import is_name_used
from .wb_utils import is_robot
from .wb_utils import is_workcell
from .wb_utils import is_sensor_joint
from .wb_utils import ros_name
from .utils import warn_unsupported
from .sdf import setup,export
# Stubs and typing hints.
from .joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
from .joint import ViewProviderJoint as VP
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
from .sensors.sensor import Sensor as CrossSensor  # A Cross::Sensor, i.e. a DocumentObject with Proxy "SensorProxyJoint" or "SensorProxyJoint". # noqa: E501

DO = fc.DocumentObject
DOList = List[DO]


class JointProxy(ProxyBase):
    """The proxy for a Cross::Joint object."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::Joint'

    # The names cannot be changed because they are used as-is in the generated
    # URDF. The order can be changed and influences the order in the GUI.
    # The first element is the default.
    type_enum = ['fixed', 'prismatic', 'revolute', 'continuous', 'planar', 'floating']

    def __init__(self, obj: CrossJoint):
        super().__init__(
            'joint',
            [
                'Group',
                'Child',
                'Effort',
                'LowerLimit',
                'Mimic',
                'MimickedJoint',
                'Multiplier',
                'Offset',
                'Origin',
                'Parent',
                'Placement',
                'PlacementRelTotalCenterOfMass',
                'Position',
                'Type',
                'UpperLimit',
                'Velocity',
                'JointSpecific',
                'JoinRotationDirection',
                '_Type',
            ],
        )
        obj.Proxy = self
        self.joint = obj

        # Updated in onChanged().
        self.child_link: Optional[CrossLink] = None
        self.parent_link: Optional[CrossLink] = None

        # Used to recover a valid and unique name on change of `Label` or
        # `Label2`.
        # Updated in `onBeforeChange` and potentially used in `onChanged`.
        self.old_ros_name: str = ''

        # Save the robot to speed-up self.get_robot().
        self._robot: Optional[CrossRobot] = None

        self._sensors: Optional[list[CrossSensor]] = None

        self.init_extensions(obj)
        self.init_properties(obj)
        setup.joint_properties(self.joint,"joint",True,fcgui.getMainWindow())

    def init_extensions(self, obj: CrossJoint) -> None:
        # Need a group to put the generated FreeCAD links in.
        obj.addExtension('App::GroupExtensionPython')

    def init_properties(self, obj: CrossJoint):
        add_property(
            obj, 'App::PropertyString', '_Type', 'Internal',
            'The type of object',
        )
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(
            obj, 'App::PropertyEnumeration', 'Type', 'Elements',
            'The kinematical type of the joint',
        )
        obj.Type = JointProxy.type_enum
        add_property(
            obj, 'App::PropertyEnumeration', 'Parent', 'Elements',
            'Parent link (from RobotCAD)',
        )
        add_property(
            obj, 'App::PropertyEnumeration', 'Child', 'Elements',
            'Child link (from RobotCAD)',
        )
        add_property(
            obj, 'App::PropertyPlacement', 'Origin', 'Elements',
            'Joint origin relative to the parent link',
        )
        add_property(
            obj, 'App::PropertyFloat', 'LowerLimit', 'Limits',
            'Lower position limit (mm or deg)',
        )
        add_property(
            obj, 'App::PropertyFloat', 'UpperLimit', 'Limits',
            'Upper position limit (mm or deg)',
        )
        add_property(
            obj, 'App::PropertyFloat', 'Effort', 'Limits',
            'Maximal effort (N or Nm)',
        )
        add_property(
            obj, 'App::PropertyFloat', 'Velocity', 'Limits',
            'Maximal velocity (mm/s or deg/s)',
        )
        add_property(
            obj, 'App::PropertyFloat', 'Position', 'Value',
            'Joint position (m or rad)',
        )
        obj.setEditorMode('Position', ['ReadOnly'])

        # Mimic joint.
        add_property(
            obj, 'App::PropertyBool', 'Mimic', 'Mimic',
            'Whether this joint mimics another one',
        )
        add_property(
            obj, 'App::PropertyLink', 'MimickedJoint', 'Mimic',
            'Joint to mimic',
        )
        add_property(
            obj, 'App::PropertyFloat', 'Multiplier', 'Mimic',
            'value = Multiplier * other_joint_value + Offset', 1.0,
        )
        add_property(
            obj, 'App::PropertyFloat', 'Offset', 'Mimic',
            'value = Multiplier * other_joint_value + Offset, in mm or deg',
        )

        add_property(
            obj, 'App::PropertyEnumeration', 'JoinRotationDirection', 'Robot',
            'Propellers rotation direction. Can be used for code generation',
        )
        obj.JoinRotationDirection=["unset","cw","ccw"]
        obj.setPropertyStatus('JoinRotationDirection', ['Hidden'])
        add_property(
            obj, 'App::PropertyEnumeration', 'JointSpecific', 'Robot',
            'Specific of joint working. Can be used for code generation',
        )
        obj.JointSpecific=["unset","propeller"]

        add_property(
            obj, 'App::PropertyPlacement', 'Placement', 'Internal',
            'Placement of the joint in the robot frame',
        )
        obj.setEditorMode('Placement', ['ReadOnly'])
        add_property(
            obj, 'App::PropertyPlacement', 'PlacementRelTotalCenterOfMass', 'Internal',
            'Placement of the joint in the center of mass frame of robot',
        )
        obj.setEditorMode('Placement', ['ReadOnly'])

        self._toggle_editor_mode()

    def onBeforeChange(self, obj: CrossLink, prop: str) -> None:
        """Called before a property of `obj` is changed."""
        # TODO: save the old ros_name and update all joints that used it.
        if prop in ['Label', 'Label2']:
            robot = self.get_robot()
            if (robot and is_name_used(obj, robot)):
                self.old_ros_name = ''
            else:
                self.old_ros_name = ros_name(obj)

    def onChanged(self, obj: CrossJoint, prop: str) -> None:
        """Called when a property has changed."""
        # print(f'{obj.Label}.onChanged({prop})') # DEBUG
        if prop == 'Group':
            self._sensors = None
            self._cleanup_children()
        if prop == 'Mimic':
            self._toggle_editor_mode()
        if prop == 'MimickedJoint':
            if ((obj.MimickedJoint is not None)
                    and (obj.Type != obj.MimickedJoint.Type)):
                warn(
                    'Mimicked joint must have the same type'
                    f' but "{obj.Label}"\'s type is {obj.Type} and'
                    f' "{obj.MimickedJoint}"\'s is {obj.MimickedJoint.Type}',
                    True,
                )
                obj.MimickedJoint = None
        if prop in ('Label', 'Label2'):
            robot = self.get_robot()
            if robot and hasattr(robot, 'Proxy'):
                robot.Proxy.add_joint_variables()
            if (
                robot
                and is_name_used(obj, robot)
                and getattr(obj, prop) != self.old_ros_name
            ):
                setattr(obj, prop, self.old_ros_name)
        if prop == 'Type':
            self._toggle_editor_mode()
        if prop == 'Child':
            if obj.Child:
                # No need to update if the link name is still in the enum after
                # the link name changed. However, we need to save it for when
                # the child name will be changed, which provokes
                # `obj.Child = ''`.
                robot = self.get_robot()
                if ((robot is None)
                        or (not hasattr(robot, 'Proxy'))):
                    self.child_link = None
                    return
                self.child_link = robot.Proxy.get_link(obj.Child)
            new_link_name = ros_name(self.child_link) if self.child_link else obj.Child
            if ((
                self.child_link
                and (new_link_name in obj.getEnumerationsOfProperty('Child'))
            )
                    and (obj.Child != new_link_name)):
                obj.Child = new_link_name
        if prop == 'Parent':
            if obj.Parent:
                # No need to update if the link name is still in the enum after
                # the link name changed. However, we need to save it for when
                # the parent name will be changed, which provokes
                # `obj.Parent = ''`.
                robot = self.get_robot()
                if ((robot is None)
                        or (not hasattr(robot, 'Proxy'))):
                    self.parent_link = None
                    return
                self.parent_link = robot.Proxy.get_link(obj.Parent)
            new_link_name = ros_name(self.parent_link) if self.parent_link else obj.Parent
            if ((
                self.parent_link
                and (new_link_name in obj.getEnumerationsOfProperty('Parent'))
            )
                    and (obj.Parent != new_link_name)):
                obj.Parent = new_link_name
        if prop == 'JointSpecific':
            if obj.JointSpecific != 'unset':
                obj.setPropertyStatus('JoinRotationDirection', '-Hidden')
            else:
                obj.setPropertyStatus('JoinRotationDirection', 'Hidden')

    def onDocumentRestored(self, obj: CrossJoint):
        self.__init__(obj)
        self._fix_lost_fc_links()

    def _fix_lost_fc_links(self) -> None:
        """Fix linked objects in CROSS joints lost on restore.

        Probably because these elements are restored before the CROSS links.

        """
        if not self.is_execute_ready():
            return
        elem = self.joint
        for obj in elem.Document.Objects:
            if (not hasattr(obj, 'InList')) or (len(obj.InList) != 1):
                continue
            potential_self = obj.InList[0]
            if ((obj is elem)
                    or (potential_self is not elem)
                    or (obj in elem.Group)):
                continue
            elem.addObject(obj)

    def dumps(self):
        return self.Type,

    def loads(self, state):
        if state:
            self.Type, = state

    def _cleanup_children(self) -> DOList:
        """Remove and return all objects not supported by CROSS::Link."""
        if not self.is_execute_ready():
            return []
        removed_objects: set[DO] = set()
        # Group is managed by us and the containing robot.
        for o in self.joint.Group:
            if is_sensor_joint(o):
                # Supported, and managed by us.
                continue
            warn_unsupported(o, by='CROSS::Joint', gui=True)
            # implementation note: removeobject doesn't raise any exception
            # and `o` exists even if already removed from the group.
            removed_objects.update(self.joint.removeObject(o))

        return list(removed_objects)

    def is_fixed(self) -> bool:
        """Return whether the joint is of type 'fixed'."""
        return self.joint.Type == 'fixed'

    def get_actuation_placement(
        self,
        joint_value: Optional[float] = None,
    ) -> fc.Placement:
        """Return the transform due to actuation.

        Parameters
        ----------

        - joint_value: joint value in mm or deg. If `joint_value` is `None`,
                       the current value of the joint is used.

        """
        if not self.is_execute_ready():
            return fc.Placement()
        # Only actuation around/about z supported.
        if self.joint.Mimic and self.joint.MimickedJoint:
            mult = self.joint.Multiplier
            if self.joint.Type == 'prismatic':
                # User value in mm.
                off = self.joint.Offset / 1000.0
            elif self.joint.Type in ('revolute', 'continuous'):
                # User value in deg.
                off = radians(self.joint.Offset)
            else:
                warn('Mimicking joint must be prismatic, revolute or continuous', True)
                mult = 0.0
                off = 0.0
            p = self.joint.MimickedJoint.Position
            pos = mult * p + off
            if self.joint.Position != pos:
                # Implementation note: avoid recursion.
                self.joint.Position = pos
        if self.joint.Type == 'prismatic':
            if joint_value is None:
                joint_value = self.joint.Position * 1000.0
            return fc.Placement(fc.Vector(0.0, 0.0, joint_value), fc.Rotation())
        if self.joint.Type in ('revolute', 'continuous'):
            if joint_value is None:
                joint_value = degrees(self.joint.Position)
            return fc.Placement(
                fc.Vector(),
                fc.Rotation(
                    fc.Vector(0.0, 0.0, 1.0),
                    joint_value,
                ),
            )
        return fc.Placement()

    def get_robot(self) -> Optional[CrossRobot]:
        """Return the Cross::Robot this joint belongs to."""
        # TODO: as property.
        if (
            hasattr(self, '_robot')
            and self._robot
                and hasattr(self._robot, 'Group')
                and (self.joint in self._robot.Group)
        ):
            return self._robot
        if not self.is_execute_ready():
            return None
        for o in self.joint.InList:
            if is_robot(o):
                self._robot = cast(CrossRobot, o)
                return self._robot
        return None

    def get_predecessor(self) -> Optional[CrossJoint]:
        """Return the predecessing joint."""
        robot = self.get_robot()
        if robot is None:
            return None
        for candidate_joint in robot.Proxy.get_joints():
            child_of_candidate = robot.Proxy.get_link(candidate_joint.Child)
            parent_of_self = robot.Proxy.get_link(self.joint.Parent)
            if child_of_candidate is parent_of_self:
                return candidate_joint
        return None

    def get_unit_type(self) -> str:
        """Return `Length` or `Angle`."""
        if not self.is_execute_ready():
            return ''
        if self.joint.Type == 'prismatic':
            return 'Length'
        if self.joint.Type in ('revolute', 'continuous'):
            return 'Angle'
        return ''

    def export_urdf(self,format:str="urdf") -> et.ElementTree:
       
        joint = self.joint
        if format=="urdf":
            joint_xml = et.fromstring('<joint/>')
            joint_xml.attrib['name'] = get_valid_urdf_name(ros_name(joint))
            joint_xml.attrib['type'] = joint.Type
        elif format=="sdf":
            # this configures most ot the user defined properties 
            joint_xml:et.Element=export.create_sdf_element(joint,ros_name(joint),"joint")
        # this are similar for both sdf and urdf
            joint_xml.attrib['name'] = get_valid_urdf_name(ros_name(joint))
            if joint.Type=="continuous":
                # gazebo harmonic seems not to recognize continuous joints only revolute 
                # so basically a continuous  joint is a  revolute with range (-inf,inf)
                joint_xml.attrib['type'] = "revolute"
            
        
        # # find the axis element 
        ax=joint_xml.find("axis")
        if joint.Parent:
            if format=="urdf":
                joint_xml.append(et.fromstring(f'<parent link="{get_valid_urdf_name(joint.Parent)}"/>'))
            elif format=="sdf":
                #  parent was ignored hence not part of the joint_xml 
                # needs to be added 
                prnt=et.fromstring("<parent/>")
                prnt.text=get_valid_urdf_name(joint.Parent)
                joint_xml.append(prnt)
        else:
            joint_xml.append(et.fromstring('<parent link="NO_PARENT_DEFINED"/>'))
        if joint.Child:
            if format=="urdf":
                joint_xml.append(et.fromstring(f'<child link="{get_valid_urdf_name(joint.Child)}"/>'))
            elif format=="sdf":
                c=et.fromstring("<child/>")
                c.text=get_valid_urdf_name(joint.Child)
                joint_xml.append(c)
        else:
            joint_xml.append(et.fromstring('<child link="NO_CHILD_DEFINED"/>'))
        if format=="urdf":
            joint_xml.append(urdf_origin_from_placement(joint.Origin))
        elif format=="sdf":
            # this was also removed hence needs to be added i.e pose
            # get placement
            # ensure joints are aligned to child link axis 
            child_placement=export.ref_data.placement_info[joint.Child]
            placement=child_placement.inverse() *joint.Origin
            ps=urdf_origin_from_placement(placement,format=format)
            joint_xml.append(ps)
            pass
            # pose is defined  for link
        if joint.Type != 'fixed':
            if format=="urdf":
                joint_xml.append(et.fromstring('<axis xyz="0 0 1" />'))
            elif format=="sdf":
                # xyz element was ignored a child of the axis element add it 
                xyz=et.fromstring("<xyz/>")
                xyz.text="0 0 1"
                ax.append(xyz)
            if joint.Type in ('revolute', 'continuous'):
                factor = radians(1.0)
            else:
                factor = 0.001
            if joint.Type != 'continuous':
                # The URDF specification only says that the `limit` element
                # is optional for the `continuous` joint type. However, the
                # `urdfdom` library does not like if the `limit` element is
                # present but not complete but the `lower` and `upper` values
                # should not be set for the `continuous` joint type .
                # Cf. https://github.com/ros/urdfdom/issues/180.
                if format=="urdf":
                    limit_xml = et.fromstring('<limit/>')
                    limit_xml.attrib['lower'] = str(joint.LowerLimit * factor)
                    limit_xml.attrib['upper'] = str(joint.UpperLimit * factor)
                    limit_xml.attrib['velocity'] = str(joint.Velocity * factor)
                    limit_xml.attrib['effort'] = str(joint.Effort)
                    joint_xml.append(limit_xml)
                elif format=="sdf":
                    lim=ax.find("limit")
                    l=et.fromstring("<lower/>")
                    l.text=radians(joint.LowerLimit)
                    up=et.fromstring("<upper/>")
                    up.text=radians(joint.UpperLimit)
                    vel=et.fromstring("<velocity/>")
                    vel.text=joint.Velocity
                    eff=et.fromstring("<effort/>")
                    eff.text=joint.Effort
                    lim.append(l)
                    lim.append(up)
                    lim.append(vel)
                    lim.append(eff)
        elif joint.Type=="fixed":
            lim=ax.find("limit")
            if len(lim)==0:
                ax.remove(lim) 
        if joint.Mimic:
            if format=="urdf":
                mimic_xml = et.fromstring('<mimic/>')
                mimic_joint = ros_name(joint.MimickedJoint)
                mimic_xml.attrib['joint'] = get_valid_urdf_name(mimic_joint)
                mimic_xml.attrib['multiplier'] = str(joint.Multiplier)
                if joint.Type == 'prismatic':
                    # Millimeters (FreeCAD) to meters (URDF).
                    urdf_offset = joint.Offset / 1000.0
                else:
                    # Should be only 'revolute' or 'continuous'.
                    # Degrees (FreeCAD) to meters (URDF).
                    urdf_offset = radians(joint.Offset)
                mimic_xml.attrib['offset'] = str(urdf_offset)
                joint_xml.append(mimic_xml)
            elif format=="sdf":
                # ignore mimic for now
                pass
        return joint_xml

    def _toggle_editor_mode(self):
        if not self.is_execute_ready():
            return
        joint = self.joint
        if joint.Mimic:
            mimic_editor_mode = []
        else:
            mimic_editor_mode = ['Hidden', 'ReadOnly']
        joint.setEditorMode('MimickedJoint', mimic_editor_mode)
        joint.setEditorMode('Multiplier', mimic_editor_mode)
        joint.setEditorMode('Offset', mimic_editor_mode)

        if joint.Type in ('revolute', 'prismatic'):
            continuous_editor_mode = []
        else:
            continuous_editor_mode = ['Hidden']
        joint.setEditorMode('LowerLimit', continuous_editor_mode)
        joint.setEditorMode('UpperLimit', continuous_editor_mode)

        if joint.Type == 'fixed':
            fixed_editor_mode = ['Hidden']
        else:
            fixed_editor_mode = []
        joint.setEditorMode('Velocity', fixed_editor_mode)
        joint.setEditorMode('Effort', fixed_editor_mode)

    def get_sensors(self) -> list[CrossSensor]:
        """Return the list of CROSS sensors in the order of creation."""
        # TODO: as property.
        if self._sensors is not None:
            # self._sensors is updated in self.onChanged().
            return list(self._sensors)  # A copy.
        if not self.is_execute_ready():
            return []
        self._sensors = get_joint_sensors(self.joint.Group)
        return list(self._sensors)  # A copy.


class _ViewProviderJoint(ProxyBase):
    """The view provider for CROSS::Joint objects."""

    def __init__(self, vobj: VP) -> None:
        super().__init__(
            'view_object',
            [
                'AxisLength',
                'ShowAxis',
                'Visibility',
            ],
        )
        if vobj.Proxy is not self:
            # Implementation note: triggers `self.attach`.
            vobj.Proxy = self
        self._init(vobj)

    def _init(self, vobj: VP) -> None:
        self.view_object = vobj
        self.pose = vobj.Object
        self._init_extensions(vobj)
        self._init_properties(vobj)

    def _init_extensions(self, vobj: VP):
        vobj.addExtension('Gui::ViewProviderGroupExtensionPython')

    def _init_properties(self, vobj: VP) -> None:
        """Set properties of the view provider."""
        add_property(
            vobj, 'App::PropertyBool', 'ShowAxis',
            'ROS Display Options',
            "Toggle the display of the joint's Z-axis",
            True,
        )
        add_property(
            vobj, 'App::PropertyLength', 'AxisLength',
            'ROS Display Options',
            "Length of the arrow for the joint's axis",
            500.0,
        )

    def getIcon(self) -> str:
        # Implementation note: "return 'joint.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'joint.svg')

    def attach(self, vobj: VP) -> None:
        """Setup the scene sub-graph of the view provider."""
        # `self.__init__()` is not called on document restore, do it manually.
        self.__init__(vobj)

    def updateData(
        self,
        obj: CrossJoint,
        prop: str,
    ) -> None:
        # print(f'{obj.Name}.ViewObject.updateData({prop})') # DEBUG
        if not self.is_execute_ready():
            return
        if prop in ['Placement', 'Type', 'Position']:
            self.draw()
        # Implementation note: no need to react on prop == 'Origin' because
        # this triggers a change in 'Placement'.

    def onChanged(self, vobj: VP, prop: str) -> None:
        # print(f'{self.view_object.Object.Name}.onChanged({prop})') # DEBUG
        if prop in ('ShowAxis', 'AxisLength'):
            self.draw()
        if prop == 'Visibility':
            self.draw()

    def draw(self) -> None:
        from .coin_utils import arrow_group
        from .coin_utils import face_group

        if not self.is_execute_ready():
            return
        vobj = self.view_object
        if not hasattr(vobj, 'RootNode'):
            return
        root_node = vobj.RootNode
        root_node.removeAllChildren()
        if not (vobj.Visibility and vobj.AxisLength and vobj.ShowAxis):
            return
        obj = vobj.Object
        if not obj:
            return
        if not hasattr(obj, 'Placement'):
            return
        placement = obj.Placement  # A copy.
        if placement is None:
            return
        if obj.Type == 'fixed':
            color = (0.0, 0.0, 0.7)
        else:
            color = (0.0, 0.0, 1.0)
        if hasattr(vobj, 'AxisLength'):
            length = vobj.AxisLength.Value  # mm.
        else:
            length = 1000.0  # mm.
        p0 = placement.Base
        pz = placement * fc.Vector(0.0, 0.0, length)
        arrow = arrow_group([p0, pz], scale=0.2, color=color)
        root_node.addChild(arrow)
        px = placement * fc.Vector(length / 2.0, 0.0, 0.0)
        arrow = arrow_group([p0, px], scale=0.2, color=(1.0, 0.0, 0.0))
        root_node.addChild(arrow)
        py = placement * fc.Vector(0.0, length / 2.0, 0.0)
        arrow = arrow_group([p0, py], scale=0.2, color=(0.0, 1.0, 0.0))
        root_node.addChild(arrow)
        if obj.Type == 'prismatic':
            placement *= obj.Proxy.get_actuation_placement()
            scale = length * 0.05
            ps0 = placement * fc.Vector(+scale / 2.0, +scale / 2.0, 0.0)
            ps1 = placement * fc.Vector(-scale / 2.0, +scale / 2.0, 0.0)
            ps2 = placement * fc.Vector(-scale / 2.0, -scale / 2.0, 0.0)
            ps3 = placement * fc.Vector(+scale / 2.0, -scale / 2.0, 0.0)
            square = face_group([ps0, ps1, ps2, ps3], color=color)
            root_node.addChild(square)
        if obj.Type in ('revolute', 'continuous'):
            placement *= obj.Proxy.get_actuation_placement()
            scale = length * 0.2
            pt0 = placement * fc.Vector(0.0, 0.0, 0.0)
            pt1 = placement * fc.Vector(scale, 0.0, 0.0)
            pt2 = placement * fc.Vector(0.0, 0.0, scale / 2.0)
            triangle = face_group([pt0, pt1, pt2], color=color)
            root_node.addChild(triangle)

    def doubleClicked(self, vobj):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj, mode):
        return False

    def unsetEdit(self, vobj, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()

    def dumps(self):
        return None

    def loads(self, state) -> None:
        pass


def make_joint(name, doc: Optional[fc.Document] = None, robot:CrossRobot | None = None) -> CrossJoint:
    """Add a Cross::Joint to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    joint: CrossJoint = doc.addObject('App::FeaturePython', name)
    JointProxy(joint)
    # Default to type "fixed".
    joint.Type = 'fixed'
    joint.Label2 = name

    if robot:
        joint.adjustRelativeLinks(robot)
        robot.addObject(joint)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderJoint(joint.ViewObject)

        # Make `obj` part of the selected `Cross::Robot`.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if (
                is_robot(candidate)
                or is_workcell(candidate)
            ):
                joint.adjustRelativeLinks(candidate)
                candidate.addObject(joint)
                if is_robot(candidate) and candidate.ViewObject:
                    joint.ViewObject.AxisLength = candidate.ViewObject.JointAxisLength
            elif is_link(candidate):
                robot = candidate.Proxy.get_robot()
                if robot:
                    joint.adjustRelativeLinks(robot)
                    robot.addObject(joint)
                    joint.Parent = ros_name(candidate)
                    if robot.ViewObject:
                        joint.ViewObject.AxisLength = robot.ViewObject.JointAxisLength
    doc.recompute()
    return joint


def make_robot_joint_filled(link1:fc.DO, link2:fc.DO, robot:CrossRobot | None = None) -> CrossJoint :
    ''' Make robot joint and fill it with links (parent, child)  '''

    if is_link(link1) and is_link(link2):
        joint = make_joint(ros_name(link1) + '__to__' + ros_name(link2), robot = robot)
        try:
            joint.Parent = ros_name(link1)
            joint.Child = ros_name(link2)
        except ValueError:
            fc.ActiveDocument.removeObject(joint.Name)
            message(
                'Links must be in robot container for joint connection. Closed links loop does not supported.',
                True,
            )
        fc.ActiveDocument.recompute()

        return joint
    else:
        message(
            'Make filled robot joint(s) works only with selected links. Select minimum 2 links.',
            True,
        )

    return False


def make_robot_joints_filled(
        links:list[CrossLink] = [],
        robot:CrossRobot | None = None,
        joints_group_connect_type:  Union['chain': str, 'spider': str] = 'chain',
) -> list[CrossJoint] | False :
    ''' Make robot joints and fill it with selected links (parent, child) '''

    if not len(links):
        selection = fcgui.Selection.getSelection()
    else:
        selection = links

    sel_len = len(selection)
    if sel_len < 2:
        message(
            'Choose minimum 2 links.',
            True,
        )
        return False

    joints = [CrossJoint]

    if joints_group_connect_type == 'chain':
        for i in range(sel_len):
            if i+1 < sel_len:
                joints.append(make_robot_joint_filled(selection[i], selection[i+1], robot))
    elif joints_group_connect_type == 'spider':
        first_sel = None
        for i in range(sel_len):
            if first_sel is None:
                first_sel = i

            if i+1 < sel_len:
                joints.append(make_robot_joint_filled(selection[first_sel], selection[i+1], robot))
    else:
        raise TypeError('Joints group making connect_type (' + joints_group_connect_type + ') does not support.')

    return joints
