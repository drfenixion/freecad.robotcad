from __future__ import annotations

from typing import Any, Optional

import FreeCAD as fc

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import warn
from .wb_utils import ICON_PATH
from .wb_utils import is_link
from .wb_utils import is_robot
from .wb_utils import ros_name

try:
    from moveit_msgs.msg import PlanningScene as PlanningSceneMsg
except ImportError:
    PlanningSceneMsg = Any

# Stubs and type hints.
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .pose import Pose as CrossPose  # A Cross::Pose, i.e. a DocumentObject with Proxy "Pose". # noqa: E501
from .pose import ViewProviderPose as VP


class PoseProxy(ProxyBase):
    """The proxy for CROSS::Pose objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::Pose'

    def __init__(self,
                 obj: CrossPose,
                 planning_scene_msg: Optional[PlanningSceneMsg] = None):
        """Initialize the proxy.

        Not called on document restore.

        """
        warn(f'{obj.Name}.__init__()') # DEBUG
        super().__init__('pose', [
            'AllowNonLeafLink',
            'EndEffector',
            'Placement',  # Set by `App::GeometryPython`.
            'Robot',
            '_Type',
            ])
        if obj.Proxy is not self:
            obj.Proxy = self
        self._init(obj)

    def _init(self, obj: CrossPose) -> None:
        self.pose = obj
        self._init_properties(obj)
        if obj.ViewObject and obj.ViewObject.Proxy:
            # `onDocumentRestored` is called after the view object is created.
            # This means that all calls to draw are discarded because the
            # properties of `obj` are not set.
            # Call `draw` manually.
            obj.ViewObject.Proxy.draw()

    def _init_properties(self, obj: CrossPose) -> None:
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(obj, 'App::PropertyLink', 'Robot', 'Robot',
                     'The associated robot')
        add_property(obj, 'App::PropertyEnumeration', 'EndEffector', 'Robot',
                     'End-effector link (from CROSS) to bring to the pose')
        add_property(obj, 'App::PropertyBool', 'AllowNonLeafLink', 'Robot',
                     'Whether to use allow non-leaf links in `EndEffector`',
                     False)

    def onDocumentRestored(self, obj: CrossPose) -> None:
        """Handle the object after a document restore.

        Required by FreeCAD.

        """
        warn(f'{obj.Name}.onDocumentRestored()') # DEBUG
        # `self.__init__()` is not called on document restore, do it manually.
        self.__init__(obj)

    def dumps(self):
        return self.Type,

    def __getstate__(self):
        # Deprecated.
        return self.dumps()

    def loads(self, state):
        if state:
            self.Type, = state

    def __setstate__(self, state):
        # Deprecated.
        return self.loads(state)

    def onChanged(self, obj: CrossPose, prop: str) -> None:
        """Handle a property change of the object, after the change.

        Required by FreeCAD.

        """
        if prop in ['AllowNonLeafLink', 'Robot']:
            self._update_end_effector_list()

    def _update_end_effector_list(self) -> None:
        """Update the list of potential end-effectors."""
        if not self.is_execute_ready():
            return
        pose_obj = self.pose
        if not pose_obj.Robot:
            pose_obj.EndEffector = []
            return
        robot = pose_obj.Robot
        if ((not hasattr(robot, 'Proxy'))
                or (robot.Proxy is None)
                or (not robot.Proxy.is_execute_ready())):
            return
        chains = robot.Proxy.get_chains()
        links: set[CrossLink] = set()
        for chain in chains:
            if pose_obj.AllowNonLeafLink:
                links.update([ln for ln in chain if is_link(ln)])
            else:
                links.add(chain[-1])
        link_str = sorted([ros_name(ln) for ln in links])
        pose_obj.EndEffector = link_str


class _ViewProviderPose(ProxyBase):
    """The view provider for CROSS::Pose objects."""

    def __init__(self, vobj: VP) -> None:
        """Initialize the view provider.

        Not called on document restore.

        """
        warn(f'view_object({vobj.Object.Name}).__init__()') # DEBUG
        super().__init__('view_object', [
            'AxisLength',
            'ShowEndEffector',
            ])

        if vobj.Proxy is not self:
            # Implementation note: triggers `self.attach`.
            vobj.Proxy = self
        self._init(vobj)

    def _init(self, vobj: VP) -> None:
        self.view_object = vobj
        self.pose = vobj.Object
        self._init_properties(vobj)

    def _init_properties(self, vobj: VP) -> None:
        add_property(vobj, 'App::PropertyBool', 'ShowEndEffector', 'ROS Display Options',
                     'Whether to show the end-effector link or a symbolic'
                     'representation of the pose',
                     True)
        add_property(vobj, 'App::PropertyLength', 'AxisLength',
                     'ROS Display Options',
                     "Length of the rods for the joint's axis",
                     500.0)  # mm.

    def getIcon(self) -> str:
        """Return the icon for the view provider.

        Required by FreeCAD.

        """
        # Implementation note: "return 'pose.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'pose.svg')

    def attach(self, vobj: VP) -> None:
        """Setup the scene sub-graph of the view provider.

        Required by FreeCAD. This is the first method called on document
        restore (`__init__` is not called).

        """
        warn(f'view_object({vobj.Object.Name}).attach()') # DEBUG
        from pivy import coin

        # `self.__init__()` is not called on document restore, do it manually.
        self.__init__(vobj)

        self.shaded = coin.SoGroup()

        style = coin.SoDrawStyle()
        style.style = coin.SoDrawStyle.LINES
        self.wireframe = coin.SoGroup()
        self.wireframe.addChild(style)

        vobj.addDisplayMode(self.shaded, 'Shaded')
        vobj.addDisplayMode(self.wireframe, 'Wireframe')

    def updateData(self,
                   obj: CrossPose,
                   prop: str) -> None:
        """Handle a property change of the object, after the change.

        Required by FreeCAD.

        """
        print(f'{obj.Name}.updateData({prop})') # DEBUG
        self.draw()

    def onChanged(self, vobj: VP, prop: str) -> None:
        """Handle a property change of the view object, after the change.

        Required by FreeCAD.

        """
        print(f'view_object({vobj.Object.Name}).onChanged({prop})') # DEBUG
        self.draw()

    def getDisplayModes(self, vobj: VP) -> list[str]:
        """Return a list of display modes.

        Required by FreeCAD.

        """
        return ['Shaded', 'Wireframe']

    def getDefaultDisplayMode(self) -> str:
        """Return the name of the default display mode.

        Required by FreeCAD.

        """
        return 'Shaded'

    def setDisplayMode(self, mode: str) -> str:
        """Set the display mode.

        Required by FreeCAD.

        """
        return mode

    def draw(self) -> None:
        print(f'view_object({self.view_object.Object.Name}).draw()') # DEBUG
        from pivy import coin
        from .coin_utils import tcp_group
        from .coin_utils import transform_from_placement

        if ((not self.is_execute_ready(debug=True))
                or (not hasattr(self, 'shaded'))
                or (not hasattr(self, 'wireframe'))):
            return

        vobj: VP = self.view_object
        obj: CrossPose = vobj.Object

        self.shaded.removeAllChildren()
        self.wireframe.removeAllChildren()
        style = coin.SoDrawStyle()
        style.style = coin.SoDrawStyle.LINES
        self.wireframe.addChild(style)

        if not vobj.Visibility:
            return
        sep = coin.SoSeparator()
        # When `self.pose` is a `App::GeometryPython` object, the
        # placement of the view object is managed by the view object itself
        # and sep.addChild(transform_from_placement(Placement)) should not be used.
        sep.addChild(tcp_group(
            tcp_length_mm=0.66 * vobj.AxisLength,
            tcp_diameter_ratio_to_length=0.17,
            tcp_color=(0.7, 0.7, 0.7),
            axis_length_mm=vobj.AxisLength,
            axis_diameter_ratio_to_length=0.03,
            ))

        if (self.pose.Proxy.is_execute_ready(debug=True)
                and vobj.ShowEndEffector
                and obj.Robot):
            robot_proxy = obj.Robot.Proxy
            link = robot_proxy.get_link(obj.EndEffector)
            if link is not None:
                # TODO: only works if robot at null-joint-space pose.
                # TODO: get the correct transform.
                sep.addChild(_get_link_separator(link))
        else:
            warn(f'{vobj.ShowEndEffector=}, {obj.Robot=}') # DEBUG

        self.shaded.addChild(sep)
        self.wireframe.addChild(sep)


def make_pose(name, doc: Optional[fc.Document] = None) -> CrossPose:
    """Add a Cross::Pose to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossPose = doc.addObject('App::GeometryPython', name)
    PoseProxy(obj)
    obj.Label2 = name

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderPose(obj.ViewObject)

        # Set `obj.Robot` and possibly `obj.EndEffector` if the selected
        # object is a robot or a link.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if is_robot(candidate):
                obj.Robot = candidate
            elif is_link(candidate) and (hasattr(candidate, 'Proxy')):
                robot = candidate.Proxy.get_robot()
                if robot:
                    obj.Robot = robot
                    link_name = ros_name(candidate)
                    if link_name not in obj.getEnumerationsOfProperty('EndEffector'):
                        obj.AllowNonLeafLink = True
                    obj.EndEffector = ros_name(candidate)
    return obj


def _get_group_separator(link: CrossLink) -> 'coin.SoSeparator':
    """Return the SoSeparator with all element in `link.Group`.

    Elements in `link.Group` are FreeCAD links. Their inventor representation
    contains an SoTransform node with their location in the 3D scene and an
    SoSwitch node for the different display nodes.
    For each element, we ignore the SoTransform and add only the SoSwitch to
    the separator.

    """
    from pivy import coin
    from .coin_utils import transform_from_placement

    sep = coin.SoSeparator()
    sep.addChild(transform_from_placement(link.MountedPlacement))
    for fc_link in link.Group:
        if fc_link.ViewObject is None:
            continue
        for node in fc_link.ViewObject.RootNode.getChildren():
            if isinstance(node, coin.SoSwitch):
                sep.addChild(node)
                break
    return sep


def _get_link_separator(link: CrossLink) -> 'coin.SoSeparator':
    """Return the SoSeparator of the link.

    Apply the transform `link.MountedPlacement` and add a SoSeparator with all
    elements in `link.Group` and all CROSS::Link objects that are fixed to
    `link`.

    """
    from pivy import coin
    from .coin_utils import transform_from_placement

    sep = coin.SoSeparator()

    if not link.Proxy.is_execute_ready(debug=True):
        return sep
    robot = link.Proxy.get_robot()
    if robot is None:
        sep.addChild(_get_group_separator(link))
        return sep
    for fixed_with_link in robot.Proxy.get_links_fixed_with(ros_name(link)):
        ln_sep = coin.SoSeparator()
        fixed_transform = robot.Proxy.get_transform(ros_name(link),
                                                    ros_name(fixed_with_link))
        ln_sep.addChild(transform_from_placement(fixed_transform))
        ln_sep.addChild(_get_group_separator(fixed_with_link))
        sep.addChild(ln_sep)
    return sep
