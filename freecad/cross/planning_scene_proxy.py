"""Pure GUI object to display the content of a moveit_msgs/msg/PlanningScene message."""

from __future__ import annotations

from typing import Any, Optional
from xml.etree import ElementTree as et

import FreeCAD as fc

from pivy import coin

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import warn
from .gui_utils import tr
from .planning_scene_utils import coin_from_planning_scene_msg
from .wb_utils import ICON_PATH
from .wb_utils import is_robot

try:
    from moveit_msgs.msg import PlanningSceneMsg
except ImportError:
    PlanningSceneMsg = Any

# Stubs and type hints.
from .planning_scene import PlanningScene as CrossPlanningScene  # A Cross::PlanningScene, i.e. a DocumentObject with Proxy "PlanningScene". # noqa: E501
from .planning_scene import ViewProviderPlanningScene as VP


class PlanningSceneProxy(ProxyBase):
    """The proxy for CROSS::PlanningScene objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::PlanningScene'

    def __init__(
        self,
        obj: CrossPlanningScene,
        planning_scene_msg: Optional[PlanningSceneMsg] = None,
    ):
        super().__init__(
            'scene',
            [
                '_Type',
                'Robot',
            ],
        )
        obj.Proxy = self
        self.scene = obj

        self.planning_scene_msg = planning_scene_msg

        self.init_properties(obj)

    def init_properties(self, obj: CrossPlanningScene):
        add_property(
                obj, 'App::PropertyString', '_Type', 'Internal',
                'The type',
        )
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(
                obj, 'App::PropertyLink', 'Robot', 'ROS',
                tr('The associated robot.'),
        )

    def execute(self, obj: CrossPlanningScene) -> None:
        self._update_robot_joint_values()

    def onChanged(self, obj: CrossPlanningScene, prop: str) -> None:
        if prop == 'Robot':
            if obj.Robot and (not is_robot(obj.Robot)):
                if obj.Robot:
                    warn('The selected object is not a robot, rejecting.', True)
                obj.Robot = None

    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        self.__init__(obj)

    def dumps(self):
        return self.Type,

    def loads(self, state):
        self.planning_scene_msg = None  # TODO
        if state:
            self.Type, = state

    def update_scene(self, planning_scene_msg: PlanningSceneMsg) -> None:
        """Update the scene with a new PlanningScene message."""
        self.planning_scene_msg = planning_scene_msg
        self.scene.recompute()
        if self.scene.ViewObject and self.scene.ViewObject.Proxy:
            self.scene.ViewObject.Proxy.draw()

    def export_urdf(self, interactive: bool = False) -> Optional[et.Element]:
        """Export the scene as URDF, writing files."""
        xml = et.fromstring('<robot/>')
        return xml

    def _update_robot_joint_values(self) -> None:
        """Update the joint values of the robot."""
        if not self.is_execute_ready():
            return
        if not self.scene.Robot:
            return
        robot = self.scene.Robot
        if not self.planning_scene_msg:
            return
        names = self.planning_scene_msg.robot_state.joint_state.name
        joints = [robot.Proxy.get_joint(n) for n in names]
        positions = self.planning_scene_msg.robot_state.joint_state.position
        position_map = {j: float(p) for j, p in zip(joints, positions) if j}
        robot.Proxy.set_joint_values(position_map)


class _ViewProviderPlanningScene(ProxyBase):
    """A view provider for the PlanningScene object """

    def __init__(self, vobj: VP):
        super().__init__(
            'view_object',
            [
                'Visibility',
                'PlaneSides',
            ],
        )

        if vobj.Proxy is not self:
            # Implementation note: triggers `self.attach`.
            vobj.Proxy = self
        self._init(vobj)

    def _init(self, vobj: VP) -> None:
        self.view_object = vobj
        self.scene = vobj.Object
        self._init_properties(vobj)

    def _init_properties(self, vobj: VP):
        """Set properties of the view provider."""
        # Default to 1000.0, i.e. 1 meter.
        add_property(
            vobj, 'App::PropertyLength', 'PlaneSides',
            'ROS Display Options',
            'The length of the sides of the plane.',
            1000.0,
        )

        # Default to 100.0, i.e. 100 cm.
        add_property(
            vobj, 'App::PropertyLength', 'SubframeSize',
            'ROS Display Options',
            'The length of the subframe axes.',
            100.0,
        )

    def getIcon(self):
        # Implementation note: "return 'planning_scene.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'planning_scene.svg')

    def attach(self, vobj: VP):
        """Setup the scene sub-graph of the view provider, this method is mandatory."""
        # `self.__init__()` is not called on document restore, do it manually.
        self.__init__(vobj)

        self.shaded = coin.SoGroup()

        style = coin.SoDrawStyle()
        style.style = coin.SoDrawStyle.LINES
        self.wireframe = coin.SoGroup()
        self.wireframe.addChild(style)

        vobj.addDisplayMode(self.shaded, 'Shaded')
        vobj.addDisplayMode(self.wireframe, 'Wireframe')

    def updateData(self, obj: CrossPlanningScene, prop: str) -> None:
        pass

    def onChanged(self, vobj: VP, prop: str) -> None:
        self.draw()

    def getDisplayModes(self, vobj: VP) -> list[str]:
        """Return a list of display modes."""
        modes = []
        modes.append('Shaded')
        modes.append('Wireframe')
        return modes

    def getDefaultDisplayMode(self) -> str:
        """Return the name of the default display mode.."""
        return 'Shaded'

    def setDisplayMode(self, mode: str) -> str:
        return mode

    def dumps(self):
        return None

    def loads(self, state) -> None:
        pass

    def draw(self) -> None:
        if ((not self.is_execute_ready())
                or (not hasattr(self.view_object.Proxy, 'shaded'))
                or (not hasattr(self.view_object.Proxy, 'wireframe'))):
            return
        scene: CrossPlanningScene = self.view_object.Object
        if not hasattr(scene.Proxy, 'planning_scene_msg'):
            return
        msg = scene.Proxy.planning_scene_msg

        self.shaded.removeAllChildren()
        self.wireframe.removeAllChildren()
        style = coin.SoDrawStyle()
        style.style = coin.SoDrawStyle.LINES
        self.wireframe.addChild(style)

        if not self.view_object.Visibility:
            return

        if not msg:
            return

        planning_scene_group = coin_from_planning_scene_msg(
                msg,
                plane_sides_mm=self.view_object.PlaneSides,
                subframe_length_mm=self.view_object.SubframeSize,
        )
        self.shaded.addChild(planning_scene_group)
        self.wireframe.addChild(planning_scene_group)


def make_planning_scene(
        name: str,
        planning_scene_msg: PlanningSceneMsg,
        doc: Optional[fc.Document] = None,
) -> CrossPlanningScene:
    """Add a Cross::PlanningScene to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossPlanningScene = doc.addObject('App::FeaturePython', name)
    PlanningSceneProxy(obj, planning_scene_msg)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderPlanningScene(obj.ViewObject)

        # Set the selected object as obj.Robot if applicable.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if is_robot(candidate):
                obj.Robot = candidate

    doc.recompute()
    return obj
