"""Pure GUI object to display the content of a moveit_msgs/msg/PlanningScene message."""

from __future__ import annotations

from typing import Any, ForwardRef, Optional
from xml.etree import ElementTree as et

import FreeCAD as fc

from pivy import coin

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import quantity_as
from .freecad_utils import warn
from .planning_scene_utils import coin_from_planning_scene_msg
from .wb_utils import ICON_PATH

try:
    from moveit_msgs.msg import PlanningScene
    imports_ok = True
except ImportError:
    PlanningScene = Any
    imports_ok = False

# Stubs and type hints.
from .planning_scene import PlanningScene as CrossPlanningScene  # A Cross::PlanningScene, i.e. a DocumentObject with Proxy "PlanningScene". # noqa: E501
from .planning_scene import ViewProviderPlanningScene as VP


class PlanningSceneProxy(ProxyBase):
    """The proxy for CROSS::PlanningScene objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::PlanningScene'

    def __init__(self,
                 obj: CrossPlanningScene,
                 planning_scene_msg: Optional[PlanningScene] = None):
        super().__init__('scene', [
            '_Type',
            ])
        obj.Proxy = self
        self.scene = obj

        self.planning_scene_msg = planning_scene_msg

        self.init_properties(obj)

    def init_properties(self, obj: CrossPlanningScene):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        # TODO: export as package (+OutputPath) or single file.

    def execute(self, obj: CrossPlanningScene) -> None:
        pass

    def onChanged(self, obj: CrossPlanningScene, prop: str) -> None:
        pass

    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        self.__init__(obj)

    def __getstate__(self):
        return self.Type,

    def __setstate__(self, state):
        if state:
            self.Type, = state

    def export_urdf(self, interactive: bool = False) -> Optional[et.Element]:
        """Export the scene as URDF, writing files."""
        xml = et.fromstring('<robot/>')
        return xml


class _ViewProviderPlanningScene(ProxyBase):
    """A view provider for the PlanningScene object """

    def __init__(self, vobj: VP):
        super().__init__('view_object', [
            'Visibility',
            'PlaneSides',
            ])

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
        add_property(vobj, 'App::PropertyLength', 'PlaneSides',
                     'ROS Display Options',
                     'The length of the sides of the plane.',
                     1000.0)

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
        if ((not self.is_execute_ready())
                or (not hasattr(vobj.Proxy, 'shaded'))
                or (not hasattr(vobj.Proxy, 'wireframe'))):
            return
        scene: CrossPlanningScene = vobj.Object
        msg = scene.Proxy.planning_scene_msg

        self.shaded.removeAllChildren()
        self.wireframe.removeAllChildren()
        style = coin.SoDrawStyle()
        style.style = coin.SoDrawStyle.LINES
        self.wireframe.addChild(style)

        if not self.view_object.Visibility:
            return

        plane_sides_mm = quantity_as(self.view_object.PlaneSides, 'mm')
        planning_scene_group = coin_from_planning_scene_msg(
                msg, plane_sides_mm=plane_sides_mm)
        self.shaded.addChild(planning_scene_group)
        self.wireframe.addChild(planning_scene_group)

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

    def __getstate__(self) -> None:
        return

    def __setstate__(self, state) -> None:
        return


def make_planning_scene(name, planning_scene_msg: PlanningScene, doc: Optional[fc.Document] = None) -> CrossPlanningScene:
    """Add a Cross::PlanningScene to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossPlanningScene = doc.addObject('App::FeaturePython', name)
    PlanningSceneProxy(obj, planning_scene_msg)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderPlanningScene(obj.ViewObject)

    doc.recompute()
    return obj
