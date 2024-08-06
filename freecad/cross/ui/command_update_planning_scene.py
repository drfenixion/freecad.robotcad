from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import validate_types
from ..freecad_utils import warn
from ..gui_utils import tr
from ..wb_utils import is_planning_scene_selected



class _UpdatePlanningSceneCommand:
    """The command definition to get the planning scene from a service.

    The command definition to get the planning scene from a service
    (usually /get_planning_scene).

    """

    def GetResources(self):
        return {
            'Pixmap': 'update_planning_scene.svg',
            'MenuText': tr('Update Planning Scene'),
            'Accel': 'U, P',
            'ToolTip': tr('Update the selected PlanningScene object with the current planning scene from the /get_planning_scene service.'),
        }

    def IsActive(self):
        return is_planning_scene_selected()

    def Activated(self):
        doc = fc.activeDocument()
        try:
            cross_planning_scene, = validate_types(
                fcgui.Selection.getSelection(), ['Cross::PlanningScene'],
            )
        except RuntimeError:
            # The command is active only when a Cross::PlanningScene is active,
            # this should not happen.
            warn('Internal error, no Cross::PlanningScene selected', true)
            return

        fcgui.addModule('freecad.cross.ros.planning_scene')
        fcgui.doCommand('_scene_msg = freecad.cross.ros.planning_scene.get_planning_scene(timeout_sec=10.0)')
        doc.openTransaction(tr('Update Planning Scene'))
        fcgui.doCommand(
            'if _scene_msg is not None:\n'
            f"     FreeCAD.getDocument('{doc.Name}').getObject('{cross_planning_scene.Name}').Proxy.update_scene(_scene_msg)",
        )
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_scene.Name)')
        doc.recompute()
        doc.commitTransaction()
        fcgui.doCommand("Gui.SendMsgToActiveView('ViewFit')")


fcgui.addCommand('UpdatePlanningScene', _UpdatePlanningSceneCommand())
