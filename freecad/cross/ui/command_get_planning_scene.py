from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _GetPlanningSceneCommand:
    """The command definition to get the planning scene from a service.

    The command definition to get the planning scene from a service
    (usually /get_planning_scene).

    """

    def GetResources(self):
        return {'Pixmap': 'planning_scene.svg',
                'MenuText': tr('Get Planning Scene'),
                'Accel': 'S, J',
                'ToolTip': tr('Get the Current Planning Scene')}

    def IsActive(self):
        return True

    def Activated(self):
        doc = fc.activeDocument()
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.planning_scene_proxy')
        fcgui.addModule('freecad.cross.ros.planning_scene')
        fcgui.doCommand('_scene_msg = freecad.cross.ros.planning_scene.get_planning_scene(timeout_sec=1.0)')
        doc.openTransaction(tr('Get Planning Scene'))
        fcgui.doCommand('if _scene_msg is None:\n'
                        '    _scene = None\n'
                        'else:\n'
                        "    _scene = freecad.cross.planning_scene_proxy.make_planning_scene(_scene_msg.name, _scene_msg)")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_scene.Name)')
        doc.recompute()
        doc.commitTransaction()
        fcgui.doCommand("Gui.SendMsgToActiveView('ViewFit')")


fcgui.addCommand('GetPlanningScene', _GetPlanningSceneCommand())
