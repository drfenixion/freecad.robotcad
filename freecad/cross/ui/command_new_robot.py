import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _NewRobotCommand:
    """The command definition to create a new Robot object."""

    def GetResources(self):
        return {
            'Pixmap': 'robot.svg',
            'MenuText': tr('New Robot'),
            'Accel': 'N, R',
            'ToolTip': tr('Create a Robot container.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Create Robot'))
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.robot_proxy')
        fcgui.doCommand("_robot = freecad.cross.robot_proxy.make_robot('Robot')")
        fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_robot.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewRobot', _NewRobotCommand())
