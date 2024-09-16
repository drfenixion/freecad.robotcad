import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr
from ..robot_proxy import make_filled_robot


class _NewRobotCommand:
    """The command definition to create a new Robot object."""

    def GetResources(self):
        return {
            'Pixmap': 'robot.svg',
            'MenuText': tr('New Robot'),
            'Accel': 'N, R',
            'ToolTip': tr('Create a Robot container or Robot cantainer with links and joints by selected object.\n'
                              '\n'
                              'Select (for filled robot): objects (parts, bodies, etc) in order of links in joints.\n'
                              '\n'
                              'The order of objects selection will correspond to the order of links (parent, child) in the joints.\n'
                              'If no object is selected, an empty robot container will be created.'
                              ),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Create Robot'))

        robot_name = 'Robot'
        selection = fcgui.Selection.getSelection()
        if len(selection):
            make_filled_robot(robot_name)
        else:
            fcgui.doCommand('')
            fcgui.addModule('freecad.cross.robot_proxy')
            fcgui.doCommand(f"_robot = freecad.cross.robot_proxy.make_robot('{robot_name}')")
            fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_robot.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewRobot', _NewRobotCommand())
