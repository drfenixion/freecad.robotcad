import FreeCAD as fc

import FreeCADGui as fcgui
from freecad.cross.freecad_utils import is_assembly_from_assembly_wb

from ..gui_utils import tr
from ..robot_proxy import make_filled_robot, make_filled_robot_from_assembly


class _NewRobotCommand:
    """The command definition to create a new Robot object."""

    def GetResources(self):
        return {
            'Pixmap': 'robot.svg',
            'MenuText': tr('New Robot'),
            'Accel': 'N, R',
            'ToolTip': tr('Create a Robot container or Robot container with links and joints by selected objects \n'
                           'or Robot based on assembly from default Assembly WB. Choose an option:\n'
                              '\n'
                              '1) Select (objects): objects (parts, bodies, etc) in order of links in joints.\n'
                                'The order of objects selection will correspond to the order of links (parent, child) in the joints.\n'
                              '\n'
                              '2) Select (assembly): assembly from default Assembly workbench \n'
                                '(closed loop assembly is not supported, break the ring before use).\n'
                              '\n'
                              '3) If no object is selected, an empty robot container will be created.'
                              ),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        

        robot_name = 'Robot'
        sel = fcgui.Selection.getSelection()
        if len(sel) and is_assembly_from_assembly_wb(sel[0]):
            doc.openTransaction(tr('Convert assembly to robot'))
            assembly = fcgui.Selection.getSelection()[0]
            make_filled_robot_from_assembly(assembly)
            doc.commitTransaction()
        elif len(sel):
            doc.openTransaction(tr('Make filled robot'))
            make_filled_robot(robot_name)
            doc.commitTransaction()
        else:
            doc.openTransaction(tr('Create Robot'))
            fcgui.doCommand('')
            fcgui.addModule('freecad.cross.robot_proxy')
            fcgui.doCommand(f"_robot = freecad.cross.robot_proxy.make_robot('{robot_name}')")
            fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_robot.Name)')
            doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewRobot', _NewRobotCommand())
