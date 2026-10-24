import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _RobotFromKkCommand:
    """
    The command definition to create a new CROSS::Robot from KK.

    The command definition to create a new CROSS::Robot from Khalil-Kleinfinger
    parameters read from a CSV file.

    """

    def GetResources(self):
        return {
            # 'Pixmap': 'observer.svg',
            'MenuText': tr('New Robot from KK'),
            'Accel': 'N, K',
            'ToolTip': tr('New Robot from Khalil-Kleinfinger parameters.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction('Create Robot')
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.robot_from_kk')
        # fcgui.doCommand("_robot = freecad.cross.robot_from_kk.robot_from_kk('Observer')")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_observer.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewObserver', _RobotFromKkCommand())
