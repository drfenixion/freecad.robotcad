import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr
from ..wb_utils import is_link_selected
from ..wb_utils import is_robot_selected
from ..wb_utils import is_workcell_selected


class _NewJointCommand:
    """The command definition to create a new Joint object."""

    def GetResources(self):
        return {
            'Pixmap': 'joint.svg',
            'MenuText': tr('New Joint'),
            'Accel': 'N, J',
            'ToolTip': tr('Create a Joint.'),
        }

    def IsActive(self):
        return (
            is_robot_selected()
            or is_link_selected()
            or is_workcell_selected()
        )

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction('Create Joint')
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.joint_proxy')
        fcgui.doCommand("_joint = freecad.cross.joint_proxy.make_joint('Joint')")
        fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_joint.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewJoint', _NewJointCommand())
