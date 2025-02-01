"""Command to edit the selected robot in Khalil-Kleinfinger representation.

Command to edit the selected robot in Khalil-Kleinfinger (KK) representation.
Denavit-Hartenberg (DH) parameters can also be used to represent the robot.
A new robot is created if nothing is selected.

"""
from __future__ import annotations

import FreeCAD as fc
import FreeCADGui as fcgui

from ..gui_utils import tr


def _supported_object_selected():
    # Import late to avoid slowing down workbench start-up.
    from ..wb_utils import is_robot

    objs = fcgui.Selection.getSelection()
    if not objs:
        return True
    if is_robot(objs[0]):
        return True
    return False


class _KKEditCommand:
    """Command to edit the selected robot in KK representation."""

    def GetResources(self):
        return {
            'Pixmap': 'kk_edit.svg',
            'MenuText': tr('Edit DH or KK parameters'),
            'ToolTip': tr('Edit the Denavit-Hartenberg or Khalil-Kleinfinger parameters of the selected robot'),
        }

    def Activated(self):
        # Import late to avoid slowing down workbench start-up.
        from ..freecad_utils import warn
        from ..robot_proxy import make_robot
        from .kk_dialog import KKDialog

        objs = fcgui.Selection.getSelection()
        if not objs:
            doc = fc.activeDocument()
            if doc is None:
                doc = fc.newDocument()
            if not doc:
                warn('Could not create a document', True)
                return
            robot = make_robot('Robot')
        else:
            robot = objs[0]
        diag = KKDialog(robot)
        kk_robot = diag.exec_()
        diag.close()
        if not kk_robot:
            return
        doc.openTransaction(tr('Change robot from DH or KK'))
        kk_robot.transfer_to_robot(robot)
        doc.commitTransaction()

    def IsActive(self):
        return _supported_object_selected()


fcgui.addCommand('KKEdit', _KKEditCommand())
