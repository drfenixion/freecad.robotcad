from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr
from ..wb_utils import is_robot_selected
from .set_joints_dialog import SetJointsDialog


class _SetJointsCommand:
    """The command definition to create a set the joint values of a robot."""

    def GetResources(self):
        return {
            'Pixmap': 'set_joints.svg',
            'MenuText': tr('Set Joints'),
            'Accel': 'S, J',
            'ToolTip': tr('Set the joint values of a robot.'),
        }

    def IsActive(self):
        return is_robot_selected()

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Set Joints'))
        objs = fcgui.Selection.getSelection()
        if not objs:
            doc = fc.activeDocument()
        else:
            robot = objs[0]
        diag = SetJointsDialog(robot, fcgui.getMainWindow())
        joint_values = diag.exec_()
        diag.close()
        if joint_values:
            print(f'Activated(), {joint_values=}')
            robot.Proxy.set_joint_values(joint_values)
        doc.recompute()
        doc.commitTransaction()


fcgui.addCommand('SetJoints', _SetJointsCommand())
