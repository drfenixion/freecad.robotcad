from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr
from ..joint_proxy import make_robot_joints_filled
from ..wb_utils import is_link


# Stubs and type hints.
from ..joint import Joint
from ..link import Link
DO = fc.DocumentObject
CrossLink = Link
CrossJoint = Joint
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


class _NewJointsFilledSpiderCommand:
    """Command to create new joints by selected links with spider connection.

        Links must be in robot container before
    """

    def GetResources(self):
        return {
            'Pixmap': 'links_to_joints_spider.svg',
            'MenuText': tr('New joints by selected links with spider connection'),
            'Accel': 'J, S',
            'ToolTip': tr(
                'New joints by selected links with spider connection.\n'
                '\n'
                'Select: robot links (minimum 2). Links must be in robot container.\n'
                '\n'
                'Joints will be created by selected links and all links will be connected to first link.\n',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection()) and is_link(fcgui.Selection.getSelection()[0])

    def Activated(self):
        doc = fc.activeDocument()

        doc.openTransaction(tr("New filled joints by selected links with spider connection"))
        make_robot_joints_filled(joints_group_connect_type = 'spider')
        doc.commitTransaction()

        doc.recompute()


fcgui.addCommand('NewJointsFilledSpider', _NewJointsFilledSpiderCommand())
