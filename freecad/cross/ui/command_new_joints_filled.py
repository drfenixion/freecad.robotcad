from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecad_utils import is_lcs
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


class _NewJointsFilledCommand:
    """Command to create new joints by selected links.

        Links must be in robot container before
    """

    def GetResources(self):
        return {
            'Pixmap': 'links_to_joints.svg',
            'MenuText': tr('New joints by selected links with chain connection'),
            'Accel': 'J, F',
            'ToolTip': tr(
                'New joints by selected links with chain connection.\n'
                '\n'
                'Select: robot links (minimum 2). Links must be in robot container.\n'
                '\n'
                'Joints will be created by selected links with chain connection.\n'
                'Order of links in selection is order of links in joints.\n',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection()) and is_link(fcgui.Selection.getSelection()[0])

    def Activated(self):
        doc = fc.activeDocument()

        doc.openTransaction(tr("New filled joints by selected links"))
        make_robot_joints_filled()
        doc.commitTransaction()

        doc.recompute()


fcgui.addCommand('NewJointsFilled', _NewJointsFilledCommand())
