from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecad_utils import is_lcs
from ..gui_utils import tr
from ..wb_utils import rotate_origin


# Stubs and type hints.
from ..joint import Joint
from ..link import Link
DO = fc.DocumentObject
CrossLink = Link
CrossJoint = Joint
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


class _RotateJointYCommand:
    """Command to rotate joint by Y axis.
    """

    def GetResources(self):
        return {
            'Pixmap': 'rotate_joint_y.svg',
            'MenuText': tr('Rotate joint by Y axis'),
            'Accel': 'R, Y',
            'ToolTip': tr(
                'Rotate joint or link by Y axis.\n'
                '\n'
                'Select: joint or link or subobject (body, part, etc) of link\n'
                'It rotates joint Origin or Link MountedPlacement dependent on selection.\n',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        doc = fc.activeDocument()

        doc.openTransaction(tr("Rotate joint origin by Y axis"))
        rotate_origin(x = None, y = 90, z = None)
        doc.commitTransaction()

        doc.recompute()


fcgui.addCommand('RotateJointY', _RotateJointYCommand())
