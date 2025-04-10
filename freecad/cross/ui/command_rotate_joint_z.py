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


class _RotateJointZCommand:
    """Command to rotate joint by Z axis.
    """

    def GetResources(self):
        return {
            'Pixmap': 'rotate_joint_z.svg',
            'MenuText': tr('Rotate joint/link by Z axis'),
            'Accel': 'R, Z',
            'ToolTip': tr(
                'Rotate joint or link by Z axis.\n'
                '\n'
                'Select: joint or link or subelement (face, edge, vertex) of link Real or LCS of link Real.\n'
                'It rotates joint Origin or Link MountedPlacement dependent on selection.\n'
                'If selected subelement or LCS or body it rotate link around it center\n'
                'or center of gravity or concentric for curve and circle.\n',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        doc = fc.activeDocument()

        doc.openTransaction(tr("Rotate joint origin by Z axis"))
        rotate_origin(x = None, y = None, z = 90)
        doc.commitTransaction()

        doc.recompute()


fcgui.addCommand('RotateJointZ', _RotateJointZCommand())
