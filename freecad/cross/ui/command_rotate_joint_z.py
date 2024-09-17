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
        return {'Pixmap': 'rotate_joint_z.svg',
                'MenuText': tr('Rotate joint by Z axis'),
                'Accel': 'R, Z',
                'ToolTip': tr('Rotate joint by Z axis.\n'
                              '\n'
                              'Select: joint or link or subobject (body, part, etc) of link\n'
                              )}

    def IsActive(self):
        return True

    def Activated(self):
        doc = fc.activeDocument()

        doc.openTransaction(tr("Rotate joint origin by Z axis"))
        rotate_origin(x = None, y = None, z = 90)
        doc.commitTransaction()

        doc.recompute()


fcgui.addCommand('RotateJointZ', _RotateJointZCommand())
