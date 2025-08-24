from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr
from ..wb_utils import set_placement_fast


# Stubs and type hints.
from ..joint import Joint
from ..link import Link
DO = fc.DocumentObject
CrossLink = Link
CrossJoint = Joint
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


class _SetCROSSPlacementFastCommand:
    """Command to set the Origin of a joint and Mounted Placement of link and make LCS.
    """

    def GetResources(self):
        return {
            'Pixmap': 'set_cross_placement_fast.svg',
            'MenuText': tr('Set placement - fast'),
            'Accel': 'P, F',
            'ToolTip': tr(
                'Set the Origin of a joint and Mounted Placement of link.\n'
                'Use it for current unplacement tip of kinematic branch.\n'
                '\n'
                'Select (with Ctlr): \n'
                '    1) subelement (face, edge, vertex, LCS) of body (of Real) of robot link (first reference)\n'
                '    2) subelement (face, edge, vertex, LCS) of body (of Real) of robot link (second reference)\n'
                '\n'
                'Robot links must be near to each other in chain (parent, child) and have joint between.\n'
                '\n'
                'This will connect 2 links (child to parent) in reference places.\n'
                'Joint Origin and link Mounted Placement will be moved to connection position.\n'
                '\n'
                'If selected subelement (face, edge, vertex) will be used temporary LCS underhood.\n'
                '\n'
                'Dont use this for moving kinematic branch because it also set MountedPlacement\n'
                'that is may not be desirable in this case.\n',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        set_placement_fast()


fcgui.addCommand('SetCROSSPlacementFast', _SetCROSSPlacementFastCommand())
