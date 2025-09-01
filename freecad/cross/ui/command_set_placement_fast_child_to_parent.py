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


class _SetCROSSPlacementFastChildToParentCommand:
    """Command to set the Origin of a joint beetween links to reference placement.
    """

    def GetResources(self):
        return {
            'Pixmap': 'set_cross_placement_fast_child_to_parent.svg',
            'MenuText': tr('Set placement - of child kinematic branch'),
            'Accel': 'P, C',
            'ToolTip': tr(
                'Set the Origin of a joint beetween links to reference placement.\nIt moves child kinematic branch.\n'
                '\n'
                'Select (with Ctlr): \n'
                '    1) subelement (face, edge, vertex, LCS) of body (of Real) of robot link (first reference)\n'
                '    2) subelement (face, edge, vertex, LCS) of body (of Real) of robot link (second reference)\n'
                '\n'
                'Robot links must be near to each other in chain (parent, child) and have joint between.\n'
                '\n'
                'This will connect 2 links (child to parent) in reference places.\n'
                'Joint Origin will be moved to connection position.\n'
                '\n'
                'If selected subelement (face, edge, vertex) will be used temporary LCS underhood.\n'
                '\n'
                'This tool for moving kinematic branch root joint to reference placement\n',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        set_placement_fast(child_branch_to_parent_tree = True)


fcgui.addCommand('SetCROSSPlacementFastChildToParent', _SetCROSSPlacementFastChildToParentCommand())
