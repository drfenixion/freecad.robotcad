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


class _SetCROSSPlacementFastParentToChildCommand:
    """Command to move parent kinematic tree to child branch.
    """

    def GetResources(self):
        return {
            'Pixmap': 'set_cross_placement_fast_parent_to_child.svg',
            'MenuText': tr('Set placement - move parent kinematic tree to child branch'),
            'Accel': 'P, P',
            'ToolTip': tr(
                'Move parent kinematic tree to child branch and connect references.\n'
                '\n'
                'Select (with Ctlr): \n'
                '    1) subelement (face, edge, vertex, LCS) of body (of Real) of robot link (first reference)\n'
                '    2) subelement (face, edge, vertex, LCS) of body (of Real) of robot link (second reference)\n'
                '\n'
                'Robot links must be near to each other in chain (parent, child) and have joint between.\n'
                '\n'
                'This will connect parent kinematic tree and child branch in reference places.\n'
                'Parent kinematic tree will be moved relatively. Child branch visually will be in same position\n'
                '\n'
                'If selected subelement (face, edge, vertex) will be used temporary LCS underhood.\n'
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        set_placement_fast(parent_tree_to_child_branch=True)


fcgui.addCommand('SetCROSSPlacementFastParentToChild', _SetCROSSPlacementFastParentToChildCommand())
