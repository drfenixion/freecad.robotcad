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
    """Command to move parent kinematic tree. Also moved parent join of parent link to reference placement.
    """

    def GetResources(self):
        return {
            'Pixmap': 'set_cross_placement_fast_parent_to_child.svg',
            'MenuText': tr('Set placement - move parent kinematic tree to child branch'),
            'Accel': 'P, P',
            'ToolTip': tr(
                'Move parent kinematic tree to child branch.\nAlso moved parent join of parent link to reference placement.\n'
                '\n'
                'Select (with Ctlr): \n'
                '    1) subelement (face, edge, vertex, LCS) of body (of Real) of robot link (first reference)\n'
                '    2) subelement (face, edge, vertex, LCS) of body (of Real) of robot link (second reference)\n'
                '\n'
                'Robot links must be near to each other in chain (parent, child) and have joint between.\n'
                '\n'
                'This will connect 2 links (child to parent) in reference places.\n'
                'Joint Origin and link Mounted Placement of parent link will be moved to connection position.\n'
                'Parent kinematic tree will be moved relatively. Child branch visually will be in same position\n'
                '\n'
                'If selected subelement (face, edge, vertex) will be used temporary LCS underhood.\n'
                '\n'
                'This tool for moving parent kinematic tree to reference placement.\n',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        set_placement_fast(parent_tree_to_child_branch=True)


fcgui.addCommand('SetCROSSPlacementFastParentToChild', _SetCROSSPlacementFastParentToChildCommand())
