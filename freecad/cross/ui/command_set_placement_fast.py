from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecad_utils import is_lcs
from ..gui_utils import tr
from ..wb_utils import get_parent_link_of_obj
from ..wb_utils import get_chain
from ..wb_utils import is_joint
from ..wb_utils import set_placement_by_orienteer
from ..wb_utils import move_placement


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
                'Set the Origin of a joint and Mounted Placement of link and make LCS at reference places.\n'
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
                'At both reference places will be created LCS (if selected subelement).\n'
                'Be free to fix the default LCS orientation if it does not suit you.\n'
                'It is posible to use other Placements tools with the LCS.\n'
                'Also you can remove LCS, it only helpers for set placement.\n'
                'By default LCS will use InertialCS MapMode \n'
                'and Translate MapMode for vertex and Concentric for curve and circle.\n'
                '\n'
                'Dont use this for moving chain of joints because it also set MountedPlacement\n'
                'that is may not be desirable in this case.\n',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        doc = fc.activeDocument()
        selection_ok = False
        try:
            orienteer1, orienteer2 = validate_types(
                fcgui.Selection.getSelection(),
                ['Any', 'Any'],
            )
            selection_ok = True
        except RuntimeError:
            pass

        if not selection_ok:
            message(
                'Select: face or edge or vertex of body of robot link, face or edge or vertex of body of robot link.'
                'Robot links must be near to each other (parent, child) and have joint between.\n', gui=True,
            )
            return

        link1 = get_parent_link_of_obj(orienteer1)
        link2 = get_parent_link_of_obj(orienteer2)

        if link1 == None:
            message('Can not get parent robot link of first selected object', gui=True)
            return

        if link2 == None:
            message('Can not get parent robot link of second selected object', gui=True)
            return

        sel = fcgui.Selection.getSelectionEx()
        orienteer1_sub_obj = sel[0]
        orienteer2_sub_obj = sel[1]

        chain1 = get_chain(link1)
        chain2 = get_chain(link2)
        chain1_len = len(chain1)
        chain2_len = len(chain2)
        parent_link = None # same link as parent in both orienteers

        if chain1_len > chain2_len:
            parent_link = link2
            child_link = link1
            chain = chain1

            if is_lcs(orienteer2):
                parent_orienteer = orienteer2
            else:
                parent_orienteer = orienteer2_sub_obj

            if is_lcs(orienteer1):
                child_orienteer = orienteer1
            else:
                child_orienteer = orienteer1_sub_obj

        elif chain1_len < chain2_len:
            parent_link = link1
            child_link = link2
            chain = chain2
            parent_orienteer = orienteer1
            child_orienteer = orienteer2

            if is_lcs(orienteer1):
                parent_orienteer = orienteer1
            else:
                parent_orienteer = orienteer1_sub_obj

            if is_lcs(orienteer2):
                child_orienteer = orienteer2
            else:
                child_orienteer = orienteer2_sub_obj
        elif chain1_len == chain2_len == 1:
            message('Links must be connected by joints first.', gui=True)
            return

        if not parent_link:
            message(
                'Tool does not work with orienteers in same robot link.'
                ' Must be one orienteer in parent link and one in child link', gui=True,
            )
            return

        joint = chain[-2]
        if not is_joint(joint):
            message('Can not get joint between parent links of selected objects', gui=True)
            return

        doc.openTransaction(tr("Set placement - fast"))
        set_placement_by_orienteer(doc, joint, 'Origin', parent_orienteer)
        move_placement(doc, child_link, 'MountedPlacement', child_orienteer, parent_orienteer)
        doc.commitTransaction()


fcgui.addCommand('SetCROSSPlacementFast', _SetCROSSPlacementFastCommand())
