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
from ..wb_utils import is_link
from ..wb_utils import set_placement_by_orienteer
from ..wb_utils import move_placement
from ..wb_utils import make_lcs_at_link_body


# Stubs and type hints.
from ..joint import Joint
from ..link import Link
DO = fc.DocumentObject
CrossLink = Link
CrossJoint = Joint
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


class _NewLCSAtRobotLinkBodyCommand:
    """Command to create LCS (Local Coordinate System) at face of body of link.
    """

    def GetResources(self):
        return {'Pixmap': 'set_new_lcs_at_robot_link_body.svg',
                'MenuText': tr('New LCS at robot link body'),
                'Accel': 'L, B',
                'ToolTip': tr('Make LCS at robot link body (Real) subelement (face, edge, vertex).\n'
                              '\n'
                              'Select: face or edge or vertex of body of robot link \n'
                              '\n'
                              'LCS can be used with Set Placement tools. Be free with LCS params correction.\n'
                              'By default LCS will use InertialCS MapMode \n'
                              'and Translate MapMode for vertex and Concentric for curve and circle.'
                              )}

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        doc = fc.activeDocument()
        selection_ok = False
        try:
            orienteer1, = validate_types(
                fcgui.Selection.getSelection(),
                ['Any'])
            selection_ok = True
        except RuntimeError:
            pass

        if not selection_ok:
            message('Select: face or edge or vertex of body of robot link.\n', gui=True)
            return

        
        link1 = get_parent_link_of_obj(orienteer1)

        if link1 == None:
            message('Can not get parent robot link of first selected object', gui=True)
            return      
        
        sel = fcgui.Selection.getSelectionEx()
        orienteer1_sub_obj = sel[0]
        if not is_link(orienteer1) and not is_joint(orienteer1):
            orienteer1 = orienteer1_sub_obj

        doc.openTransaction(tr("Make LCS at link body"))
        lcs, body_lcs_wrapper, lcs_placement = make_lcs_at_link_body(
            orienteer1,
            delete_created_objects = False,
            deactivate_after_map_mode = False,
        )
        doc.commitTransaction()

        doc.recompute()


fcgui.addCommand('NewLCSAtRobotLinkBody', _NewLCSAtRobotLinkBodyCommand())
