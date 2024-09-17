from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecad_utils import is_lcs
from ..gui_utils import tr
from ..wb_utils import set_placement_by_orienteer
from ..wb_utils import move_placement
from ..wb_utils import is_joint
from ..wb_utils import get_chain
from ..wb_utils import is_link


# Stubs and type hints.
from ..joint import Joint
from ..link import Link
DO = fc.DocumentObject
CrossLink = Link
CrossJoint = Joint
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


class _SetCROSSPlacementByOrienteerCommand:
    """Command to set the placement of a Link or a Joint.

    Command to set the mounted placement of a Link or the Origin by orienteer placement

    """

    def GetResources(self):
        return {'Pixmap': 'set_cross_placement_by_orienteer.svg',
                'MenuText': tr('Set placement - by orienteer'),
                'Accel': 'P, O',
                'ToolTip': tr('Set the Mounted Placement of a link or the Origin of a joint.\n'
                              '\n'
                              'Select (with Ctlr) either:\n'
                              '  a) a CROSS::Link, any (face, edge, vertex) at suboject of this link or LCS at same subobject (orienteer) \n'
                              '  b) a CROSS::Joint, any (orienteer) \n'
                              '\n'
                              'This will set Mounted Placement if select link first. \n'
                              'Placement of orienteer will be placement of conjunction link to joint.\n'
                              '\n'
                              'This will set Origin if selected joint first.\n'
                              'Placement of orienteer will be placement of join origin\n'
                              '\n'
                              'LCS is convenient as orienteers because of configurable orientation.\n'
                              'Use this when you want just move Origin or Mounted placement to some placement.\n'
                              )}

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        doc = fc.activeDocument()
        selection_ok = False
        selection_link = False
        selection_joint = False
        try:
            cross_link, orienteer1 = validate_types(
                fcgui.Selection.getSelection(),
                ['Cross::Link', 'Any'])
            selection_ok = True
            selection_link = True
        except RuntimeError:
            pass

        if not selection_ok:
            try:
                cross_joint, orienteer1 = validate_types(
                    fcgui.Selection.getSelection(),
                    ['Cross::Joint', 'Any'])
                selection_ok = True
                selection_joint = True
            except RuntimeError:
                pass

        if not selection_ok:
            message('Select either\n'
                    '  a) a CROSS::Link, any (face, edge, vertex) at body of this link (orienteer) \n'
                    '  b) a CROSS::Joint, any (orienteer) \n',
                    gui=True)
            return
        
        # for work with subelement
        sel = fcgui.Selection.getSelectionEx()
        orienteer1_sub_element = sel[1]
        if not is_lcs(orienteer1) and not is_joint(orienteer1) and not is_link(orienteer1):
            orienteer1 = orienteer1_sub_element

        if selection_link:
            doc.openTransaction(tr("Set link's mounted placement"))
            # set_placement_by_orienteer() does not work for MountedPlacement
            # because link conjuction place in many cases not at origin (zero coordinates) of link
            # and therefore used move_placement() with joint as second orienteer
            chain = get_chain(cross_link)
            joint = chain[-2]
            if not is_joint(joint):
                message('Can not get joint of link. Be sure your link have join.', gui=True)
                return
            move_placement(doc, cross_link, 'MountedPlacement', orienteer1, joint)
            doc.commitTransaction()
        elif selection_joint:
            doc.openTransaction(tr("Set joint's origin"))
            set_placement_by_orienteer(doc, cross_joint, 'Origin', orienteer1)
            doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('SetCROSSPlacementByOrienteer', _SetCROSSPlacementByOrienteerCommand())
