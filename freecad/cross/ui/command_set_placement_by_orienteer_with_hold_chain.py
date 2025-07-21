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


class _SetCROSSPlacementByOrienteerWithHoldChainCommand:
    """Command to set the placement of Joint.

    Command to set the Origin by orienteer placement with hold downstream kinamatic chain

    """

    def GetResources(self):
        return {
            'Pixmap': 'set_cross_placement_by_orienteer_with_hold_chain.svg',
            'MenuText': tr('Set placement - by orienteer with hold chain'),
            'Accel': 'P, H',
            'ToolTip': tr(
                'Set the Origin of a joint with hold downstream kinematic chain.\n'
                '\n'
                'Select (with Ctlr): a CROSS::Joint, any (orienteer)\n'
                '\n'
                'This will set Origin with hold downstream kinematic chain (persist same position) \n'
                'including hold of Mounting Placement of link.\n'
                'Placement of orienteer will be placement of join origin\n'
                '\n'
                'LCS is convenient as reference because of configurable orientation.\n'
                'Use this when you want just move Origin to some placement and dont touch other downstream chain.\n',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        doc = fc.activeDocument()
        selection_ok = False
        selection_joint = False

        if not selection_ok:
            try:
                cross_joint, orienteer1 = validate_types(
                    fcgui.Selection.getSelection(),
                    ['Cross::Joint', 'Any'],
                )
                selection_ok = True
                selection_joint = True
            except RuntimeError:
                pass

        if not selection_ok:
            message(
                'Select (with Ctlr): a CROSS::Joint, any (orienteer) \n',
                gui=True,
            )
            return

        # for work with subelement
        sel = fcgui.Selection.getSelectionEx()
        orienteer1_sub_element = sel[1]
        if not is_lcs(orienteer1) and not is_joint(orienteer1) and not is_link(orienteer1):
            orienteer1 = orienteer1_sub_element

        if selection_joint:
            doc.openTransaction(tr("Set joint's origin"))
            set_placement_by_orienteer(doc, cross_joint, 'Origin', orienteer1, hold_downstream_chain = True)
            doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('SetCROSSPlacementByOrienteerWithHoldChain', _SetCROSSPlacementByOrienteerWithHoldChainCommand())
