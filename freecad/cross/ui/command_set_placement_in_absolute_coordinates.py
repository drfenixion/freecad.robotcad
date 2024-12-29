from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..wb_utils import move_placement
from ..wb_utils import is_joint
from ..wb_utils import is_link
from ..gui_utils import tr
from ..freecad_utils import is_lcs


# Stubs and type hints.
from ..joint import Joint
from ..link import Link
DO = fc.DocumentObject
CrossLink = Link
CrossJoint = Joint
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


class _SetCROSSPlacementInAbsoluteCoordinatesCommand:
    """Command to set the placement of a Link or a Joint.

    Command to set the mounted placement of a Link or the Origin of a Joint in absolute coordinates
    (improved version of Set Placement).

    """

    def GetResources(self):
        return {
            'Pixmap': 'set_cross_placement_in_absolute_coordinates.svg',
            'MenuText': tr('Set placement - as system'),
            'Accel': 'P, S',
            'ToolTip': tr(
                'Set the Mounted Placement of a link or the Origin of a joint.\n'
                '\n'
                'Select (with Ctlr) either:\n'
                '  a) a CROSS::Link, any (first reference), any (second reference)\n'
                '  b) a CROSS::Joint, any (first reference), any (second reference)\n'
                '\n'
                'This will move first reference to position of second reference\n'
                'and binded system (first reference + Link or Joint) will moved respectively.\n'
                'LCS is convenient as reference because of configurable orientation.',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        doc = fc.activeDocument()
        selection_ok = False
        selection_link = False
        selection_joint = False
        try:
            cross_link, orienteer1, orienteer2 = validate_types(
                fcgui.Selection.getSelection(),
                ['Cross::Link', 'Any', 'Any'],
            )
            selection_ok = True
            selection_link = True
        except RuntimeError:
            pass

        if not selection_ok:
            try:
                cross_joint, orienteer1, orienteer2 = validate_types(
                    fcgui.Selection.getSelection(),
                    ['Cross::Joint', 'Any', 'Any'],
                )
                selection_ok = True
                selection_joint = True
            except RuntimeError:
                pass

        if not selection_ok:
            message(
                'Select either\n'
                '  a) a CROSS::Link, any (first orienteer), any (second orienteer) \n'
                '  b) a CROSS::Joint, any (first orienteer), any (second orienteer).\n',
                gui=True,
            )
            return

        # for work with subelement
        sel = fcgui.Selection.getSelectionEx()
        orienteer1_sub_element = sel[1]
        orienteer2_sub_element = sel[2]

        if not is_lcs(orienteer1) and not is_joint(orienteer1) and not is_link(orienteer1):
            orienteer1 = orienteer1_sub_element
        if not is_lcs(orienteer2) and not is_joint(orienteer2) and not is_link(orienteer2):
            orienteer2 = orienteer2_sub_element

        if selection_link:
            doc.openTransaction(tr("Set link's mounted placement"))
            move_placement(doc, cross_link, 'MountedPlacement', orienteer1, orienteer2)
            doc.commitTransaction()
        elif selection_joint:
            doc.openTransaction(tr("Set joint's origin"))
            move_placement(doc, cross_joint, 'Origin', orienteer1, orienteer2)
            doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('SetCROSSPlacementInAbsoluteCoordinates', _SetCROSSPlacementInAbsoluteCoordinatesCommand())
