from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecad_utils import is_lcs
from ..gui_utils import tr
from ..link_proxy import make_robot_link_filled


# Stubs and type hints.
from ..joint import Joint
from ..link import Link
DO = fc.DocumentObject
CrossLink = Link
CrossJoint = Joint
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


class _NewLinksFilledCommand:
    """Command to create new robot links with filled Visual and Real by selected objects.
    """

    def GetResources(self):
        return {'Pixmap': 'body_to_link.svg',
                'MenuText': tr('New links with filled Real, Visual by selected objects'),
                'Accel': 'L, B',
                'ToolTip': tr('Make new links with filled Real, Visual by selected objects.\n'
                              '\n'
                              'Select: object (part, body) or objects \n'
                              '\n'
                              'Will be created robot links with filled Real and Visual properties by selected objects.\n'
                              'Each link for each object.\n'
                              'Will be created part-wrapper for each object if object is not a part.'
                              )}

    def IsActive(self):
        return True

    def Activated(self):
        doc = fc.activeDocument()

        selection = fcgui.Selection.getSelection()

        doc.openTransaction(tr("Set joint origin"))
        for el in selection:
            make_robot_link_filled(el)
        doc.commitTransaction()

        doc.recompute()


fcgui.addCommand('NewLinksFilled', _NewLinksFilledCommand())
