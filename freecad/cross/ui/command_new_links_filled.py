from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecad_utils import is_lcs
from ..gui_utils import tr
from ..link_proxy import get_selected_objects_of_all_documents
from ..link_proxy import make_robot_links_filled


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
                'Accel': 'L, F',
                'ToolTip': tr('New links with filled Real, Visual by selected objects.\n'
                              '\n'
                              'Select: object (part, body) or objects \n'
                              '\n'
                              'Will be created robot links with filled Real and Visual properties by selected objects.\n'
                              'Each link for each object.\n'
                              'Will be created part-wrapper for each object if object is not a part.\n'
                              '\n'
                              'Objects of other (external) documents can be selected too\n'
                              '(e.g. in the tree of the external document).\n'
                              'The part-wrapper (in the active document) links directly to them.'
                              )}

    def IsActive(self):
        # Selection of all documents: objects of external documents are allowed.
        return bool(fc.activeDocument()) and bool(get_selected_objects_of_all_documents())

    def Activated(self):
        doc = fc.activeDocument()

        doc.openTransaction(tr("New filled robot links by selected objects"))
        make_robot_links_filled()
        doc.commitTransaction()

        doc.recompute()


fcgui.addCommand('NewLinksFilled', _NewLinksFilledCommand())
