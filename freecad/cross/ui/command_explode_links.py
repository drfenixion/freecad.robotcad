from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecad_utils import is_lcs
from ..gui_utils import tr
from ..link_proxy import explode_link


# Stubs and type hints.
from ..joint import Joint
from ..link import Link
DO = fc.DocumentObject
CrossLink = Link
CrossJoint = Joint
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


class _ExplodeLinksCommand:
    """Command to move links (explode view) to see hiden faces.
    """

    def GetResources(self):
        return {'Pixmap': 'explode_links.svg',
                'MenuText': tr('Move links (explode view) to see hiden faces.'),
                'Accel': 'E, L',
                'ToolTip': tr('Move links (explode view) to see hiden faces.\n'
                              '\n'
                              'Select: link(s) or subobject of link \n'
                              '\n'
                              'Link(s) must be in joint(s) for visual effect.\n'
                              '\n'
                              'Link will be moved by changing MountedPlacement.\n'
                              'This useful for see hidden faces of link subobject\n'
                              'for select contact faces for placement tools.\n'
                              )}

    def IsActive(self):
        return True

    def Activated(self):
        doc = fc.activeDocument()

        selection = fcgui.Selection.getSelection()

        doc.openTransaction(tr("Explode links"))
        for i in range(len(selection)):
            explode_link(selection[i], i)
        doc.commitTransaction()

        doc.recompute()


fcgui.addCommand('ExplodeLinks', _ExplodeLinksCommand())
