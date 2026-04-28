from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecad_utils import is_lcs
from ..gui_utils import tr
from ..link_proxy import explode_link
from ..wb_utils import is_robot
from ..wb_utils import is_link, get_links


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
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        from ..ui.explode_links_dialog import ExplodeLinksDialog

        doc = fc.activeDocument()
        selection = fcgui.Selection.getSelection()

        if len(selection) and is_robot(selection[0]):
            robot = selection[0]
            links = robot.Proxy.get_links()
        else:
            # Filter selection to only include Cross::Link objects
            links = [obj for obj in selection if is_link(obj)]

        if not links:
            message('No links selected.')
            return

        # Show the dialog with the slider
        dialog = ExplodeLinksDialog(links)
        dialog.exec_()


fcgui.addCommand('ExplodeLinks', _ExplodeLinksCommand())
