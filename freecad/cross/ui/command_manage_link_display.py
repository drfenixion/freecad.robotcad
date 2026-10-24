from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..gui_utils import tr
from ..wb_utils import is_link, is_robot


class _ManageLinkDisplayCommand:
    """Command to show a dialog for managing Collision/Visual/Real display of links."""

    def GetResources(self):
        return {'Pixmap': 'manage_link_display.svg',
                'MenuText': tr('Manage link display'),
                'Accel': 'M, L',
                'ToolTip': tr('Manage link display: toggle Collision, Visual, Real visibility.\n'
                              '\n'
                              'Select: Robot or Robot Link(s)\n'
                              '\n'
                              'Shows a dialog with checkboxes to control visibility\n'
                              'of Collision, Visual, and Real elements.\n'
                              '"Set Placement Mode" hides Collision and Visual,\n'
                              'showing only Real for placement operations.\n'
                              ),
                }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        from ..ui.manage_link_display_dialog import (
            ManageLinkDisplayDialog,
            _get_links_from_selection,
        )

        selection = fcgui.Selection.getSelection()
        links = _get_links_from_selection(selection)

        if not links:
            message(
                tr(
                    'No Robot or Robot Link selected.\n'
                    'Select at least one Robot or Link.',
                ),
            )
            return

        # Show the dialog (non-modal, allows FreeCAD interaction).
        dialog = ManageLinkDisplayDialog(links)
        dialog.show()


fcgui.addCommand('ManageLinkDisplay', _ManageLinkDisplayCommand())
