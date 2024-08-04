import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _NewLinkCommand:
    """The command definition to create a new Link object."""

    def GetResources(self):
        return {
            'Pixmap': 'link.svg',
            'MenuText': tr('New Link'),
            'Accel': 'N, L',
            'ToolTip': tr('Create a Link container.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Create Link'))
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.link_proxy')
        fcgui.doCommand("_link = freecad.cross.link_proxy.make_link('Link')")
        fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_link.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewLink', _NewLinkCommand())
