import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _NewWorkcellCommand:
    """The command definition to create a new Workcell object."""

    def GetResources(self):
        return {
            'Pixmap': 'workcell.svg',
            'MenuText': tr('Create a Workcell'),
            'Accel': 'N, W',
            'ToolTip': tr('Create a Workcell.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction('Create Workcell')
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.workcell_proxy')
        fcgui.doCommand("_workcell = freecad.cross.workcell_proxy.make_workcell('Workcell')")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_workcell.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewWorkcell', _NewWorkcellCommand())
