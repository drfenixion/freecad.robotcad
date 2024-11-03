import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _NewObserverCommand:
    """The command definition to create a new CROSS::Observer object."""

    def GetResources(self):
        return {
            'Pixmap': 'observer.svg',
            'MenuText': tr('New Observer'),
            'Accel': 'N, O',
            'ToolTip': tr('Create an Observer.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction('Create Observer')
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.observer_proxy')
        fcgui.doCommand("_observer = freecad.cross.observer_proxy.make_observer('Observer')")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_observer.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewObserver', _NewObserverCommand())
