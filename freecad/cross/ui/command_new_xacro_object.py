import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _NewXacroObjectCommand:
    """The command definition to create a new xacro object."""

    def GetResources(self):
        return {
            'Pixmap': 'xacro.svg',
            'MenuText': tr('New xacro object'),
            'Accel': 'N, X',
            'ToolTip': tr('Create a robot from a xacro macro.'),
        }

    def IsActive(self):
        return (fc.activeDocument() is not None)

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Create xacro object'))
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.xacro_object_proxy')
        fcgui.doCommand("_xacro = freecad.cross.xacro_object_proxy.make_xacro_object('Xacro')")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_xacro.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewXacroObject', _NewXacroObjectCommand())
