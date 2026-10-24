import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _NewUltrasoundCommand:
    """The command definition to create a new CROSS::Ultrasound object."""

    def GetResources(self):
        return {
            'Pixmap': 'ultrasound.svg',
            'MenuText': tr('New ultrasound'),
            'Accel': 'N, U',
            'ToolTip': tr('Create an ultrasound distance measuring sensor.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction('Create Ultrasound')
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.ultrasound_proxy')
        fcgui.doCommand("_lidar = freecad.cross.ultrasound_proxy.make_ultrasound('Ultrasound')")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_lidar.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewUltrasound', _NewUltrasoundCommand())
