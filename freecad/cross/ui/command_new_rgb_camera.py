import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _NewRgbCameraCommand:
    """The command definition to create a new CROSS::RgbCamera object."""

    def GetResources(self):
        return {
            'Pixmap': 'camera-photo-symbolic.svg',
            'MenuText': tr('New RGB Camera'),
            'Accel': 'N, C',
            'ToolTip': tr('Create an RGB Camera.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction('Create RGB Camera')
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.rgb_camera_proxy')
        fcgui.doCommand("_camera = freecad.cross.rgb_camera_proxy.make_rgb_camera('Camera')")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_rgb_camera.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewRgbCamera', _NewRgbCameraCommand())
