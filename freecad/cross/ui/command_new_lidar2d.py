import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _NewLidar2dCommand:
    """The command definition to create a new CROSS::Lidar2d object."""

    def GetResources(self):
        return {
            'Pixmap': 'lidar2d.svg',
            'MenuText': tr('New 2D LiDAR'),
            'Accel': 'N, I',
            'ToolTip': tr('Create a 2D LiDAR.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction('Create 2D LiDAR')
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.lidar2d_proxy')
        fcgui.doCommand("_lidar = freecad.cross.lidar2d_proxy.make_lidar2d('Lidar')")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_lidar.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewLidar2d', _NewLidar2dCommand())
