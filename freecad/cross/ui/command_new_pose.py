import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _NewPoseCommand:
    """The command definition to create a new CROSS::Pose object."""

    def GetResources(self):
        return {
            'Pixmap': 'pose.svg',
            'Accel': 'N, P',
            'ToolTip': tr('Create a Pose.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction('Create Pose')
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.pose_proxy')
        fcgui.doCommand("_pose = freecad.cross.pose_proxy.make_pose('Pose')")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_pose.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewPose', _NewPoseCommand())
