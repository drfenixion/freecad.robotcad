import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr


class _NewTrajectoryCommand:
    """The command definition to create a new CROSS::Trajectory object."""

    def GetResources(self):
        return {
            'Pixmap': 'trajectory.svg',
            'Accel': 'N, T',
            'ToolTip': tr('Create a Trajectory.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction('Create Trajectory')
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.trajectory_proxy')
        fcgui.doCommand("_trajectory = freecad.cross.trajectory_proxy.make_trajectory('Trajectory')")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_trajectory.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewTrajectory', _NewTrajectoryCommand())
