
import FreeCAD as fc
import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!

from ..assembly_from_urdf import assembly_from_urdf
from ..freecad_utils import warn
from ..gui_utils import tr
from ..ros.utils import is_ros_found
try:
    from ..urdf_loader import UrdfLoader
except ImportError as e:
    # TODO: Warn the user more nicely.
    warn(str(e), gui=False)


class _AssemblyFromUrdfCommand:
    def GetResources(self):
        return {
            'Pixmap': 'assembly_from_urdf.svg',
            'MenuText': tr('Create an assembly from a URDF or xacro file'),
            'ToolTip': tr('Create an assembly from a URDF or xacro file'),
        }

    def Activated(self):
        doc = fc.activeDocument()
        dialog = QtGui.QFileDialog(
            fcgui.getMainWindow(),
            'Select URDF/xacro file to import part from',
        )
        # set option "DontUseNativeDialog"=True, as native Filedialog shows
        # misbehavior on Unbuntu 18.04. It works case sensitively, what isn't wanted
        dialog.setNameFilter('Supported Formats *.urdf *.xacro;;All files (*.*)')
        if dialog.exec_():
            if not doc:
                doc = fc.newDocument()
            filename = str(dialog.selectedFiles()[0])
            urdf_robot = UrdfLoader.load_from_file(filename)
            doc.openTransaction(tr('Assembly from URDF'))
            assembly_from_urdf(doc, urdf_robot)
            doc.commitTransaction()
            doc.recompute()
            fcgui.SendMsgToActiveView('ViewFit')

    def IsActive(self):
        return is_ros_found()


fcgui.addCommand('AssemblyFromUrdf', _AssemblyFromUrdfCommand())
