import FreeCADGui as fcgui
import FreeCAD as fc

from ..gui_utils import tr
from .dynamic_ui.models_library import ModelsLibraryModalClass


class _OpenModelsLibraryCommand:
    """The command definition to open models library."""

    def GetResources(self):
        return {
            'Pixmap': 'models_library.svg',
            'MenuText': tr('Open models library'),
            'Accel': 'O, L',
            'ToolTip': tr(
                'library with models.',
            ),
        }

    def IsActive(self):
        return True

    def Activated(self):
        doc = fc.activeDocument()
        if not doc:
            doc = fc.newDocument()
        form = ModelsLibraryModalClass()
        form.exec_()


fcgui.addCommand('OpenModelsLibrary', _OpenModelsLibraryCommand())
