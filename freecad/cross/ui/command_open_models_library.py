import FreeCADGui as fcgui
import FreeCAD as fc

from ..gui_utils import tr
from ..freecad_utils import warn
try:
    from .dynamic_ui.models_library import ModelsLibraryModalClass
    imports_ok = True
except ImportError as e:
    # TODO: Warn the user more nicely.
    warn(str(e) + '. Models library tool is disabled.', gui=False)
    imports_ok = False


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
        return imports_ok

    def Activated(self):
        doc = fc.activeDocument()
        if not doc:
            doc = fc.newDocument()
        form = ModelsLibraryModalClass()
        form.exec_()


fcgui.addCommand('OpenModelsLibrary', _OpenModelsLibraryCommand())
