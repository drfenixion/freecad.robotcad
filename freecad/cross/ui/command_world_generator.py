import FreeCAD as fc

import FreeCADGui as fcgui
try:
    from PySide.QtWidgets import QApplication
except:
    from PySide2.QtWidgets import QApplication

from ..gui_utils import tr
import sys
from ..wb_utils import DYNAMIC_WORLD_GENERATOR_MODULE_PATH, DYNAMIC_WORLD_GENERATOR_REPO_PATH, git_init_submodules


class _WorldGeneratorCommand:
    """The command definition to create a custom world."""

    def GetResources(self):
        return {
            'Pixmap': 'world_generator.svg',
            'MenuText': tr('Create custom world'),
            'Accel': 'N, W',
            'ToolTip': tr('Create custom world with dynamic and static obstacles.' \
            '\n\nDocs at https://github.com/drfenixion/Dynamic_World_Generator'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):

        def world_generator_run_wrapper():
            # add module path to sys.path for working local path imports in module
            DYNAMIC_WORLD_GENERATOR_MODULE_PATH_str = str(DYNAMIC_WORLD_GENERATOR_MODULE_PATH)
            if DYNAMIC_WORLD_GENERATOR_MODULE_PATH_str not in sys.path:
                sys.path.insert(0, DYNAMIC_WORLD_GENERATOR_MODULE_PATH_str)
            
            from modules.Dynamic_World_Generator.code.dwg_wizard import run as world_generator_run
            from modules.Dynamic_World_Generator.code.dwg_wizard import dwg_wizard_close_emitter

            def close_callback(close_emitter_instance):
                QApplication.instance().aboutToQuit.connect(close_emitter_instance.on_app_quit)

            close_emitter = dwg_wizard_close_emitter(close_callback)

            world_generator_run(close_emitter)

        git_init_submodules(
            submodule_repo_path = DYNAMIC_WORLD_GENERATOR_REPO_PATH,
            callback = world_generator_run_wrapper
        )          


fcgui.addCommand('WorldGenerator', _WorldGeneratorCommand())
