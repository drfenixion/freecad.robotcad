import FreeCAD as fc

import FreeCADGui as fcgui

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
            'ToolTip': tr('Create custom world obstacles.'),
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        doc = fc.activeDocument()

        def world_generator_run_wrapper():
            # add module path to sys.path for working local path imports in module
            DYNAMIC_WORLD_GENERATOR_MODULE_PATH_str = str(DYNAMIC_WORLD_GENERATOR_MODULE_PATH)
            if DYNAMIC_WORLD_GENERATOR_MODULE_PATH_str not in sys.path:
                sys.path.insert(0, DYNAMIC_WORLD_GENERATOR_MODULE_PATH_str)
            
            from modules.Dynamic_World_Generator.code.dwg_wizard import run as world_generator_run
        
            world_generator_run()

        git_init_submodules(
            submodule_repo_path = DYNAMIC_WORLD_GENERATOR_REPO_PATH,
            callback = world_generator_run_wrapper
        )          

        doc.recompute()


fcgui.addCommand('WorldGenerator', _WorldGeneratorCommand())
