"""Command to add a vacuum suction gripper to the selected link."""

import FreeCAD as fc
import FreeCADGui as fcgui

from ..gui_utils import tr
from ..wb_utils import is_link_selected
from ..vacuum_gripper_proxy import add_vacuum_gripper_to_link


class _NewVacuumGripperCommand:
    """The command definition to create a new VacuumGripper object."""

    def GetResources(self):
        return {
            'Pixmap': 'vacuum_gripper.svg',
            'MenuText': tr('Add Vacuum Gripper'),
            'Accel': 'N, G',
            'ToolTip': tr(
                'Add a vacuum suction gripper to the selected link.\n'
                '\n'
                'The gripper plugin xml will be added to SDF world with it options.\n'
                '\n'
                'The gripper uses the Gazebo-Vacuum-Gripper-Plugin.\n'
                'See documentation - https://github.com/drfenixion/Gazebo-Vacuum-Gripper-Plugin\n'
                '\n'
                'You should manually clone repo, build and add it to GZ_SIM_SYSTEM_PLUGIN_PATH:\n'
                'export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH::/path/to/your/Gazebo-Vacuum-Gripper-Plugin/build\n'
                'or just use External Code Generator (it will automatically do it all in Docker).',
            ),
        }

    def IsActive(self):
        return is_link_selected()

    def Activated(self):
        sel = fcgui.Selection.getSelection()
        if not sel:
            return
        link = sel[0]
        add_vacuum_gripper_to_link(link)


fcgui.addCommand('NewVacuumGripper', _NewVacuumGripperCommand())
