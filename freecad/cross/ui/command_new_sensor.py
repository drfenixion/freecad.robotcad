import FreeCADGui as fcgui

from ..gui_utils import tr
from ..wb_utils import is_robot_selected, is_link_selected, is_joint_selected
from ..wb_utils import git_init_submodules
from ..wb_utils import SDFORMAT_PATH
from .dynamic_ui.sensors_selector import SensorsSelectorModalClass


class _NewSensorCommand:
    """The command definition to create a new Sensor object."""

    def GetResources(self):
        return {
            'Pixmap': 'sensor.svg',
            'MenuText': tr('Create a Sensor'),
            'Accel': 'N, C',
            'ToolTip': tr(
                'Create a Sensor.\n'
                '\n'
                'Sensors are needed to receive data about environment.\n'
                '\n'
                'See https://gazebosim.org/docs/latest/sensors/ documentation.',
            ),
        }

    def IsActive(self):
        return is_robot_selected() or is_link_selected() or is_joint_selected()

    def Activated(self):
        git_init_submodules(update_if_dir_is_empty = SDFORMAT_PATH)
        form = SensorsSelectorModalClass()
        form.exec_()


fcgui.addCommand('NewSensor', _NewSensorCommand())
