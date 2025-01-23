import FreeCADGui as fcgui

from ..gui_utils import tr
from ..wb_utils import is_controller_selected, is_robot_selected
from ..wb_utils import git_init_submodules
from .dynamic_ui.controllers_selector import ControllersSelectorModalClass 
from ..freecad_utils import message


class _NewControllerCommand:
    """The command definition to create a new Controller or Broadcaster object."""

    def GetResources(self):
        return {
            'Pixmap': 'controller.svg',
            'MenuText': tr('Create a Controller or Broadcaster'),
            'Accel': 'N, C',
            'ToolTip': tr('Create a Controller or Broadcaster.\n'
                          '\n'
                          'Controllers are needed to control joints.\n'
                          'Broadcasters are needed for receive and transfer data from sensors.\n'
                          '\n'
                          'See ros2_controllers and ros2_control documentation.'),
        }

    def IsActive(self):
        return is_robot_selected() or is_controller_selected()

    def Activated(self):
        git_init_submodules()
        form = ControllersSelectorModalClass()
        form.exec_()


fcgui.addCommand('NewController', _NewControllerCommand())
