from __future__ import annotations

from pathlib import Path

import FreeCADGui as fcgui

from ..freecad_utils import warn
from ..gui_utils import tr
from ..wb_gui_utils import WbSettingsGetter
from ..wb_utils import set_workbench_param
from .. import wb_globals


def _warn_if_not_workspace(path: str, gui: bool = True):
    p = Path(path)
    if not (p / 'install/setup.bash').exists():
        warn(f'{path} does not appear to be a valid ROS workspace', gui)


class _WbSettingsCommand:
    """The command definition to set up the workbench."""

    def GetResources(self):
        return {
            'Pixmap': 'wb_settings.svg',
            'MenuText': tr('Workbench settings'),
            'Accel': 'W, S',
            'ToolTip': tr('Workbench settings'),
        }

    def IsActive(self):
        return True

    def Activated(self):
        settings_getter = WbSettingsGetter(wb_globals.g_ros_workspace)
        if settings_getter.get_settings():
            wb_globals.g_ros_workspace = settings_getter.ros_workspace
            set_workbench_param(wb_globals.PREF_VHACD_PATH, str(settings_getter.vhacd_path))


fcgui.addCommand('WbSettings', _WbSettingsCommand())
