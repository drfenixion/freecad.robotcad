"""Global workbench configuration."""

from pathlib import Path

from .ros.utils import add_ros_library_path
from .ros.utils import get_ros_workspace_from_env
from .ros.utils import get_ros_distro_from_env

# Constants.
PREFS_CATEGORY = 'CROSS'  # Category in the preferences dialog.
PREF_VHACD_PATH = 'vhacd_path'  # Path to the V-HACD executable.

# Session-wide globals.
g_ros_distro = get_ros_distro_from_env()

add_ros_library_path(g_ros_distro)
# Must be imported after the call to `add_ros_library_path`.
from .ros.node import get_node_and_executor  # noqa: E402.
g_ros_node, g_ros_executor = get_node_and_executor()

# Can be changed in the GUI.
g_ros_workspace: Path = get_ros_workspace_from_env()
