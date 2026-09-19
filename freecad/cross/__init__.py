"""Entry point of the RobotCAD workbench."""

import os

try:
    # For v0.21:
    from addonmanager_utilities import get_python_exe
except (ModuleNotFoundError, ImportError, AttributeError):
    # For v0.22/v1.0:
    from freecad.utils import get_python_exe

# Shared pip-install helpers. Defined in a pure-Python module (no FreeCAD
# import at module level) so that the standalone stdio MCP bridge can reuse
# them without triggering this heavy workbench initialization.
from .packages import add_packages_path, check_install_package, pip_install

add_packages_path()

# Initialize debug with debugpy.
if os.environ.get('DEBUG'):
    print('DEBUG attaching...')
    # how to use:
    # DEBUG=1 command_to_run_freecad
    # turn on Debugger in VSCODE and add breakpoints to code
    # Cf. https://github.com/FreeCAD/FreeCAD-macros/wiki/Debugging-macros-in-Visual-Studio-Code.

    def attach_debugger():
        import debugpy
        debugpy.configure(python=get_python_exe())
        debugpy.listen(("0.0.0.0", 5678))
        # debugpy.wait_for_client()
        debugpy.trace_this_thread(True)
        debugpy.debug_this_thread()
        print('DEBUG attached.')

    try:
        attach_debugger()
    except:
        pip_install('debugpy')
        attach_debugger()


import FreeCAD as fc
from .ros.utils import add_ros_library_path
from .version import __version__  # noqa: F401
from .wb_globals import g_ros_distro


add_ros_library_path(g_ros_distro)


# pip installs
# should be after add_ros_library_path because ros package must be initialized firstly
check_install_package('urdf_parser_py')

# # Looks like Xacro pip ver is updated. Persist warning comment for some time. 
# Disabled Xacro auto pip install because of on pip too old version. Xacro should be installed from Conda or by Rosdep
check_install_package('xacro')

check_install_package('ament_index_python', 'ros-ament-index-python')
check_install_package('xmltodict')
check_install_package('collada', 'pycollada')
check_install_package('PyQt5')
check_install_package('lxml')

# MCP (Model Context Protocol) server for external LLM agents.
# The `mcp` package is intentionally NOT installed here (at workbench start):
# it is installed lazily, on the first use of the MCP tools (server start).
# See `freecad/cross/mcp/server.py` -> `_ensure_mcp_packages()`.
# Note: the code targets mcp 2.x (MCPServer). If a 1.x version is already
# installed, upgrade it: pip install -U mcp

# Must be imported after the call to `add_ros_library_path`.
from freecad.cross.freecad_utils import warn
try:
    import xacro
    from .ros.utils import is_ros_found  # noqa: E402.
    if is_ros_found():
        fc.addImportType('URDF files (*.urdf *.xacro)', 'freecad.cross.import_urdf')
    else:
        fc.addImportType('URDF files (*.urdf)', 'freecad.cross.import_urdf')
        warn('ROS2 was not detected. Import of Xacro files is disabled. URDF import is posible.', gui=False)
    imports_ok = True
except Exception as e:
    # TODO: Warn the user more nicely.
    warn(str(e) + '. Models library tool is disabled.', gui=False)
    imports_ok = False
