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
from .packages import add_packages_path, pip_install

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


# Python dependencies (urdf_parser_py, xacro, xacrodoc, mujoco,
# ament_index_python, xmltodict, collada, lxml, and optionally mcp) are
# declared in the addon's package.xml and are NO LONGER installed silently
# here. They are checked and installed through the interactive
# "Check and install dependencies" action, shown automatically when the
# workbench is activated and some package is missing. See
# freecad/cross/dependencies.py and freecad/cross/ui/dependencies_dialog.py.

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
    # Reported when the workbench is activated, not at FreeCAD start-up.
    from .deferred_messages import add_message
    add_message(str(e) + '. URDF/Xacro import support is limited.')
    imports_ok = False
