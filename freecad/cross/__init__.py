"""Entry point of the CROSS workbench."""

import os

import FreeCAD as fc

from .ros.utils import add_ros_library_path
from .version import __version__
from .wb_globals import g_ros_distro
from .wb_globals import debugger

add_ros_library_path(g_ros_distro)

# Must be imported after the call to `add_ros_library_path`.
from .ros.utils import is_ros_found  # noqa: E402.

if is_ros_found():
    fc.addImportType('URDF files (*.urdf *.xacro)', 'freecad.cross.import_urdf')

# Initialize debug with debugpy.
if os.environ.get('DEBUG'):
    # how to use:
    # DEBUG=1 command_to_run_freecad
    # Use Ubuntu or install manually debugpy module FreeCAD python and FreeCAD python version to OS.
    # turn on Debugger in VSCODE and add breakpoints to code
    # Cf. https://github.com/FreeCAD/FreeCAD-macros/wiki/Debugging-macros-in-Visual-Studio-Code.

    debugger()
