"""Global workbench configuration."""

from pathlib import Path
import os
import sys
import subprocess

from .ros.utils import add_ros_library_path
from .ros.utils import get_ros_workspace_from_env
from .ros.utils import get_ros_distro_from_env
from . import wb_constants
import addonmanager_utilities as utils

# Constants.
PREFS_CATEGORY = wb_constants.PREFS_CATEGORY  # Category in the preferences dialog.
PREF_VHACD_PATH = wb_constants.PREF_VHACD_PATH  # Path to the V-HACD executable.
PREF_OVERCROSS_TOKEN = wb_constants.PREF_OVERCROSS_TOKEN  # Auth token for external code generator

# Session-wide globals.
g_ros_distro = get_ros_distro_from_env()

add_ros_library_path(g_ros_distro)
# Must be imported after the call to `add_ros_library_path`.
from .ros.node import get_node_and_executor  # noqa: E402.
g_ros_node, g_ros_executor = get_node_and_executor()

# Can be changed in the GUI.
g_ros_workspace: Path = get_ros_workspace_from_env()


def get_python_name() -> str:
    ''' Get python name like python3.10 '''

    return 'python' + str(sys.version_info.major) + '.' + str(sys.version_info.minor)

def get_python_exe_path():
    ''' Get python exe path '''

    if hasattr(utils, "get_python_exe"):
        # For v0.21:
        python_exe = utils.get_python_exe()
    else:
        # For v0.22/v1.0:
        from freecad.utils import get_python_exe
        python_exe=get_python_exe()

    return python_exe


def pip_install(pkg_name, restart_freecad = True):
    '''Python package installer for AppImage builds. It can install python module inside AppImage'''

    python_exe = get_python_exe_path()
    print('python_exe: ', python_exe)
    vendor_path = utils.get_pip_target_directory()
    if not os.path.exists(vendor_path):
        os.makedirs(vendor_path)

    p = subprocess.Popen(
        [python_exe, "-m", "pip", "install", "--disable-pip-version-check", "--target", vendor_path, pkg_name],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )

    for line in iter(p.stdout.readline, b''):
        if line:
            print(line.decode("utf-8"), end="")
    print()
        
    for err in iter(p.stderr.readline, b''):
        if err:
            print(err.decode("utf-8"), end="")
    print()
    
    p.stdout.close()
    p.stderr.close()
    p.wait(timeout=180)

    if restart_freecad:
        utils.restart_freecad()   


def debugger():
    '''Add debugger. It can autoinstall debugpy'''

    def attach_debugger():
        python_exe=get_python_exe_path()

        import debugpy
        debugpy.configure(python=python_exe)
        debugpy.listen(("0.0.0.0", 5678))
        debugpy.trace_this_thread(True)
        debugpy.debug_this_thread()
        print('DEBUG attached.')
        pass

    try:
        print('DEBUG attaching...')
        attach_debugger()
    except:
        # needed restart freecad after install debugpy
        pip_install('debugpy', restart_freecad = True)
        print('Doing restart FreeCAD...')
