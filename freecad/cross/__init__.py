"""Entry point of the RobotCAD workbench."""

from pathlib import Path
import subprocess
import os
import sys

try:
    # For v0.21:
    from addonmanager_utilities import get_python_exe
except (ModuleNotFoundError, ImportError, AttributeError):
    # For v0.22/v1.0:
    from freecad.utils import get_python_exe


def pip_install(pkg_name):
    '''Python package installer for AppImage builds. It installs python module inside AppImage'''
    # should be in __init__.py to eliminate cyrcle dependencies of installed modules

    import site

    python_exe = get_python_exe()
    print('python_exe: ', python_exe)
    vendor_path = site.getsitepackages()[0]
    if not os.path.exists(vendor_path):
        os.makedirs(vendor_path)

    p = subprocess.Popen(
        [python_exe, "-m", "pip", "install", "--disable-pip-version-check", "--target", vendor_path, pkg_name],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
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

    # dynamically add module to sys.path
    major = sys.version_info.major
    minor = sys.version_info.minor

    pythonPackagesPath = f'~/.local/share/FreeCAD/AdditionalPythonPackages/py{major}{minor}'
    path = Path(pythonPackagesPath).expanduser().absolute()
    if path.exists() and (str(path) not in sys.path):
        sys.path.append(str(path))


# Initialize debug with debugpy.
if os.environ.get('DEBUG'):
    print('DEBUG attaching...')
    # how to use:
    # DEBUG=1 command_to_run_freecad
    # Use Ubuntu or install manually debugpy module FreeCAD python and FreeCAD python version to OS.
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
try:
    import urdf_parser_py
except (ModuleNotFoundError, ImportError):
    pip_install('urdf_parser_py')

# # Disabled Xacro auto pip install because of on pip too old version. Xacro should be installed from Conda or by Rosdep
# try:
#     import xacro
# except (ModuleNotFoundError, ImportError):
#     pip_install('xacro')

try:
    import xmltodict
except (ModuleNotFoundError, ImportError):
    pip_install('xmltodict')

try:
    import collada
except (ModuleNotFoundError, ImportError):
    pip_install('pycollada')

try:
    import PyQt5
except (ModuleNotFoundError, ImportError):
    pip_install('PyQt5')

try:
    import lxml
except (ModuleNotFoundError, ImportError):
    pip_install('lxml')


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
