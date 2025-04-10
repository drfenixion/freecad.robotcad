from __future__ import annotations

from copy import copy
from pathlib import Path
from typing import Optional
import os
import sys

import FreeCAD as fc

from ..utils import get_parent_by_pattern
from ..utils import add_path_to_environment_variable
from .. import wb_constants


def warn(text: str, gui: bool = False) -> None:
    """Warn the user."""
    # This is a copy of .utils.warn but the utils module should not be imported
    # without GUI.
    fc.Console.PrintWarning(text + '\n')
    if gui and hasattr(fc, 'GuiUp') and fc.GuiUp:
        # Import here to be able to import the module without GUI.
        from PySide import QtCore  # type: ignore FreeCAD's PySide!
        from PySide import QtGui  # type: ignore FreeCAD's PySide!

        diag = QtGui.QMessageBox(
            QtGui.QMessageBox.Warning,
            wb_constants.WORKBENCH_NAME, text,
        )
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


def has_ros_distro_in_env() -> bool:
    """Return True if environment variable ROS_DISTRO is set."""
    p = get_ros_workspace_from_env()
    return (
            ('ROS_DISTRO' in os.environ)
            and (Path(p).exists())
    )


def is_ros_found() -> bool:
    return get_ros_distro_from_env_or_default() != ''


def add_ros_library_path(ros_distro: str = '') -> bool:
    """Add necessary paths to sys.path and os.environ['LD_LIBRARY_PATH'].

    If existing:
    - Add $ROS_WORKSPACE/install/lib/{python_ver}/site-packages to sys.path.
    - Add $ROS_WORKSPACE/install/local/lib/{python_ver}/dist-packages to sys.path.
    - Add /opt/ros/$ROS_DISTRO/lib/python?.?/site-packages to sys.path.
    - Add /opt/ros/$ROS_DISTRO/local/lib/python?.?/dist-packages to sys.path.
    - Add $ROS_WORKSPACE/install/lib to os.environ['LD_LIBRARY_PATH'].
    - Add /opt/ros/$ROS_DISTRO/lib to os.environ['LD_LIBRARY_PATH'].
    - Add /opt/ros/$ROS_DISTRO/opt/rviz_ogre_vendor/lib to os.environ['LD_LIBRARY_PATH'].
    - Add /opt/ros/$ROS_DISTRO/lib/x86_64-linux-gnu to os.environ['LD_LIBRARY_PATH'].

    Return true if a ROS installation may have been found, false otherwise.

    """

    if not ros_distro:
        ros_distro = get_ros_distro_from_env_or_default()
    if not ros_distro:
        warn(
            'The environment variable `ROS_DISTRO` is not set and no ROS'
            ' installation was found in /opt/ros'
            ', some functionalities will be missing',
        )
        return False

    ros_workspace = get_ros_workspace_from_env()
    if not has_ros_distro_in_env():
        if ros_workspace == Path():
            warn(
                'None of the environment variables `ROS_DISTRO`,'
                ' `ROS_WORKSPACE`, or `COLCON_PREFIX_PATH` is set but an'
                ' installation was found in'
                f' /opt/ros/{ros_distro}, attempting to use it',
            )

    # Add the paths in PYTHONPATH to sys.path.
    # Unfortunately, on some systems and with some versions of FreeCAD, the
    # environment variable PYTHONPATH is not taken into account and is reset.
    if 'PYTHONPATH' in os.environ:
        for path in os.environ['PYTHONPATH'].split(os.pathsep):
            if path not in sys.path:
                sys.path.append(path)
    # Python version.
    major = sys.version_info.major
    minor = sys.version_info.minor
    python_ver = f'python{major}.{minor}'
    # Add directories from ROS_WORKSPACE before system ones.
    ros_workspace = get_ros_workspace_from_env()
    # Works only for workspace with colcon's merge install strategy.
    _add_python_path(f'{ros_workspace}/install/lib/{python_ver}/site-packages')
    _add_python_path(f'{ros_workspace}/install/local/lib/{python_ver}/dist-packages')

    # On some systems (e.g. FreeCAD 0.21 on Ubuntu 20), $PYTHONPATH is not
    # taken into account in FreeCAD, add them manually.
    base = f'/opt/ros/{ros_distro}'
    for path in [
        Path(f'{base}/lib/{python_ver}/site-packages'),
        Path(f'{base}/local/lib/{python_ver}/dist-packages'), # Humble and later.
        Path(f'/usr/lib/python{major}/dist-packages'),
        Path(f'/usr/lib/{python_ver}/dist-packages'),
    ]:
        _add_python_path(path)

    add_path_to_environment_variable(f'{ros_workspace}/install/lib', 'LD_LIBRARY_PATH')
    for path in [
        Path(f'{base}/opt/rviz_ogre_vendor/lib'),
        Path(f'{base}/lib/x86_64-linux-gnu'),
        Path(f'{base}/lib'),
    ]:
        add_path_to_environment_variable(path, 'LD_LIBRARY_PATH')

    add_path_to_environment_variable(f'{ros_workspace}/install', 'AMENT_PREFIX_PATH')
    add_path_to_environment_variable(f'{base}', 'AMENT_PREFIX_PATH')
    return True


def get_ros_distro_from_env_or_default() -> str:
    """Return or guess the ROS distribution.

    Return the environment variable `ROS_DISTRO` if defined or guess from
    a system package installation in /opt/ros.

    When guessing, the most recent (and known) distro is returned, "rolling"
    having a higher priority.

    """
    if 'ROS_DISTRO' in os.environ:
        return os.environ.get('ROS_DISTRO')
    candidates = [
            'rolling',
            'jazzy',
            'iron',
            'humble',
            'galactic',
            'foxy',
    ]
    for c in candidates:
        if Path(f'/opt/ros/{c}').exists():
            return c
    return ''


def get_ros_workspace_from_env() -> Path:
    """Return the content of environment variable ROS_WORKSPACE.

    If not defined, try to guess from environment variable COLCON_PREFIX_PATH.

    """
    ws = os.environ.get('ROS_WORKSPACE', '')
    if ws:
        return Path(ws)

    # ros workspace from docker
    ws = os.environ.get('PATH_TO_WS', '')
    if ws:
        return Path(ws)

    # Guess from COLCON_PREFIX_PATH that looks like /home/user/ros_ws/install.
    colcon_prefix_path = os.environ.get('COLCON_PREFIX_PATH', '')
    if not colcon_prefix_path.endswith('/install'):
        return Path()
    return Path(colcon_prefix_path[:-len('/install')])


def get_ros_workspace_from_file(file_path: Path | str) -> Path:
    """Return the workspace containing the given file or directory.

    Return Path() if no workspace was found.

    """
    path, _ = get_parent_by_pattern(file_path, 'install/setup.bash')
    return path


def is_in_ros_workspace(path: Path | str) -> bool:
    """Return true if the given path starts with $ROS_WORKSPACE/src."""
    # Import here to avoid circular import.
    from ..wb_globals import g_ros_workspace

    src = str(g_ros_workspace / 'src')
    return str(path).startswith(src)


def without_ros_workspace(path: Path | str) -> str:
    """Return the path relative to $ROS_WORKSPACE/src.

    Return the path as-is if it doesn't start with $ROS_WORKSPACE/src.

    """
    # Import here to avoid circular import.
    from ..wb_globals import g_ros_workspace

    src = str(g_ros_workspace / 'src')
    if str(path).startswith(src):
        len_src_with_sep = len(src) + len(os.path.sep)
        return path[len_src_with_sep:]
    return copy(path)


def get_package_and_file(file_path: Path | str) -> tuple[str, str]:
    """Return the package name and relative file path.

    For example, if the file path is `$HOME/ros2_ws/src/dir/my_package/file.py`,
    return (`my_package`, `file.py`), supposing that `my_package` is a ROS package.

    If the file path is relative, return an empty package and `file_path`.

    """
    pkg_path, relative_file_path = get_parent_by_pattern(file_path, 'package.xml')
    if not pkg_path.name:
        # No package found.
        return '', str(file_path)
    return pkg_path.name, relative_file_path


def pkg_and_file_from_ros_path(
        path: str,
) -> tuple[Optional[str], Optional[str]]:
    """Return the tuple (package_name, relative_file_path).

    Return (None, None) if the guessed package does not exist.

    The input path must have the following format
    `package://<package_name>/<relative_file_path>`.

    """
    # Import here to be able to import the module without ROS.
    try:
        from ament_index_python.packages import PackageNotFoundError  # type: ignore
        from ament_index_python.packages import get_package_share_directory  # type: ignore
    except ImportError:
        warn('Cannot import ament_index_python.packages', False)
        return None, None

    if not path or not isinstance(path, str):
        return None, None
    if not path.startswith('package://'):
        warn(
            f'Invalid ROS path `{path}`, only the'
            ' `package://<package_name>/<relative_file_path>`'
            ' format is supported', False,
        )
        return None, None
    try:
        pkg, _, rel_path = path[len('package://'):].partition('/')
    except ValueError:
        warn(f'Invalid ROS path `{path}`', False)
        return None, None
    try:
        get_package_share_directory(pkg)
    except PackageNotFoundError as e:
        warn(f'Package {pkg} not found: {e}', False)
        return None, None
    return pkg, rel_path


def abs_path_from_ros_path(
        path: str,
        relative_to: Optional[Path | str] = None,
) -> Optional[Path]:
    """Return the absolute path to a file given in ROS format.

    Return the absolute path to a file given in ROS format.
    Supported formats are:
    - `package://<package_name>/<relative_file_path>` and
    - `file://<absolute_file_path>`.
    - `file://<relative_file_path>` if `relative_to` is given.

    This is the inverse function of `ros_path_from_abs_path`.

    With the `file://` format, if the input path is relative and `relative_to`
    is not given, the output path will also be relative. You've been warned.

    """
    # Import here to be able to import the module without ROS.
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore
    except ImportError:
        warn('Cannot import ament_index_python.packages', False)
        return None

    if not path:
        return None
    if (
        not (
            path.startswith('package://')
            or path.startswith('file://')
        )
    ):
        return None
    if path.startswith('package://'):
        pkg, rel_path = pkg_and_file_from_ros_path(path)
        if not pkg:
            return None
        pkg_path = get_package_share_directory(pkg)
        return Path(pkg_path) / rel_path
    elif path.startswith('file://'):
        if relative_to is not None:
            return Path(relative_to) / path[len('file://'):]
        return Path(path[len('file://'):])
    return None


def ros_path_from_abs_path(
        path: Path | str,
) -> Optional[str]:
    """Return the ROS path to the given file.

    The ROS path has the following format
    `package://<package_name>/<relative_file_path>`.
    This is the inverse function of `abs_path_from_ros_path`.

    """
    pkg, rel_path = get_package_and_file(path)
    if not pkg:
        return None
    return f'package://{pkg}/{rel_path}'


def split_package_path(output_path: Path | str) -> tuple[Path, str, Path]:
    """Return the project path, package name, package path.

    For example, if `package_path` is `/home/user/ros_ws/src/my_pkg`,
    return (Path('/home/user/ros_ws/src/my_pkg'), 'my_pkg', Path('/home/user/ros_ws/src/my_pkg/my_pkg')).

    Parameters
    ----------
    - output_path: path to the directory where will be created project and package inside the project.

    """
    output_path = Path(output_path)
    if not output_path.is_dir():
        warn('"output_path" must be a directory', True)
    output_path = output_path.resolve()
    parent = output_path.parent
    package_name = output_path.name

    # Make project folder with same name as package. Example: some_package/some_package.
    # Project folder can be used for store other packages of this project generated by external code generator
    # or by OVERCROSS
    #

    # parent is a root of project (can be git root and root of packages)
    project_path = parent / package_name # Example: parent/some_package.

    # path to description package in project folder with same name
    description_package_path = project_path / package_name # Example: parent/some_package/some_package.

    # Example structure of project:
    # parent/some_package/some_package - description package (generated by OVERCROSS)
    # Can be generated by OVERCROSS or external code generator
    # parent/some_package/other_package - example of any other package
    # parent/some_package/.git - git root place
    # parent/some_package/modules/ - git submodules folder place
    # parent/some_package/modules/ros_gz - ros package compiled from source and controlled by git submodule
    # parent/some_package/docker/ - git folder place

    return project_path, package_name, description_package_path


def _add_python_path(path: Path | str) -> None:
    """Add the path to sys.path if existing."""
    path = Path(path).expanduser().absolute()
    if path.exists() and (str(path) not in sys.path):
        sys.path.append(str(path))


def _add_ld_library_path(path: Path | str) -> None:
    """Add the path to LD_LIBRARY_PATH if existing."""
    path = Path(path).expanduser().absolute()
    existing_paths = os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep)
    if path.exists() and (str(path) not in existing_paths):
        if 'LD_LIBRARY_PATH' not in os.environ:
            os.environ['LD_LIBRARY_PATH'] = str(path)
        else:
            os.environ['LD_LIBRARY_PATH'] += os.pathsep + str(path)


def _add_ament_prefix_path(path: Path | str) -> None:
    """Add the path to AMENT_PREFIX_PATH if existing."""
    path = Path(path).expanduser().absolute()
    existing_paths = os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    if path.exists() and (str(path) not in existing_paths):
        if 'AMENT_PREFIX_PATH' not in os.environ:
            os.environ['AMENT_PREFIX_PATH'] = str(path)
        else:
            os.environ['AMENT_PREFIX_PATH'] += os.pathsep + str(path)
