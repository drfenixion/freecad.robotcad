import sys
from pathlib import Path

from ..utils import add_python_path


def add_robotpkg_library_path() -> bool:
    """Add necessary paths to sys.path.

    If existing:
    - Add /opt/openrobots/lib/python?.?/site-packages to sys.path.

    Return True if a robotpkg installation may have been found, false
    otherwise.

    """
    # Python version.
    major = sys.version_info.major
    minor = sys.version_info.minor
    python_ver = f'python{major}.{minor}'
    robotpkg_path = Path(f'/opt/openrobots/lib/{python_ver}/site-packages')
    add_python_path(robotpkg_path)
    return robotpkg_path.exists()
