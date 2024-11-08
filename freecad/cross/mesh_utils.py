"""Utility functions to work with meshes."""

from __future__ import annotations

from pathlib import Path
import subprocess
import tempfile
from typing import Iterable

import FreeCAD as fc

import Mesh  # FreeCAD

from . import wb_globals
from .deep_copy import deep_copy_object
from .freecad_utils import is_mesh
from .freecad_utils import warn
from .import_dae import export as export_dae
from .import_dae import read as read_dae
from .wb_gui_utils import WbSettingsGetter
from .wb_utils import get_workbench_param
from .wb_utils import set_workbench_param

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]


def read_mesh_dae(
        filename: Path | str,
) -> Mesh.Mesh:
    current_doc = fc.activeDocument()
    path = Path(filename)
    # `read_dae` does not export its object, so we need to let it create an
    # object in a temporary document.
    tmp_doc = fc.newDocument(hidden=True, temp=True)
    fc.setActiveDocument(tmp_doc.Name)
    read_dae(str(path))
    tmp_doc.recompute()
    merged_raw_mesh = Mesh.Mesh()
    for mesh_obj in tmp_doc.Objects:
        if hasattr(mesh_obj, 'Mesh'):
            merged_raw_mesh.addMesh(mesh_obj.Mesh)
    fc.closeDocument(tmp_doc.Name)
    if current_doc:
        fc.setActiveDocument(current_doc.Name)
    return merged_raw_mesh


def save_mesh_dae(
    obj: DO,
    filename: Path | str,
) -> None:
    """Save the mesh of a FreeCAD object into a Collada file."""
    current_doc = fc.activeDocument()
    # `export_dae` doesn't support links. Deep copy the shape in a new
    # temporary document and export them.
    tmp_doc = fc.newDocument(hidden=True, temp=True)
    copies = deep_copy_object(obj, tmp_doc)
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    export_dae(copies, str(filename))
    if current_doc:
        fc.setActiveDocument(current_doc.Name)


def read_mesh(
        filename: Path | str,
) -> Mesh.Mesh:
    """Read a mesh from a file.

    All files format supported by the Mesh module are supported.

    """
    current_doc = fc.activeDocument()
    path = Path(filename)
    # `Mesh.insert` does not export its object, so we need to let it create an
    # object in a temporary document.
    tmp_doc = fc.newDocument(hidden=True, temp=True)
    fc.setActiveDocument(tmp_doc.Name)
    Mesh.insert(str(path))
    tmp_doc.recompute()
    merged_raw_mesh = Mesh.Mesh()
    for mesh_obj in tmp_doc.Objects:
        if hasattr(mesh_obj, 'Mesh'):
            merged_raw_mesh.addMesh(mesh_obj.Mesh)
    fc.closeDocument(tmp_doc.Name)
    if current_doc:
        fc.setActiveDocument(current_doc.Name)
    return merged_raw_mesh


def save_mesh(
    obj: DO,
    filename: Path | str,
) -> None:
    """Save the mesh of a FreeCAD object into a file.

    The type of the exported file is determined by the Mesh module.
    See the Mesh module for a list of supported formats.

    """
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    # TODO: scale to meters.
    Mesh.export([obj], str(filename))


def scale_mesh_object(obj: DO, scale_factor: float | Iterable[float]) -> None:
    """Scale a mesh object in place.

    Parameters
    ----------
    - obj: FreeCAD object of type `Mesh::Feature`.
    - scale_factor: a single float or a list of 3 floats.

    """
    if not is_mesh(obj):
        raise RuntimeError(
            'First argument must be `Mesh::Feature` FreeCAD object',
        )
    if isinstance(scale_factor, float):
        scaling_vector = fc.Vector(scale_factor, scale_factor, scale_factor)
    else:
        try:
            scaling_vector = fc.Vector(scale_factor)
        except (IndexError, ValueError):
            raise RuntimeError(
                'Scaling factor must be a float or a list'
                f' of 3 floats, got {scale_factor}',
            )
    scale_mat = fc.Matrix()
    scale_mat.scale(scaling_vector)
    mesh = obj.Mesh.copy()
    mesh.transform(scale_mat)
    obj.Mesh = mesh


def get_simplified_mesh(
        obj: DO,
) -> Mesh.Mesh:
    """Create a simplified mesh from a FreeCAD object.

    Create an approximate-convex-decomposition mesh with V-HACD.

    Requires the V-HACD executable from https://github.com/kmammou/v-hacd/.
    If compiled from source, the executable is `app/build/TestVHACD` on Linux.
    On Windows, the executable is `app/TestVHACD.exe` and is provided without
    compiling from the repository.

    Parameters
    ----------

    - obj: FreeCAD object

    """
    vhacd_path = get_workbench_param(wb_globals.PREF_VHACD_PATH, '')
    if vhacd_path == '':
        # Get the path to V-HACD from the settings.
        settings_getter = WbSettingsGetter(wb_globals.g_ros_workspace)
        # if settings_getter.get_vhacd_path(Path()).samefile(Path()):
        if settings_getter.get_vhacd_path(Path()).samefile(Path()):
            # The dialog returns the input path if the user canceled.
            warn('V-HACD executable not set', True)
            raise RuntimeError('V-HACD executable not set')
        else:
            vhacd_path = str(settings_getter.vhacd_path)
            set_workbench_param(wb_globals.PREF_VHACD_PATH, vhacd_path)

    # Test if V-HACD is available.
    try:
        subprocess.run(
            [vhacd_path],
            capture_output=True,
            check=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        warn('V-HACD executable not found', True)
        raise RuntimeError('V-HACD executable not found')

    # Save the mesh to a temporary file.
    tmp_dir = tempfile.TemporaryDirectory(prefix='cross-vhacd-')
    # Input for V-HACD, must be an OBJ file.
    tmp_input_obj = tempfile.NamedTemporaryFile(suffix='.obj', dir=tmp_dir.name, delete=False)
    Mesh.export([obj], tmp_input_obj.name)
    # The name of the output file cannot be specified with V-HACD as of
    # 2022-11-22 (commit 454913f), it is 'decomp.obj', along other files.
    try:
        subprocess.run(
            [
                vhacd_path,
                tmp_input_obj.name,
                '-g', 'false',  # No logging.
            ],
            capture_output=True,
            check=True,
            cwd=tmp_dir.name,
        )
    except subprocess.CalledProcessError:
        warn('V-HACD failed', True)
        raise RuntimeError('V-HACD failed `cd {tmp_dir} && {vhacd_path} {tmp_input_obj.name}`')
    return read_mesh(Path(tmp_dir.name) / 'decomp.obj')
