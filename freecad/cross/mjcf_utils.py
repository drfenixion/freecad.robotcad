#!/usr/bin/env python3
"""Convert MuJoCo MJCF models to URDF for the models library.

The conversion is delegated to the MuJoCo Python bindings
(:mod:`freecad.cross.mjcf_urdf_mujoco`), which load the MJCF with
:meth:`mujoco.MjModel.from_xml_path` and emit a URDF directly from the compiled
model. MuJoCo natively resolves every MJCF convenience feature that real
models (e.g. MuJoCo Menagerie) rely on:

* ``default`` classes inherited through the body tree via ``childclass``;
* implicit mesh names (an unnamed ``<mesh file="assets/base_0.obj"/>`` is
  referenced as ``mesh="base_0"``);
* mesh directory resolution, mesh ``scale`` and centring offsets;
* implicit joint types (``hinge`` when ``type`` is omitted) and the
  ``autolimits`` / ``limited`` / ``range`` flags used for joint limits.

Generated URDFs are cached under
``~/.cache/robot_descriptions/mjcf_to_urdf/<version>-<hash>/`` and consumed by
the standard URDF import pipeline
(:func:`freecad.cross.robot_from_urdf.robot_from_urdf_path`).
"""

import hashlib
import os
import os.path as osp
from pathlib import Path

# Bumped whenever the conversion output changes in a way that invalidates
# previously cached URDFs (the cache key includes this value).
CONVERTER_VERSION = '8'


# ---------------------------------------------------------------------------
# Package path (mujoco lives in AdditionalPythonPackages)
# ---------------------------------------------------------------------------


def _ensure_packages_path():
    try:
        from .packages import add_packages_path
        add_packages_path()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Cache / output helpers
# ---------------------------------------------------------------------------


def _cache_dir():
    cache_root = os.path.expanduser(
        os.environ.get(
            "ROBOT_DESCRIPTIONS_CACHE",
            "~/.cache/robot_descriptions",
        )
    )
    return os.path.join(cache_root, "mjcf_to_urdf")


def _convert_with_mujoco(mjcf_path, final_path, output_dir):
    """Convert ``mjcf_path`` with the MuJoCo bindings.

    Thin wrapper around
    :func:`freecad.cross.mjcf_urdf_mujoco.convert_mjcf_to_urdf`.
    """
    from .mjcf_urdf_mujoco import convert_mjcf_to_urdf
    mesh_dir = output_dir / 'meshes'
    convert_mjcf_to_urdf(
        str(mjcf_path), str(final_path), mesh_dir=str(mesh_dir))
    return final_path


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def get_urdf_path(
    mjcf_path,
    package_path=None,
    repository_path=None,
    output_dir=None,
):
    """Convert an MJCF file to URDF and return the path to the URDF file.

    Args:
        mjcf_path: Path to the MJCF file.
        package_path: Accepted for API compatibility with the models library
            (unused: meshes are exported next to the generated URDF and
            referenced with absolute ``file://`` paths).
        repository_path: Accepted for API compatibility (unused).
        output_dir: Optional output directory. Defaults to a cache directory
            keyed by the MJCF content and the converter version.

    Returns:
        Path to the generated URDF file.
    """
    _ensure_packages_path()

    mjcf_path = Path(mjcf_path)
    if not mjcf_path.exists():
        raise FileNotFoundError(f'MJCF file not found: {mjcf_path}')

    if output_dir is None:
        with open(str(mjcf_path), 'rb') as f:
            content_hash = hashlib.sha256(f.read()).hexdigest()[:16]
        output_dir = osp.join(
            _cache_dir(), f'{CONVERTER_VERSION}-{content_hash}')
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    final_path = output_dir / f'{mjcf_path.stem}.urdf'

    return _convert_with_mujoco(mjcf_path, final_path, output_dir)
