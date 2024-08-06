# Utility function depending on the `urdf_parser_py` module (provided by ROS).

from __future__ import annotations

from pathlib import Path
from typing import Optional

import FreeCAD as fc

import Mesh as fcmesh  # FreeCAD.

from urdf_parser_py.urdf import Box
from urdf_parser_py.urdf import Cylinder
from urdf_parser_py.urdf import Joint as UrdfJoint
from urdf_parser_py.urdf import Joint as UrdfLink
from urdf_parser_py.urdf import Mesh
from urdf_parser_py.urdf import Pose
from urdf_parser_py.urdf import Sphere

from .freecad_utils import add_object
from .freecad_utils import is_group
from .freecad_utils import is_mesh
from .freecad_utils import warn
from .mesh_utils import read_mesh_dae
from .mesh_utils import scale_mesh_object
from .ros.utils import abs_path_from_ros_path
from .ros.utils import pkg_and_file_from_ros_path
from .ros.utils import ros_path_from_abs_path
from .urdf_utils import rotation_from_rpy

# Typing hints.
Doc = fc.Document
DO = fc.DocumentObject
Shape = [Box, Cylinder, Mesh, Sphere]


def obj_from_geometry(
        geometry: Shape,
        doc_or_group: [Doc | DO],
) -> tuple[Optional[DO], Optional[Path]]:
    """Return a FreeCAD object for the URDF shape with the path for meshes."""
    if isinstance(geometry, Box):
        return obj_from_box(geometry, doc_or_group)
    if isinstance(geometry, Cylinder):
        return obj_from_cylinder(geometry, doc_or_group)
    if isinstance(geometry, Mesh):
        return obj_from_mesh(geometry, doc_or_group)
    if isinstance(geometry, Sphere):
        return obj_from_sphere(geometry, doc_or_group)
    raise NotImplementedError('Primitive not implemented')


def placement_from_origin(
        origin: Pose,
) -> fc.Placement:
    """Return the FreeCAD placement corresponding to URDF origin."""
    placement = fc.Placement()
    if origin is None:
        return placement
    if hasattr(origin, 'position'):
        # Convert from meters to millimeters.
        placement.Base = fc.Vector(origin.position) * 1000.0
    if hasattr(origin, 'rpy'):
        placement.Rotation = rotation_from_rpy(origin.rpy)
    return placement


def placement_from_link(
        link: UrdfLink,
) -> fc.Placement:
    """Return the FreeCAD placement corresponding to the URDF link."""
    if not hasattr(link, 'origin'):
        return fc.Placement()
    return placement_from_origin(link.origin)


def placement_from_joint(
        joint: UrdfJoint,
) -> fc.Placement:
    """Return the FreeCAD placement corresponding to the URDF joint."""
    if not hasattr(joint, 'origin'):
        return fc.Placement()
    return placement_from_origin(joint.origin)


def axis_to_z(
        joint: UrdfJoint,
) -> fc.Placement:
    """Return the rotation to bring `joint.axis` to z."""
    if hasattr(joint, 'axis') and (joint.axis is not None):
        axis = fc.Vector(joint.axis)
    else:
        # URDF's default axis.
        if joint.type in ['prismatic', 'revolute', 'continuous', 'planar']:
            axis = fc.Vector(1.0, 0.0, 0.0)
        else:
            axis = fc.Vector(0.0, 0.0, 1.0)
    zaxis = fc.Vector(0.0, 0.0, 1.0)
    # Orient to bring the joint axis along z.
    return fc.Rotation(zaxis, axis)


def placement_along_z_from_joint(
        joint: UrdfJoint,
) -> fc.Placement:
    """Return the joint placement so that its axis is along z."""
    placement = placement_from_joint(joint)
    placement.Rotation *= axis_to_z(joint)
    return placement


def obj_from_box(
        geometry: Box,
        doc_or_group: [Doc | DO],
) -> tuple[Optional[DO], None]:
    """Return a `Part::Box` object and None.

    Return a `Part::Box` object for the URDF shape.
    The second element of the tuple is None for API consistency.

    """
    obj = add_object(doc_or_group, 'Part::Box', 'box')
    obj.Length = geometry.size[0] * 1000.0  # m to mm.
    obj.Width = geometry.size[1] * 1000.0
    obj.Height = geometry.size[2] * 1000.0
    obj.Placement.Base -= fc.Vector(geometry.size) * 1000.0 / 2.0
    return obj, None


def obj_from_cylinder(
        geometry: Cylinder,
        doc_or_group: [Doc | DO],
) -> tuple[Optional[DO], None]:
    """Return a `Part::Cylinder` object and None.

    Return a `Part::Cylinder` object for the URDF shape.
    The second element of the tuple is None for API consistency.

    """
    obj = add_object(doc_or_group, 'Part::Cylinder', 'cylinder')
    obj.Radius = geometry.radius * 1000.0  # m to mm.
    obj.Height = geometry.length * 1000.0  # m to mm.
    obj.Placement.Base.z -= geometry.length * 1000.0 / 2.0
    return obj, None


def obj_from_mesh(
        geometry: Mesh,
        doc_or_group: [Doc | DO],
) -> tuple[Optional[DO], Optional[Path]]:
    """Return a `Mesh::Feature` object and the path to its file.

    Return a `Mesh::Feature` object for the URDF shape and the path to its
    file.
    If the same file was already imported, return the corresponding existing
    object.

    """
    mesh_path = abs_path_from_ros_path(geometry.filename)
    if not mesh_path:
        pkg, rel_path = pkg_and_file_from_ros_path(mesh_path)
        if pkg:
            warn(f'ROS package {pkg} not found, cannot read {geometry.filename}')
        else:
            warn(f'Cannot parse mesh path {geometry.filename}')
        return None, None

    # We do not take `geometry.filename` directly because it may use a
    # different format than `package://...`.
    mesh_ros_path = ros_path_from_abs_path(mesh_path.expanduser())

    # Look for an existing object with the same mesh.
    if is_group(doc_or_group):
        doc = doc_or_group.Document
        group = doc_or_group
    else:
        doc = doc_or_group
        group = None
    for obj in doc.Objects:
        if is_mesh(obj):
            if obj.Label2 == mesh_ros_path:
                return obj, mesh_path
    if mesh_path.suffix.lower() == '.dae':
        raw_mesh = read_mesh_dae(mesh_path)
    else:
        raw_mesh = fcmesh.read(str(mesh_path))
    mesh_obj = doc.addObject('Mesh::Feature', mesh_path.name)
    mesh_obj.Label = mesh_path.name
    mesh_obj.Label2 = mesh_ros_path
    if group:
        group.addObject(mesh_obj)
    mesh_obj.Mesh = raw_mesh
    if mesh_path.suffix.lower() in ['.stl', '.obj']:
        scale_mesh_object(mesh_obj, 1000.0)  # m to mm.
    if ((geometry.scale is not None)
            and (
                not (geometry.scale == 1.0)
                or geometry.scale == [1.0, 1.0, 1.0]
            )):
        scale_mesh_object(mesh_obj, geometry.scale)
    return mesh_obj, mesh_path


def obj_from_sphere(
        geometry: Sphere,
        doc_or_group: [Doc | DO],
) -> tuple[Optional[DO], None]:
    obj = add_object(doc_or_group, 'Part::Sphere', 'sphere')
    obj.Radius = geometry.radius * 1000.0  # m to mm.
    return obj, None
