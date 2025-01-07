"""Contains various helper functions to export to URDF from FreeCAD."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, Tuple, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

import numpy as np
from numpy.typing import ArrayLike

from freecad.cross.placement_utils import get_absolute_placement

from .freecad_utils import get_leafs_and_subnames
from .freecad_utils import is_box
from .freecad_utils import is_cylinder
from .freecad_utils import is_group
from .freecad_utils import is_lcs
from .freecad_utils import is_link as is_fclink
from .freecad_utils import is_part
from .freecad_utils import is_sphere
from .freecad_utils import label_or
from .freecad_utils import warn
from .utils import get_valid_filename
from .wb_utils import is_primitive


# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
PartBox = DO  # TypeId == 'Part::Box'
PartCyl = DO  # TypeId == 'Part::Cylinder'
PartSphere = DO  # TypeId == 'Part::Sphere'
Rpy = Tuple[float, float, float]  # URDF rotation, Roll-Pitch-Yaw.
QuatList = Tuple[float, float, float, float]  # (qx, qy, qz, qw).

# Small number to test whether a number is close to zero.
_EPS = np.finfo(float).eps * 4.0


@dataclass
class XmlForExport:
    xml: et.Element
    object: DO
    placement: fc.Placement
    mesh_filename: str = ''


def quaternion_matrix(quaternion: QuatList) -> ArrayLike:
    """Return the homogeneous rotation matrix from quaternion.

    The quaternion must have the format (qx, qy, qz, qw).

    Inspired from quaternion_matrix in
    https://github.com/ros/geometry/blob/noetic-devel/tf/src/tf/transformations.py.

    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array(
        (
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0),
        ), dtype=np.float64,
    )


def euler_from_matrix(matrix) -> Rpy:
    """Convert a 3x3 rotation matrix to Euler angles (X, Y, Z) in fixed frame.

    This Euler angle convention is the one used in URDF format.

    The quaternion must have the format (qx, qy, qz, qw).

    Inspired from euler_from_matrix(matrix, axes='sxyz') in
    https://github.com/ros/geometry/blob/noetic-devel/tf/src/tf/transformations.py.

    """
    i, j, k = 0, 1, 2

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
    if cy > _EPS:
        ax = math.atan2( M[k, j],  M[k, k])
        ay = math.atan2(-M[k, i],  cy)
        az = math.atan2( M[j, i],  M[i, i])
    else:
        ax = math.atan2(-M[j, k],  M[j, j])
        ay = math.atan2(-M[k, i],  cy)
        az = 0.0

    return ax, ay, az


def rpy_from_quaternion(q: QuatList) -> Rpy:
    """Convert quaternion to rpy (URDF convention).

    The quaternion must have the format (qx, qy, qz, qw).

    Cf. `rotation_from_rpy` for the "inverse" function.

    """
    return euler_from_matrix(quaternion_matrix(q))


def rotation_from_rpy(rpy: Rpy) -> fc.Rotation:
    """Convert rpy (URDF convention) to a FreeCAD's rotation (quaternion).

    Cf. `rpy_from_quaternion` for the "inverse" function.

    """
    # Inverse the order to get from URDF's rotations about absolute axes to
    # relative axes.
    return (
            fc.Rotation(fc.Vector(0.0, 0.0, 1.0), np.degrees(rpy[2]))
            * fc.Rotation(fc.Vector(0.0, 1.0, 0.0), np.degrees(rpy[1]))
            * fc.Rotation(fc.Vector(1.0, 0.0, 0.0), np.degrees(rpy[0]))
    )


def sanitize_for_xml_comment(comment: str) -> str:
    """Returns the string without '--'."""
    return comment.replace('--', '⸗⸗')


def xml_comment_element(obj_label: str) -> et.Element:
    # Workaround et.Comment() not being correctly serialized in FreeCAD 0.21.
    # return et.fromstring(f'<cad_name value="{obj_label}"/>')
    return et.Comment(sanitize_for_xml_comment(obj_label))


def urdf_origin_from_placement(p: fc.Placement) -> et.Element:
    """Return an xml element 'origin'."""

    # previous implementation had a precision bug.
    # rpy = rpy_from_quaternion(p.Rotation.Q)
    # In some cases of joint rotation this gave incorrect URDF rpy radians,
    # resulting in some apparent tilt of the orientation.

    # new implementation have less float operation
    # and correct in all tested rotation cases
    xyz = p.Rotation.toEulerAngles('XYZ')
    xyz_rad = (math.radians(xyz[0]), math.radians(xyz[1]), math.radians(xyz[2]))
    pattern = '<origin xyz="{v.x:.6} {v.y:.6} {v.z:.6}" rpy="{r[0]} {r[1]} {r[2]}" />'
    return et.fromstring(pattern.format(v=p.Base * 1e-3, r=xyz_rad))


def urdf_geometry_box(length_x: float, length_y: float, length_z: float) -> et.Element:
    """Return an xml element 'geometry' with a box.

    Lengths must be given in meters.

    """
    geometry = et.fromstring('<geometry/>')
    geometry.append(
        et.fromstring(
        f'<box size="{length_x} {length_y} {length_z}" />',
        ),
    )
    return geometry


def urdf_box_placement_from_object(
        box: PartBox,
        placement: Optional[fc.Placement] = None,
) -> fc.Placement:
    """Return the FreeCAD placement of the box center.

    Return the placement of the box center for a FreeCAD box with the placement
    at its lower-left-bottom point.

    """
    if not is_box(box):
        raise RuntimeError("First argument must be a 'Part::Box'")
    if not placement:
        placement = box.Placement
    to_center = fc.Placement(
        fc.Vector(
            box.Length.Value,
            box.Width.Value,
            box.Height.Value,
        ) / 2.0,
        fc.Rotation(),
    )
    return placement * to_center


def urdf_geometry_sphere(radius: float) -> et.Element:
    """Return an xml element 'geometry' with a sphere.

    The radius must be given in meters.

    """
    geometry = et.fromstring('<geometry/>')
    geometry.append(
        et.fromstring(
        f'<sphere radius="{radius}" />',
        ),
    )
    return geometry


def urdf_sphere_placement_from_object(
        sphere: PartSphere,
        placement: Optional[fc.Placement] = None,
) -> fc.Placement:
    """Return the FreeCAD placement of the sphere center.

    Return the placement of the sphere center for a FreeCAD sphere.

    """
    if not is_sphere(sphere):
        raise RuntimeError("First argument must be a 'Part::Sphere'")
    if not placement:
        placement = sphere.Placement
    return placement


def urdf_geometry_cylinder(radius: float, length: float) -> et.Element:
    """Return an xml element 'geometry' with a cylinder.

    Lengths must be given in meters.

    """
    geometry = et.fromstring('<geometry/>')
    geometry.append(
        et.fromstring(
        f'<cylinder radius="{radius}" length="{length}" />',
        ),
    )
    return geometry


def urdf_cylinder_placement_from_object(
        cyl: PartCyl,
        placement: Optional[fc.Placement] = None,
) -> fc.Placement:
    """Return the FreeCAD placement of the cylinder center.

    Return the placement of the cylinder center for a FreeCAD cylinder with the
    placement at the center of its bottom disc.

    """
    if not is_cylinder(cyl):
        raise RuntimeError("Argument must be a 'Part::Cylinder'")
    if not placement:
        placement = cyl.Placement
    to_center = fc.Placement(fc.Vector(0.0, 0.0, cyl.Height.Value / 2.0), fc.Rotation())
    return placement * to_center


def _urdf_generic_from_box(
        box: PartBox,
        generic: str,
        obj_label: str = '',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False,
) -> et.Element:
    """Return the xml element for visual or collision for a FreeCAD's box.

    Parameters
    ----------

    - cyl: the FreeCAD cylinder Part object
    - generic: {'visual', 'collision'}.
    - obj_label: Label of the FreeCAD object. To comment the output xml.
      Defaults to box.Label.
    - placement:
      - additional displacement of the box, the box' original
        placement is added to this if ``ignore_obj_placement`` is False.
      - box' placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    if not is_box(box):
        raise RuntimeError("Argument must be a 'Part::Box'")
    parent = et.fromstring(f'<{generic}/>')

    if not obj_label:
        obj_label = box.Label
    parent.append(xml_comment_element(obj_label))

    if not ignore_obj_placement:
        placement = placement * box.Placement
    center_placement = urdf_box_placement_from_object(box, placement)
    parent.append(urdf_origin_from_placement(center_placement))

    parent.append(
        urdf_geometry_box(
        box.Length.getValueAs('m'),
        box.Width.getValueAs('m'),
        box.Height.getValueAs('m'),
        ),
    )
    return parent


def urdf_visual_from_box(
        box: PartBox,
        obj_label: str = '',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False,
) -> et.Element:
    """Return the xml element for visual for a FreeCAD's box.

    Parameters
    ----------

    - cyl: the FreeCAD cylinder Part object
    - obj_label: Label of the FreeCAD object. To comment the output xml.
        Defaults to box.Label.
    - placement:
      - additional displacement of the box, the box' original
        placement is added to this if ``ignore_obj_placement`` is False.
      - box' placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_box(
        box, 'visual', obj_label,
        placement, ignore_obj_placement,
    )


def urdf_collision_from_box(
        box: PartBox,
        obj_label: str = '',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False,
) -> et.Element:
    """Return the xml element for collision for a FreeCAD's box.

    Parameters
    ----------

    - cyl: the FreeCAD cylinder Part object
    - obj_label: Label of the FreeCAD object. To comment the output xml.
        Defaults to box.Label.
    - placement:
      - additional displacement of the box, the box' original
        placement is added to this if ``ignore_obj_placement`` is False.
      - box' placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_box(
        box, 'collision', obj_label,
        placement, ignore_obj_placement,
    )


def _urdf_generic_from_sphere(
        sphere: PartSphere,
        generic: str,
        obj_label: str = '',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False,
) -> et.Element:
    """Return the xml element for visual or collision for a FreeCAD's sphere.

    Parameters
    ----------

    - sphere: the FreeCAD sphere Part object
    - generic: {'visual', 'collision'}.
    - obj_label: Label of the FreeCAD object. To comment the output xml.
        Defaults to sphere.Label.
    - placement:
      - additional displacement of the sphere, the sphere's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - sphere's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    if not is_sphere(sphere):
        raise RuntimeError("Argument must be a 'Part::Sphere'")

    parent = et.fromstring(f'<{generic}/>')

    if not obj_label:
        obj_label = sphere.Label
    parent.append(xml_comment_element(obj_label))

    if not ignore_obj_placement:
        placement = placement * sphere.Placement

    parent.append(
        urdf_geometry_sphere(
        sphere.Radius.getValueAs('m'),
        ),
    )
    return parent


def urdf_visual_from_sphere(
        sphere: PartSphere,
        obj_label: str = '',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False,
) -> et.Element:
    """Return the xml element for visual for a FreeCAD's sphere.

    Parameters
    ----------

    - sphere: the FreeCAD sphere Part object
    - obj_label: Label of the FreeCAD object. To comment the output xml.
        Defaults to sphere.Label.
    - placement:
      - additional displacement of the sphere, the sphere's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - sphere's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_sphere(
        sphere, 'visual', obj_label,
        placement, ignore_obj_placement,
    )


def urdf_collision_from_sphere(
        sphere: PartSphere,
        obj_label: str = '',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False,
) -> et.Element:
    """Return the xml element for collision for a FreeCAD's sphere.

    Parameters
    ----------

    - sphere: the FreeCAD sphere Part object
    - obj_label: Label of the FreeCAD object. To comment the output xml.
        Defaults to sphere.Label.
    - placement:
      - additional displacement of the sphere, the sphere's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - sphere's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_sphere(
        sphere, 'collision', obj_label,
        placement, ignore_obj_placement,
    )


def _urdf_generic_from_cylinder(
        cyl: PartCyl,
        generic: str,
        obj_label: str = '',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False,
) -> et.Element:
    """Return the xml element for visual or collision for a FreeCAD's cylinder.

    Parameters
    ----------

    - cyl: the FreeCAD cylinder Part object
    - generic: {'visual', 'collision'}.
    - obj_label: Label of the FreeCAD object. To comment the output xml.
        Defaults to cyl.Label.
    - placement:
      - additional displacement of the cylinder, the cylinder's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - cylinder's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    if not is_cylinder(cyl):
        raise RuntimeError("Argument must be a 'Part::Cylinder'")

    parent = et.fromstring(f'<{generic}/>')

    if not obj_label:
        obj_label = cyl.Label
    parent.append(xml_comment_element(obj_label))

    if not ignore_obj_placement:
        placement = placement * cyl.Placement
    center_placement = urdf_cylinder_placement_from_object(cyl, placement)
    parent.append(urdf_origin_from_placement(center_placement))

    parent.append(
        urdf_geometry_cylinder(
        cyl.Radius.getValueAs('m'),
        cyl.Height.getValueAs('m'),
        ),
    )
    return parent


def urdf_visual_from_cylinder(
        cyl: PartCyl,
        obj_label: str = '',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False,
) -> et.Element:
    """Return the xml element for visual for a FreeCAD's cylinder.

    Parameters
    ----------

    - cyl: the FreeCAD cylinder Part object
    - obj_label: Label of the FreeCAD object. To comment the output xml.
        Defaults to cyl.Label.
    - placement:
      - additional displacement of the cylinder, the cylinder's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - cylinder's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_cylinder(
        cyl, 'visual', obj_label,
        placement, ignore_obj_placement,
    )


def urdf_collision_from_cylinder(
        obj_label: str,
        cyl: PartCyl,
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False,
) -> et.Element:
    """Return the xml element for collision for a FreeCAD's cylinder.

    Parameters
    ----------

    - cyl: the FreeCAD cylinder Part object
    - obj_label: Label of the FreeCAD object. To comment the output xml.
        Defaults to cyl.Label.
    - placement:
      - additional displacement of the cylinder, the cylinder's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - cylinder's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_cylinder(
        cyl, 'collision', obj_label,
        placement, ignore_obj_placement,
    )


def urdf_geometry_mesh(mesh_name: str, package_name: str) -> et.Element:
    """Return an xml element 'geometry' with a mesh.

    Parameters
    ----------
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.

    """
    geometry = et.fromstring('<geometry/>')
    geometry.append(
        et.fromstring(
        f'<mesh filename="package://{package_name}/meshes/{mesh_name}" />',
        ),
    )
    return geometry


def _urdf_generic_mesh(
        obj_label: str,
        mesh_name: str,
        package_name: str,
        generic: str,
        placement: fc.Placement = fc.Placement(),
) -> et.Element:
    """Return the xml element for visual or collision mesh for a FreeCAD object.

    The URDF just contains a reference to the object.
    The mesh is not exported.

    Parameters
    ----------

    - obj_label: Label of the FreeCAD object. To comment the output xml.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - generic: {'visual', 'collision'}.
    - placement: additional displacement of the mesh, the mesh's
        original placement is added to this.

    """
    parent = et.fromstring(f'<{generic}/>')
    parent.append(xml_comment_element(obj_label))
    parent.append(urdf_origin_from_placement(placement))
    parent.append(urdf_geometry_mesh(mesh_name, package_name))
    return parent


def urdf_visual_mesh(
        obj_label: str,
        mesh_name: str,
        package_name: str,
        placement: fc.Placement = fc.Placement(),
) -> et.Element:
    """Return the xml element for visual mesh for a FreeCAD object.

    The URDF just contains a reference to the object.
    The mesh is not exported.

    Parameters
    ----------

    - obj_label: Label of the FreeCAD object. To comment the output xml.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - placement: additional displacement of the mesh, the mesh's
        original placement is added to this.

    """
    return _urdf_generic_mesh(obj_label, mesh_name, package_name, 'visual', placement)


def urdf_collision_mesh(
        obj_label: str,
        mesh_name: str,
        package_name: str,
        placement: fc.Placement = fc.Placement(),
) -> et.Element:
    """Return the xml element for collision mesh for a FreeCAD object.

    The URDF just contains a reference to the mesh.
    The mesh is not exported.

    Parameters
    ----------

    - obj_label: Label of the FreeCAD object. To comment the output xml.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - placement: additional displacement of the mesh, the mesh's
        original placement is added to this.

    """
    return _urdf_generic_mesh(obj_label, mesh_name, package_name, 'collision', placement)


def _get_mesh_filename(obj: DO) -> str:
    """Return the mesh filename for a FreeCAD object."""
    if hasattr(obj, 'LinkedObject'):
        linked_obj = obj.LinkedObject
    else:
        linked_obj = obj
    label = label_or(linked_obj, 'mesh')
    if hasattr(linked_obj, 'Document'):
        doc_name = linked_obj.Document.Name
    else:
        doc_name = 'unsaved_doc'
    return get_valid_filename(f'{doc_name}_{label}.dae')


def _urdf_generic_from_object(
        obj: fc.DocumentObject,
        generic: str,
        package_name: Optional[str] = None,
        placement: Optional[fc.Placement] = None,
) -> list[XmlForExport]:
    """Return the xml elements for visual or collision for a FreeCAD object.

    For meshes, the URDF just contains a reference to the object, the mesh is
    not exported here.

    Returns the necessary elements for export.

    The path is `None` for primitive types (box, sphere, cylinder). The path is
    the filename with extension for meshes, not the full path.

    Parameters
    ----------

    - obj: the FreeCAD object.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - generic: {'visual', 'collision'}.
    - placement: optional additional displacement. The object's placement will
        be added to this.

    """
    out_data: list[XmlForExport] = []
    for subobj, subname in get_leafs_and_subnames(obj):
        # Implementation note: linked_object is subobj if subobj is not a link.
        linked_object, link_matrix = subobj.getLinkedObject(
            recursive=True,
            transform=True,
            matrix=fc.Matrix(),
        )
        if is_primitive(linked_object):
            placement_for_dae_export = fc.Placement(link_matrix)
        else:
            # Implementation note: the DAE export of FreeCAD integrates the
            # placement of the linked object into the mesh, as of 2023-09.
            # We thus need to remove it from the placement of the object.
            placement_for_dae_export = (
                fc.Placement(link_matrix)
                * linked_object.Placement.inverse()
            )
        if not placement:
            this_placement = placement_for_dae_export
        else:
            # subobj_to_obj_diff is needed in case of wrapper with non-zero placement between subobject and object (f.e. of Visual)
            obj_ab_pl = get_absolute_placement(obj)
            subobj_ab_pl = get_absolute_placement(subobj, with_leaf_el = False)
            subobj_to_obj_diff = subobj_ab_pl * obj_ab_pl.inverse()

            this_placement = placement * subobj_to_obj_diff * placement_for_dae_export

        filename = ''
        if subobj is obj:
            # No FreeCAD link.
            name_for_comment = sanitize_for_xml_comment(subobj.Label)
        else:
            name_for_comment = sanitize_for_xml_comment(obj.Label + '/' + subname)
        if is_box(linked_object):
            # We ignore the object placement because it's given by link_matrix.
            xml = _urdf_generic_from_box(
                linked_object, generic, name_for_comment,
                this_placement, ignore_obj_placement=True,
            )
        elif is_sphere(linked_object):
            # We ignore the object placement because it's given by link_matrix.
            xml = _urdf_generic_from_sphere(
                linked_object, generic, name_for_comment,
                this_placement, ignore_obj_placement=True,
            )
        elif is_cylinder(linked_object):
            # We ignore the object placement because it's given by link_matrix.
            xml = _urdf_generic_from_cylinder(
                linked_object, generic, name_for_comment,
                this_placement, ignore_obj_placement=True,
            )
        elif is_lcs(linked_object):
            # LCS have a shape and would thus be exported.
            continue
        else:
            # TODO: handle duplicated labels.
            filename = _get_mesh_filename(subobj)
            if not package_name:
                warn(
                    'Internal error, `package_name` empty'
                    ' but mesh found, using "package" as package name',
                )
                package_name = 'package'
            xml = _urdf_generic_mesh(
                name_for_comment, filename,
                package_name, generic, this_placement,
            )
        out_data.append(XmlForExport(xml, linked_object, this_placement, filename))
    return out_data


def urdf_visual_from_object(
        obj: fc.DocumentObject,
        package_name: Optional[str] = None,
        placement: Optional[fc.Placement] = None,
) -> list[XmlForExport]:
    """Return the xml element for visual for a FreeCAD object.

    For meshes, the URDF just contains a reference to the object, the mesh is
    not exported.

    Returns (list of xml elements, list of objects, list of placements).

    Parameters
    ----------

    - obj: the FreeCAD object.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - placement: optional placement of the mesh relative to the URDF link.
        If not given, obj.Placement will be used.

    """
    return _urdf_generic_from_object(obj, 'visual', package_name, placement)


def urdf_collision_from_object(
        obj: fc.DocumentObject,
        package_name: str = '',
        placement: Optional[fc.Placement] = None,
) -> list[XmlForExport]:
    """Return the xml element for collision for a FreeCAD object.

    For meshes, the URDF just contains a reference to the object, the mesh is
    not exported.

    Returns (list of xml elements, list of objects, list of placements).

    Parameters
    ----------

    - obj: the FreeCAD object.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - placement: optional placement of the mesh relative to the URDF link.
        If not given, obj.Placement will be used.

    """
    return _urdf_generic_from_object(obj, 'collision', package_name, placement)


def urdf_inertial(
        mass: float,
        center_of_mass: fc.Placement,
        ixx: float,
        ixy: float,
        ixz: float,
        iyy: float,
        iyz: float,
        izz: float,
) -> et.Element:
    """Return the xml element for inertial."""
    inertial_et = et.Element('inertial')
    et.SubElement(inertial_et, 'mass', {'value': str(mass)})
    origin_et = urdf_origin_from_placement(center_of_mass)
    inertial_et.append(origin_et)
    pattern = (
        '<inertia ixx="{ixx}" ixy="{ixy}" ixz="{ixz}"'
        ' iyy="{iyy}" iyz="{iyz}" izz="{izz}"/>'
    )
    inertia_et = et.fromstring(
        pattern.format(
            ixx=ixx, ixy=ixy, ixz=ixz,
            iyy=iyy, iyz=iyz, izz=izz,
        ),
    )
    inertial_et.append(inertia_et)
    return inertial_et


def export_group_with_lcs(
    group: fc.DocumentObjectGroup,
    package_path: [str | Path],
) -> bool:
    """Export a group containing a link to an assembly and links to LCS.

    Export a group containing a link to an assembly and links to local
    coordinate systems (LCS). Each LCS represent a joint. The assembly is a
    part with part children (or links to parts). It will be used to get the
    link associated to each joint as well as the relative link poses.

    """
    if not is_group(group):
        raise ValueError(
            'First argument must be a group, i.e.'
            ' "App::DocumentObjectGroup"',
        )
    fc_links = group.Group
    for fc_link in fc_links:
        if not is_fclink(fc_link):
            warn(f'"{label_or(fc_link)}" is not a link, ignoring')
            continue
        linked_object = fc_link.LinkedObject
        if not (is_lcs(linked_object) or is_part(linked_object)):
            # linked_object is not a coordinate system, cannot handle it.
            warn(
                f'Linked object of "{label_or(fc_link)}" is'
                ' not a coordinate system or part, ignoring',
            )
            continue
    return True
