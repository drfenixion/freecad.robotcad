from __future__ import annotations

from typing import Iterable, Optional

import FreeCAD as fc

from .freecad_utils import get_leafs_and_subnames
from .freecad_utils import is_body
from .freecad_utils import is_box
from .freecad_utils import is_cylinder
from .freecad_utils import is_link
from .freecad_utils import is_mesh
from .freecad_utils import is_part
from .freecad_utils import is_sphere


# Typing hints.
DO = fc.DocumentObject
AppPart = DO  # TypeId == 'App::Part'.
MeshFeature = DO  # TypeId == 'Mesh::Feature'.
DOList = Iterable[DO]


def deep_copy_object(
        obj: DO,
        doc: Optional[fc.Document] = None,
        placement: fc.Placement = fc.Placement(),
) -> DOList:
    """Copy the shapes and meshes of a FreeCAD object.

    Parameters
    ----------
    - obj: object from which to copy the shape and meshes.
    - doc: document to copy the shape to. Defaults to the document of the
           given object.
    - placement: placement applied to the copied shape and meshes.

    """
    if is_part(obj):
        return deep_copy_part(obj, doc, placement)
    elif is_mesh(obj):
        return deep_copy_mesh(obj, doc, placement)
    elif (
        is_box(obj)
        or is_cylinder(obj)
        or is_sphere(obj)
    ):
        doc = obj.Document if doc is None else doc
        copy_of_obj = doc.copyObject(obj)
        copy_of_obj.Placement = placement * copy_of_obj.Placement
        return [copy_of_obj]
    elif is_body(obj):
        return deep_copy_shape(obj, doc, placement)
    elif is_link(obj):
        return deep_copy_object(obj.LinkedObject, doc, placement * obj.Placement)
    else:
        # Let's try if `obj` has `Shape'.
        return deep_copy_shape(obj, doc, placement)


def deep_copy_part(
        part: AppPart,
        doc: Optional[fc.Document] = None,
        placement: fc.Placement = fc.Placement(),
) -> DOList:
    """Copy the shape of a "App::Part" object.

    Parameters
    ----------
    - part: "App::Part" object from which to copy the shape and meshes,
            recursively.
    - doc: document to copy the shape to. Defaults to the document of the
           given object.
    - placement: placement applied to the copied shape and meshes.

    """
    if (not hasattr(part, 'TypeId')) or part.TypeId != 'App::Part':
        # Part is not a part, return.
        try:
            fc.Console.PrintWarning(
                f'"{part.Label}" ({part.Name})'
                ' is not a part, ignoring\n',
            )
        except AttributeError:
            fc.Console.PrintWarning('Object is not a part, ignoring\n')
        return []

    doc = doc if doc else part.Document

    # `part.Shape` contains all objects with a shape except meshes.
    # `part.Shape` is already placed with `part.Placement`.
    # However, it has not `Shape` attribute if it has only meshes.
    # Maybe only on recent FreeCAD versions (works with 0.21)?
    # Implementation note: part.Shape returns a copy.
    objects: DOList = []
    if hasattr(part, 'Shape'):
        copy_of_shape = doc.addObject('Part::Feature', part.Label + '_shape')
        copy_of_shape.Shape = part.Shape
        copy_of_shape.Placement = placement * copy_of_shape.Placement
        objects.append(copy_of_shape)
    mesh_copies = get_placed_mesh_copies(part)
    for mesh in mesh_copies:
        mesh.Placement = placement * mesh.Placement
    objects += mesh_copies
    return objects


def get_placed_mesh_copies(
        part: AppPart,
        doc: Optional[fc.Document] = None,
) -> list[MeshFeature]:
    mesh_copies: list[MeshFeature] = []
    doc = doc if doc else part.Document
    for mesh, placement in get_meshes_and_placements(part):
        mesh_copies += deep_copy_mesh(mesh, doc, placement)
    return mesh_copies


def get_meshes_and_placements(
        part: AppPart,
) -> list[tuple[MeshFeature, fc.Placement]]:
    """Return all meshes in a part and their path.

    Return a list of (mesh_object, path), where path (also called subname) can be
    used to retrieve the physical placement of the mesh, for example with
    `obj.getSubObject()`.

    Parameters
    ----------
    - obj: a FreeCAD object that has the attribute `getSubObjects()`.
           If the object doesn't have `getSubObjects()`, it's considered a leaf
           and (obj, '') is returned.

    """
    outlist: list[tuple[MeshFeature, fc.Placement]] = []
    for sobj, subname in get_leafs_and_subnames(part):
        # Cf. DocumentObjectPyImp.cpp::getSubObject (l.417).
        return_type_placement = 3
        if is_mesh(sobj):
            outlist.append((sobj, part.getSubObject(subname, return_type_placement)))
        elif hasattr(sobj, 'LinkedObject') and is_mesh(sobj.LinkedObject):
            outlist.append((
                sobj.LinkedObject,
                part.getSubObject(subname, return_type_placement),
            ))
    return outlist


def deep_copy_mesh(
        mesh: MeshFeature,
        doc: Optional[fc.Document] = None,
        placement: fc.Placement = fc.Placement(),
) -> DOList:
    """Copy a "Mesh::Feature" object.

    Parameters
    ----------
    - mesh: "Mesh::Feature" object.
    - doc: document to copy the mesh to. Defaults to the document of the
           given object.
    - placement: placement applied to the copied shape and meshes.

    """
    if not is_mesh(mesh):
        return []
    doc = doc if doc else mesh.Document
    copy_of_mesh = doc.copyObject(mesh)
    # Reset the placement because FreeCAD copies it from the original without
    # respecting the tree of placements when inside a part.
    copy_of_mesh.Placement = placement
    copy_of_mesh.Label = mesh.Label + '_copy'
    return [copy_of_mesh]


def deep_copy_shape(
        obj: DO,
        doc: Optional[fc.Document] = None,
        placement: fc.Placement = fc.Placement(),
) -> DOList:
    """Copy `obj.Shape`."""
    if not hasattr(obj, 'Shape'):
        return []
    try:
        label = obj.Label
    except AttributeError:
        label = 'no_label'
    copy_of_shape = doc.addObject('Part::Feature', f'{label}_shape')
    copy_of_shape.Shape = obj.Shape
    copy_of_shape.Placement = placement * copy_of_shape.Placement
    return [copy_of_shape]
