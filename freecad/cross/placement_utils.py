# Adapted from macro
# `GetGlobalPlacement<https://github.com/FreeCAD/FreeCAD-macros/blob/master/Information/GetGlobalPlacement.FCMacro>`_
# v 1.1.2.

from __future__ import annotations

from math import copysign, hypot

import FreeCAD as fc
from freecad.cross.freecad_utils import get_linked_obj


def get_global_placement_and_scale(
        object: fc.DocumentObject,
        subobject_fullpath: str,
) -> tuple[fc.Placement, fc.Vector]:
    """Return the global placement and the total scale, respecting links.
    Returns the placement and scale the objects content is related to,
    which means the properties LinkTransform and Scale is respected if
    path points to a link.

    This is in contrast with ``object.getGlobalPlacement()`` that returns
    the placement of the original object, not the linked one.

    Parameters
    ----------
    - root_object: SelectionObject.Object, where SelectionObject is obtained
        with gui.Selection.getSelectionEx('', 0)
        (i.e. not gui.Selection.getSelectionEx()).
    - subobject_fullpath: SelectionObject.SubElementNames[i].
        Examples:
        - 'Face6' if you select the top face of a cube solid made in Part.
        - 'Body.Box001.' if you select the tip of a Part->Body->"additive
            primitive" in PartDesign.
        - 'Body.Box001.Face6' if you select the top face of a Part->Body->
            "additive primitive" in PartDesign.
    """
    return_type_link_matrix = 6  # Cf. DocumentObjectPyImp.cpp::getSubObject (l.417).
    matrix = object.getSubObject(
        subobject_fullpath, return_type_link_matrix,
        transform=True,
    )
    if matrix is None:
        return
    scale_type = matrix.hasScale(1e-5)
    if scale_type == fc.ScaleType.NoScaling:
        return fc.Placement(matrix), fc.Vector(1.0, 1.0, 1.0)
    if scale_type != fc.ScaleType.Uniform:
        fc.Console.PrintWarning('Non-uniform scaling not supported\n')
        return
    fc.Console.PrintWarning('Uniform scaling may give wrong results, use with care\n')
    # Find scale.
    # Works only if uniform?
    s_gen = (
        copysign(hypot(*matrix.col(i)), matrix.col(i)[i])
        for i in range(3)
    )
    scale_vec = fc.Vector(*s_gen)
    # Workaround for scale affecting rotation
    # see https://forum.freecad.org/viewtopic.php?t=75448
    # Remove the scale from the rotation.
    position = matrix.col(3)
    matrix.setCol(3, fc.Vector())
    matrix.scale(*(1/s for s in scale_vec))
    matrix.setCol(3, position)
    return fc.Placement(matrix), scale_vec


def get_global_placement(
        object: fc.DocumentObject,
        subobject_fullpath: str,
) -> fc.Placement:
    """Return the global placement respecting links.
    Returns the placement the objects content is related to, which means
    the properties LinkTransform is respected if path points to a link.

    This is in contrast with ``object.getGlobalPlacement()`` that returns
    the placement of the original object, not the linked one.

    Parameters
    ----------
    - root_object: SelectionObject.Object, where SelectionObject is obtained
        with gui.Selection.getSelectionEx('', 0)
        (i.e. not gui.Selection.getSelectionEx()).
    - subobject_fullpath: SelectionObject.SubElementNames[i].
        Examples:
        - 'Face6' if you select the top face of a cube solid made in Part.
        - 'Body.Box001.' if you select the tip of a Part->Body->"additive
            primitive" in PartDesign.
        - 'Body.Box001.Face6' if you select the top face of a Part->Body->
            "additive primitive" in PartDesign.
    """
    p_and_s = get_global_placement_and_scale(object, subobject_fullpath)
    if p_and_s is None:
        return
    return p_and_s[0]


def get_absolute_placement(obj, with_obj_placement: bool = True):
    """ Get absolute placement of obj or first non-link object in link chain.
    Considers only Part and Assembly ancestors for calculation of placement.

    param: with_obj_placement - with object placement
    """
    obj = get_linked_obj(obj)

    # globalPlace = fc.Placement()
    # if with_obj_placement:
    #     globalPlace = obj.Placement

    # # loop via non-link ancestors to get absolute placement
    # for ancestor in obj.InListRecursive:
    #     if ancestor.TypeId != 'App::Part' or ancestor.TypeId != 'Assembly::AssemblyObject':
    #         break
    #     globalPlace = globalPlace.multiply(ancestor.Placement)

    # return globalPlace

    if with_obj_placement:
        return obj.getGlobalPlacement()
    else:
        return obj.getGlobalPlacement() * obj.Placement.inverse()


def get_obj_to_subobj_diff(obj: fc.DocumentObject, subobj: fc.DocumentObject, with_leaf_el:bool = True) -> fc.Placement:
    """Transform from object to subobject"""
    obj_ab_pl = get_absolute_placement(obj)
    subobj_ab_pl = get_absolute_placement(subobj, with_leaf_el)
    obj_to_subobj_diff = subobj_ab_pl * obj_ab_pl.inverse()

    return obj_to_subobj_diff
