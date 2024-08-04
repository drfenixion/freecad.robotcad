# Adapted from macro
# `GetGlobalPlacement<https://github.com/FreeCAD/FreeCAD-macros/blob/master/Information/GetGlobalPlacement.FCMacro>`_
# v 1.1.2.

from __future__ import annotations

from math import copysign, hypot

import FreeCAD as fc


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
            primitve" in PartDesign.
        - 'Body.Box001.Face6' if you select the top face of a Part->Body->
            "additive primitve" in PartDesign.
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
            primitve" in PartDesign.
        - 'Body.Box001.Face6' if you select the top face of a Part->Body->
            "additive primitve" in PartDesign.
    """
    p_and_s = get_global_placement_and_scale(object, subobject_fullpath)
    if p_and_s is None:
        return
    return p_and_s[0]
