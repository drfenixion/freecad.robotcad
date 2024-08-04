"""Functions that could have belonged to FreeCADGui."""

from __future__ import annotations

import FreeCAD as fc

from .freecad_utils import get_subobjects_by_full_name
from .placement_utils import get_global_placement

# Typing hints.
DO = fc.DocumentObject
SO = 'FreeCADGui.SelectionObject'  # Could not get the class from Python.


def get_subobjects_and_placements(
        selection: list[SO],
) -> list[tuple[DO, fc.Placement]]:
    """Return the list of selected subobjects and their placement."""
    outlist: list[tuple[DO, fc.Placement]] = []
    for selection_object in selection:
        root_obj = selection_object.Object
        sub_fullpaths = selection_object.SubElementNames
        if not sub_fullpaths:
            # An object is selected, not a face, edge, vertex.
            sub_fullpaths = ('',)
        for sub_fullpath in sub_fullpaths:
            subobjects = get_subobjects_by_full_name(root_obj, sub_fullpath)
            if subobjects:
                obj = subobjects[-1]
            else:
                obj = root_obj
            # One or more subelements are selected.
            placement = get_global_placement(root_obj, sub_fullpath)
            outlist.append((obj, placement))
    return outlist
