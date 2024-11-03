"""Command to simplify a part or mesh into an approximate convex decomposition mesh."""

from __future__ import annotations

import FreeCAD as fc
import FreeCADGui as fcgui

from ..gui_utils import tr
from ..freecad_utils import warn
from ..mesh_utils import get_simplified_mesh
from ..placement_utils import get_global_placement


# Typing hints.
DO = fc.DocumentObject
SO = 'FreeCADGui.SelectionObject'  # Could not get the class from Python.


class _SimplifyMeshCommand:
    """Command to make a simplified mesh from the selected objects."""

    def GetResources(self):
        return {
            'Pixmap': 'simplify_mesh.svg',
            'MenuText': tr('Simplify mesh'),
            'ToolTip': tr(
                'Generate an approximate convex decomposition mesh'
                'from the selected object with V-HACD from'
                ' https://github.com/kmammou/v-hacd/'
                ' configured in the workbench settings'
            ),
        }

    def Activated(self):
        selection = fcgui.Selection.getSelectionEx('', 0)
        if not selection:
            warn(tr('Nothing selected, nothing to do'), True)
            return
        for sel in selection:
            obj = sel.Object
            sub_fullpaths = sel.SubElementNames
            if not sub_fullpaths:
                # An object is selected, not a face, edge, vertex.
                placement = get_global_placement(obj, '')
            else:
                # One or more subelements are selected, only consider the first one.
                placement = get_global_placement(obj, sub_fullpaths[0])
            mesh = get_simplified_mesh(obj)
            mesh_obj = obj.Document.addObject('Mesh::Feature', f'{obj.Name}_simplified')
            mesh_obj.Label = f'{obj.Label} simplified'
            mesh_obj.Mesh = mesh
            mesh_obj.Placement = placement

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('SimplifyMesh', _SimplifyMeshCommand())
