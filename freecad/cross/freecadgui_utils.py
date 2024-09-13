"""Functions that could have belonged to FreeCADGui."""

from __future__ import annotations

import FreeCAD as fc
import FreeCADGui as fcgui

from .freecad_utils import get_subobjects_by_full_name
from .freecad_utils import first_object_with_volume
from .freecad_utils import adjustedGlobalPlacement
from .freecad_utils import is_lcs
from .freecad_utils import is_part
from .freecad_utils import message
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


def createBoundBox(obj):
    return createBoundAbstract(obj, createPrimitive = createBox)


def createBoundCylinder(obj):
    return createBoundAbstract(obj, createPrimitive = createCylinder)


def createBoundSphere(obj):
    return createBoundAbstract(obj, createPrimitive = createSphere)


def createBox(boundBox_, nameLabel):
    boundObj = fc.ActiveDocument.addObject("Part::Box", nameLabel + "_BoundBox")
    boundObj.Length.Value = boundBox_.XLength
    boundObj.Width.Value  = boundBox_.YLength
    boundObj.Height.Value = boundBox_.ZLength

    boundBoxLocation = fc.Vector(boundBox_.XMin,boundBox_.YMin,boundBox_.ZMin)
    
    return boundObj, boundBoxLocation
    

def createCylinder(boundBox_, nameLabel):
    boundObj = fc.ActiveDocument.addObject('Part::Cylinder', nameLabel + "_BoundCylinder")
    boundObj.Height = boundBox_.ZLength
    boundObj.Radius = ((boundBox_.XLength ** 2 + boundBox_.YLength ** 2) ** 0.5) / 2.0
    boundBoxLocation = fc.Vector(boundBox_.Center.x, boundBox_.Center.y, boundBox_.ZMin)

    return boundObj, boundBoxLocation


def createSphere(boundBox_, nameLabel):
    boundObj = fc.ActiveDocument.addObject('Part::Sphere', nameLabel + "_BoundSphere")
    boundObj.Radius = boundBox_.DiagonalLength / 2.0
    boundBoxLocation = boundBox_.Center

    return boundObj, boundBoxLocation


def createBoundAbstract(obj, createPrimitive = createBox):

    obj = first_object_with_volume(obj)

    if hasattr(obj, "Shape"):
        s = obj.Shape
    elif hasattr(obj, "Mesh"):      # upgrade with wmayer thanks #http://forum.freecadweb.org/viewtopic.php?f=13&t=22331
        s = obj.Mesh
    elif hasattr(obj, "Points"):
        s = obj.Points

    boundObj = False
    try:
        # LineColor
        red   = 1.0  # 1 = 255
        green = 1.0  #
        blue  = 0.4  #
    
        # boundBox
        boundBox_    = s.BoundBox
        boundBoxLX   = boundBox_.XLength
        boundBoxLY   = boundBox_.YLength
        boundBoxLZ   = boundBox_.ZLength
        boundBoxXMin = boundBox_.XMin
        boundBoxYMin = boundBox_.YMin
        boundBoxZMin = boundBox_.ZMin

        nameLabel  = obj.Label

        try:
            import unicodedata    
            nameLabel = str(unicodedata.normalize('NFKD', nameLabel).encode('ascii','ignore'))[2:]
        except Exception:
            None

        fc.Console.PrintMessage(str(boundBox_)+"\r\n")
        fc.Console.PrintMessage("Rectangle      : "+str(boundBoxLX)+" x "+str(boundBoxLY)+" x "+str(boundBoxLZ)+"\r\n")
        
        if (boundBoxLX > 0) and (boundBoxLY > 0) and (boundBoxLZ > 0):  # Create Volume

            boundObj, boundBoxLocation = createPrimitive(boundBox_, nameLabel)

            boundObj.Placement = adjustedGlobalPlacement(obj, boundBoxLocation)

            boundObjGui = fcgui.ActiveDocument.getObject(boundObj.Name)
            boundObjGui.LineColor  = (red, green, blue)
            boundObjGui.PointColor = (red, green, blue)
            boundObjGui.ShapeColor = (red, green, blue)
            boundObjGui.LineWidth = 1
            boundObjGui.Transparency = 90
    
    except Exception:
        fc.Console.PrintError("Bad selection"+"\n")

    return boundObj


def get_placement(
        orienteer1: DO
        ) -> fc.Placement:
    """Return absolute coordinates of orienteer."""
    resolve_mode_resolve = 0 # 0 - absolute, 1 relative
    selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
    objects_placements = get_subobjects_and_placements(selection)
    objects, placements = zip(*objects_placements)
    orienteer1_placement = placements[objects.index(orienteer1)]

    return orienteer1_placement


def get_placements(
        orienteer1: DO,
        orienteer2: DO
        ) -> fc.Placement:
    """Return the transform from `lcs` to `obj`."""
    resolve_mode_resolve = 0 # 0 - absolute, 1 relative
    selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
    objects_placements = get_subobjects_and_placements(selection)
    objects, placements = zip(*objects_placements)
    orienteer1_placement = placements[objects.index(orienteer1)]
    orienteer2_placement = placements[objects.index(orienteer2)]

    return orienteer1_placement, orienteer2_placement
