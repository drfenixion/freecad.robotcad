import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecad_utils import error
from ..freecad_utils import strip_subelement
from ..freecad_utils import warn
from ..gui_utils import tr
from ..placement_utils import get_global_placement


class SphereFromBoundingBoxCommand:
    def GetResources(self):
        return {
            'Pixmap': 'sphere_from_bbox.svg',
            'MenuText': tr('Sphere from bounding box'),
            'ToolTip': tr(
                'Add a Part::Cube corresponding to the'
                ' bounding box of the selected objects',
            ),
        }

    def Activated(self):
        is_one_object_compatible = False
        is_one_object_incompatible = False
        selection = fcgui.Selection.getSelectionEx('', 0)
        if not selection:
            # Should not happen.
            return
        for selection_object in selection:
            obj = selection_object.Object
            if selection_object.HasSubObjects:
                # Inside a part.
                subpath = selection_object.SubElementNames[0]
                try:
                    subobj = obj.getSubObjectList(subpath)[-1].getPropertyOfGeometry()
                except AttributeError:
                    is_one_object_incompatible = True
                    continue
                placement = get_global_placement(obj, subpath)
                # Cancel the rotation because bounding boxes are axis-aligned.
                placement.Rotation = fc.Rotation()
                sphere_name = obj.Label + '.' + strip_subelement(subpath) + '_bbox'
            else:
                # Outside of any part.
                try:
                    subobj = obj.getPropertyOfGeometry()
                except AttributeError:
                    is_one_object_incompatible = True
                    continue
                sphere_name = obj.Label + '_bbox'
                placement = fc.Placement()
            # Cf. https://github.com/pboechat/pyobb for oriented bounding-box.
            is_one_object_compatible = True
            bbox = subobj.BoundBox
            doc = fc.activeDocument()
            doc.openTransaction(tr('Sphere from bounding box'))
            sphere = doc.addObject('Part::Sphere', sphere_name)
            sphere.Radius = bbox.DiagonalLength / 2.0
            sphere.Placement.Base = bbox.Center
            sphere.Placement = placement * sphere.Placement
            doc.commitTransaction()
            doc.recompute()
        if not is_one_object_compatible:
            error(tr('No compatible object selected'), gui=True)
        if is_one_object_incompatible:
            warn(tr('One or more incompatible object selected'), gui=True)

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('SphereFromBoundingBox', SphereFromBoundingBoxCommand())
