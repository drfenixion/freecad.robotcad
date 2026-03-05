import FreeCAD as fc
import FreeCADGui as fcgui

from ..gui_utils import tr
from ..freecadgui_utils import createBoundYAlignedCylinder
from ..wb_gui_utils import createBoundObjects


class YAlignedCylinderFromBoundingBoxCommand:
    def GetResources(self):
        return {
            'Pixmap': 'cylinder_y_from_bbox.svg',
            'MenuText': tr('Create collision cylinder from bounding box'),
            'ToolTip': tr(
                'Add a y-aligned Part::Cylinder englobing the'
                ' bounding box of the selected objects.'
                '\n\nSelect Robot or Robot Links.\n'
                'An object will be created based on Real element.'
                ' This object will be then bound to the link as a collision.',
            ),
        }

    def Activated(self):
        createBoundObjects(createBoundFunc = createBoundYAlignedCylinder)


    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('YAlignedCylinderFromBoundingBox', YAlignedCylinderFromBoundingBoxCommand())
