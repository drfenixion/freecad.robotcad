import FreeCAD as fc
import FreeCADGui as fcgui

from ..gui_utils import tr
from ..freecadgui_utils import createBoundYAlignedCylinder
from ..wb_gui_utils import createBoundObjects


class YAlignedCylinderFromBoundingBoxCommand:
    def GetResources(self):
        return {
            'Pixmap': 'cylinder_y_from_bbox.svg',
            'MenuText': tr('Cylinder from bounding box'),
            'ToolTip': tr(
                'Add a y-aligned Part::Cylinder englobing the'
                ' bounding box of the selected objects.'
                '\n\nIf you select a link, an object based on Real element will be created. '
                'This object will then be bound to the link as a collision.',
            ),
        }

    def Activated(self):
        createBoundObjects(createBoundFunc = createBoundYAlignedCylinder)


    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('YAlignedCylinderFromBoundingBox', YAlignedCylinderFromBoundingBoxCommand())
