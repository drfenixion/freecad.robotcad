import FreeCAD as fc
import FreeCADGui as fcgui

from ..gui_utils import tr
from ..freecadgui_utils import createBoundXAlignedCylinder
from ..wb_gui_utils import createBoundObjects


class XAlignedCylinderFromBoundingBoxCommand:
    def GetResources(self):
        return {
            'Pixmap': 'cylinder_x_from_bbox.svg',
            'MenuText': tr('Cylinder from bounding box'),
            'ToolTip': tr(
                'Add a x-aligned Part::Cylinder englobing the'
                ' bounding box of the selected objects.'
                '\n\nIf you select a link, an object based on Real element will be created. '
                'This object will then be bound to the link as a collision.',
            ),
        }

    def Activated(self):
        createBoundObjects(createBoundFunc = createBoundXAlignedCylinder)


    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('XAlignedCylinderFromBoundingBox', XAlignedCylinderFromBoundingBoxCommand())
