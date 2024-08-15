import FreeCAD as fc
import FreeCADGui as fcgui

from ..gui_utils import tr
from ..freecadgui_utils import createBoundCylinder
from ..wb_gui_utils import createBoundObjects


class CylinderFromBoundingBoxCommand:
    def GetResources(self):
        return {
            'Pixmap': 'cylinder_from_bbox.svg',
            'MenuText': tr('Cylinder from bounding box'),
            'ToolTip': tr(
                'Add a z-aligned Part::Cylinder englobing the'
                ' bounding box of the selected objects.'
                '\n\nIf you select a link, an object based on Real element will be created. '
                'This object will then be bound to the link as a collision.',
            ),
        }

    def Activated(self):
        createBoundObjects(createBoundFunc = createBoundCylinder)


    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('CylinderFromBoundingBox', CylinderFromBoundingBoxCommand())
