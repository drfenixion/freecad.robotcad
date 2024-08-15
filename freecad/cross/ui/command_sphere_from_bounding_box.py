import FreeCAD as fc
import FreeCADGui as fcgui

from ..gui_utils import tr
from ..freecadgui_utils import createBoundSphere
from ..wb_gui_utils import createBoundObjects


class SphereFromBoundingBoxCommand:
    def GetResources(self):
        return {
            'Pixmap': 'sphere_from_bbox.svg',
            'MenuText': tr('Sphere from bounding box'),
            'ToolTip': tr(
                'Add a Part::Sphere corresponding to the'
                ' bounding box of the selected objects.'
                '\n\nIf you select a link, an object based on Real element will be created. '
                'This object will then be bound to the link as a collision.',
            ),
        }

    def Activated(self):
        createBoundObjects(createBoundFunc = createBoundSphere)

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('SphereFromBoundingBox', SphereFromBoundingBoxCommand())
