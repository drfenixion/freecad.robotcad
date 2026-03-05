import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecadgui_utils import createBoundBox
from ..gui_utils import tr
from ..wb_gui_utils import createBoundObjects
from typing import List
DO = fc.DocumentObject
DOList = List[DO]


class BoxFromBoundingBoxCommand:
    def GetResources(self):
        return {
            'Pixmap': 'box_from_bbox.svg',
            'MenuText': tr('Create collision box from bounding box'),
            'ToolTip': tr(
                'Add a Part::Cube corresponding to the'
                ' bounding box of the selected objects.'
                '\n\nSelect Robot or Robot Links.\n'
                'An object will be created based on Real element.'
                ' This object will be then bound to the link as a collision.',
            ),
        }

    def Activated(self):
        createBoundObjects(createBoundFunc = createBoundBox)

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('BoxFromBoundingBox', BoxFromBoundingBoxCommand())
