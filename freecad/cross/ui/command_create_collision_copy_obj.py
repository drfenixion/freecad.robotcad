import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecadgui_utils import createCollisionCopyObj
from ..gui_utils import tr
from ..wb_gui_utils import createBoundObjects
from typing import List
DO = fc.DocumentObject
DOList = List[DO]


class CreateCollisionCopyObj:
    def GetResources(self):
        return {
            'Pixmap': 'collision_copy_obj.svg',
            'MenuText': tr('Create collision as copy of object'),
            'ToolTip': tr(
                'Add a Part::Feature copies of the selected objects.'
                '.'
                '\n\nIf selected a link, an object based on Real element will be created. '
                'This object will then be bound to the link as a collision.',
            ),
        }

    def Activated(self):
        createBoundObjects(createBoundFunc = createCollisionCopyObj)

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('CreateCollisionCopyObj', CreateCollisionCopyObj())
