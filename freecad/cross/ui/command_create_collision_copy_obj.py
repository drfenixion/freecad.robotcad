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
                ''
                '\n\nSelect Robot or Robot Links.\n'
                'An object will be created based on Real element.'
                ' This object will be then bound to the link as a collision.',
            ),
        }

    def Activated(self):
        createBoundObjects(createBoundFunc = createCollisionCopyObj)

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('CreateCollisionCopyObj', CreateCollisionCopyObj())
