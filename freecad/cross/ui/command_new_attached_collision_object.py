import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import warn
from ..gui_utils import tr


class _NewAttachedCollisionObjectCommand:
    """The command definition to create a new AttachedCollisionObject object."""

    def GetResources(self):
        return {
            'Pixmap': 'attached_collision_object.svg',
            'MenuText': tr('New AttachedCollisionObject'),
            'Accel': 'N, A',
            'ToolTip': tr('Create an AttachedCollisionObject container.'),
        }

    def IsActive(self):
        return True

    def Activated(self):
        doc = fc.activeDocument()
        if not doc:
            doc = fc.newDocument()
        if not doc:
            warn('No active document and cannot create a new document, doing nothing', True)
        doc.openTransaction(tr('Create AttachedCollisionObject'))
        fcgui.doCommand('')
        fcgui.addModule('freecad.cross.attached_collision_object_proxy')
        fcgui.doCommand("_attached_collision_object = freecad.cross.attached_collision_object_proxy.make_attached_collision_object('AttachedCollisionObject')")
        # fcgui.doCommand('FreeCADGui.ActiveDocument.setEdit(_attached_collision_object.Name)')
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('NewAttachedCollisionObject', _NewAttachedCollisionObjectCommand())
