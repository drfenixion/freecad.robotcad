import FreeCAD as fc
import FreeCADGui as fcgui
import MaterialEditor

from ..gui_utils import tr
from ..wb_utils import is_robot_selected, is_link_selected


class SetMaterialCommand:
    def GetResources(self):
        return {'Pixmap': 'set_material.svg',
                'MenuText': tr('Set material to whole robot or link'),
                'ToolTip': tr('Select robot or link and press this bottom for selection of material. Material will be used for calculating mass and inertia based on material density'),
                }
    
    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Calculate mass and inertia'))
        objs = fcgui.Selection.getSelection()
        if not objs:
            doc = fc.activeDocument()
        else:
            obj = objs[0]
        
        try:
            card_path = obj.MaterialCardPath
        except (KeyError, AttributeError):
            card_path = ''
        
        materialEditor = MaterialEditor.MaterialEditor(card_path=card_path)
        result = materialEditor.exec_()

        if result != True:
            # on cancel button an empty dict is returned
            return
    
        try:
            card_name = materialEditor.cards[materialEditor.card_path]
            density = materialEditor.materials[materialEditor.card_path]['Density']
        except (KeyError, AttributeError):
            card_name = ''
            density = ''
        
        obj.MaterialCardName = card_name
        obj.MaterialCardPath = materialEditor.card_path
        obj.MaterialDensity = density

        doc.recompute()
        doc.commitTransaction()

    def IsActive(self):
        return is_robot_selected() or is_link_selected()


fcgui.addCommand('SetMaterial', SetMaterialCommand())
