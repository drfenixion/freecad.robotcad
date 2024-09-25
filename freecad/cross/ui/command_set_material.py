import FreeCAD as fc
import FreeCADGui as fcgui
import MaterialEditor

from ..gui_utils import tr
from ..wb_utils import is_robot_selected, is_link_selected
from ..freecad_utils import error


class _SetMaterialCommand:
    def GetResources(self):
        return {
            'Pixmap': 'set_material.svg',
            'MenuText': tr('Set the material to the whole robot or a link'),
            'ToolTip': tr(
                'Select a robot or a link and use this action to'
                ' select a material. The material will be used to'
                ' estimate the mass and moments of inertia',
            ),
        }

    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Calculate mass and inertia'))
        objs = fcgui.Selection.getSelection()

        # TODO: apply to all selected robots and links.

        if not objs:
            doc = fc.activeDocument()
        else:
            obj = objs[0]

        try:
            card_path = obj.MaterialCardPath
        except (KeyError, AttributeError):
            card_path = ''

        material_editor = MaterialEditor.MaterialEditor(card_path=card_path)
        result = material_editor.exec_()

        if not result:
            # on cancel button an empty dict is returned.
            return

        try:
            card_name = material_editor.cards[material_editor.card_path]
            density = material_editor.materials[material_editor.card_path]['Density']
        except (AttributeError, KeyError):
            card_name = ''
            density = ''

        if not density:
            error('Material without density. Choose other material or fill density.', True)
        elif fc.Units.Quantity(density,) <= 0.0:
            error('Material density must be stringly positive. Correct material density or choose another material.', True)

        obj.MaterialCardName = card_name
        # TODO: make MaterialCardPath portable.
        obj.MaterialCardPath = material_editor.card_path
        obj.MaterialDensity = density

        doc.recompute()
        doc.commitTransaction()

    def IsActive(self):
        return is_robot_selected() or is_link_selected()


fcgui.addCommand('SetMaterial', _SetMaterialCommand())
