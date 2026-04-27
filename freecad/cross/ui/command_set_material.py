import os
from pathlib import PurePath
import FreeCAD as fc
import FreeCADGui as fcgui
from freecad.cross.vendor.fc_material_editor import MaterialEditor as MaterialEditor_1_1
import MaterialEditor

from ..gui_utils import tr
from ..wb_utils import is_robot_selected, is_link_selected
from ..freecad_utils import error, getFreeCADversion


def read_fcmat_file(card_path):
    """Read a .FCMat file and return its contents as a dictionary."""
    if not card_path or not os.path.isfile(card_path):
        return {}
    
    # Try to use FreeCAD's importFCMat
    try:
        from importFCMat import read
        return read(card_path)
    except ImportError:
        # Fallback: try to parse the file manually
        result = {}
        with open(card_path, 'r') as f:
            for line in f:
                line = line.strip()
                if '=' in line and not line.startswith('#'):
                    key, value = line.split('=', 1)
                    result[key.strip()] = value.strip()
        return result


class _SetMaterialCommand:
    def GetResources(self):
        return {
            'Pixmap': 'set_material.svg',
            'MenuText': tr('Set the material to the whole robot or a link'),
            'ToolTip': tr(
                'Select a robot or a link and use this action to'
                ' select a material. The material will be used to'
                ' calculate mass and moments of inertia of the links.',
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
        # new editor
        #MaterialEditor.openEditor("SolidMaterial", "Material")

        major, minor = getFreeCADversion()

        if (major, minor) >= (1, 1):
            material_editor = MaterialEditor_1_1.MaterialEditor(card_path=card_path)
        else:
            material_editor = MaterialEditor.MaterialEditor(card_path=card_path)
        result = material_editor.exec_()

        if not result:
            # on cancel button an empty dict is returned.
            return

        # Read the material data directly from the selected .FCMat file
        card_path = material_editor.card_path
        material_data = read_fcmat_file(card_path)
        
        # Get card name from filename if not in the file
        if card_path:
            card_name = material_data.get('CardName', PurePath(card_path).stem)
        else:
            card_name = ''
        
        density = material_data.get('Density', '')

        if not density:
            error('Material without density. Choose other material or fill density.', True)
        elif fc.Units.Quantity(density) <= 0.0:
            error('Material density must be stringly positive. Correct material density or choose another material.', True)

        obj.MaterialCardName = card_name
        # TODO: make MaterialCardPath portable.
        obj.MaterialCardPath = card_path
        obj.MaterialDensity = density

        doc.recompute()
        doc.commitTransaction()

    def IsActive(self):
        return is_robot_selected() or is_link_selected()


fcgui.addCommand('SetMaterial', _SetMaterialCommand())