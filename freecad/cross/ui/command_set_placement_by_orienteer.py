from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecadgui_utils import get_subobjects_and_placements
from ..gui_utils import tr


# Stubs and type hints.
from ..joint import Joint
from ..link import Link
DO = fc.DocumentObject
CrossLink = Link
CrossJoint = Joint
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


def get_placement(
        orienteer1: DO,
        ) -> fc.Placement:
    """Return absolute coordinates of orienteer."""
    resolve_mode_resolve = 0 # 0 - absolute, 1 relative
    selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
    objects_placements = get_subobjects_and_placements(selection)
    objects, placements = zip(*objects_placements)
    orienteer1_placement = placements[objects.index(orienteer1)]

    return orienteer1_placement


def set_local_placement(doc: DO, link_or_joint: DO, origin_or_mounted_placement_name: str, orienteer1: DO):
    """Set element (joint or link) local placement (Origin or Mounted placement) by orienteer.

    Set local placement with orienteer placement value
    """

    placement1 = get_placement(orienteer1)
    
    # prepare data
    origin_or_mounted_placement_name__old = getattr(link_or_joint, origin_or_mounted_placement_name)
    setattr(link_or_joint, origin_or_mounted_placement_name, fc.Placement(fc.Vector(0,0,0), fc.Rotation(0,0,0), fc.Vector(0,0,0)))  # set zero Origin
    doc.recompute() # trigger compute element placement based on zero Origin
    element_basic_placement = getattr(link_or_joint, 'Placement')
    setattr(link_or_joint, origin_or_mounted_placement_name, origin_or_mounted_placement_name__old)
    doc.recompute()

    ## prepare data
    placement1_diff = element_basic_placement.inverse() * placement1

    # Set Origin or Mounted placement
    setattr(link_or_joint, origin_or_mounted_placement_name, placement1_diff)


class _SetCROSSPlacementByOrienteerCommand:
    """Command to set the placement of a Link or a Joint.

    Command to set the mounted placement of a Link or the Origin by orienteer placement

    """

    def GetResources(self):
        return {'Pixmap': 'set_cross_placement_by_orienteer.svg',
                'MenuText': tr('Set placement - by orienteer'),
                'Accel': 'P, O',
                'ToolTip': tr('Set the mounted placement of a link or the origin of a joint.\n'
                              'Select either:\n'
                              '  a) a CROSS::Link, any (orienteer) \n'
                              '  b) a CROSS::Joint, any (orienteer) \n'
                              '\n'
                              'This will set Origin or Mounted placement with orienteer placement.\n'
                              'LCS is convenient as orienteer, because of configurable orientation.\n'
                              'Use this when you want just move Origin or Mounted placement to some placement.\n'
                              )}

    def IsActive(self):
        return True

    def Activated(self):
        doc = fc.activeDocument()
        selection_ok = False
        selection_link = False
        selection_joint = False
        try:
            cross_link, orienteer1 = validate_types(
                fcgui.Selection.getSelection(),
                ['Cross::Link', 'Any'])
            selection_ok = True
            selection_link = True
        except RuntimeError:
            pass

        if not selection_ok:
            try:
                cross_joint, orienteer1 = validate_types(
                    fcgui.Selection.getSelection(),
                    ['Cross::Joint', 'Any'])
                selection_ok = True
                selection_joint = True
            except RuntimeError:
                pass

        if not selection_ok:
            message('Select either\n'
                    '  a) a CROSS::Link, any (orienteer) \n'
                    '  b) a CROSS::Joint, any (orienteer) \n',
                    gui=True)
            return

        if selection_link:
            doc.openTransaction(tr("Set link's mounted placement"))
            set_local_placement(doc, cross_link, 'MountedPlacement', orienteer1)
            doc.commitTransaction()
        elif selection_joint:
            doc.openTransaction(tr("Set joint's origin"))
            set_local_placement(doc, cross_joint, 'Origin', orienteer1)
            doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('SetCROSSPlacementByOrienteer', _SetCROSSPlacementByOrienteerCommand())
