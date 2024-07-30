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


def get_placements(
        orienteer1: DO,
        orienteer2: DO,
        ) -> fc.Placement:
    """Return the transform from `lcs` to `obj`."""
    resolve_mode_resolve = 0 # 0 - absolute, 1 relative
    selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
    objects_placements = get_subobjects_and_placements(selection)
    objects, placements = zip(*objects_placements)
    orienteer1_placement = placements[objects.index(orienteer1)]
    orienteer2_placement = placements[objects.index(orienteer2)]

    return orienteer1_placement, orienteer2_placement


def move_local_placement(doc: DO, link_or_joint: DO, origin_or_mounted_placement_name: str, orienteer1: DO, orienteer2: DO):
    """Move element (joint or link) local placement (Origin or Mounted placement).

    Move first orienteer to placement of second and second orienteer
    and element to positions relative their bind system (element and orienteers) before.
    This move does not change spatial relation between each other.
    """

    placement1, placement2 = get_placements(orienteer1, orienteer2)
    
    # prepare data
    origin_or_mounted_placement_name__old = getattr(link_or_joint, origin_or_mounted_placement_name)
    setattr(link_or_joint, origin_or_mounted_placement_name, fc.Placement(fc.Vector(0,0,0), fc.Rotation(0,0,0), fc.Vector(0,0,0)))  # set zero Origin
    doc.recompute() # trigger compute element placement based on zero Origin
    element_basic_placement = getattr(link_or_joint, 'Placement')
    setattr(link_or_joint, origin_or_mounted_placement_name, origin_or_mounted_placement_name__old)
    doc.recompute()

    ## prepare data
    element_local_placement = getattr(link_or_joint, origin_or_mounted_placement_name)
    origin_placement1_diff = (element_basic_placement * element_local_placement).inverse() * placement1
    origin_placement2_diff = (element_basic_placement * element_local_placement).inverse() * placement2

    # do Origin move
    # first orienteer come to second orienteer place and Origin respectively moved
    # in local frame every tool click will result Origin move because both orienteers moved and received new position
    new_local_placement = element_local_placement * origin_placement2_diff * origin_placement1_diff.inverse()
    setattr(link_or_joint, origin_or_mounted_placement_name, new_local_placement)


class _SetCROSSPlacementInAbsoluteCoordinatesCommand:
    """Command to set the placement of a Link or a Joint.

    Command to set the mounted placement of a Link or the Origin of a Joint in absolute coordinates
    (improved version of Set Placement).

    """

    def GetResources(self):
        return {'Pixmap': 'set_cross_placement_in_absolute_coordinates.svg',
                'MenuText': tr('Set placement - in absolute coordinates'),
                'Accel': 'P, A',
                'ToolTip': tr('Set the mounted placement of a link or the origin of a joint.\n'
                              'Select either:\n'
                              '  a) a CROSS::Link, a local frame LCS (first orienteer), any (second orienteer) \n'
                              '  b) a CROSS::Joint, a local frame LCS (first orienteer), a LCS (second orienteer).\n'
                              '\n'
                              'This will move first orienteer to position of second orienteer\n'
                              'and binded system (first orienteer + Link or Joint) will moved respectively. \n'
                              '\n'
                              'It uses absolute coordinates and works correct in any case.\n'
                              'Use this tool instead of old Set placement tool (that works correct only in local frame and have bug with rotated LCS).\n',
                              )}

    def IsActive(self):
        return True

    def Activated(self):
        doc = fc.activeDocument()
        selection_ok = False
        selection_link = False
        selection_joint = False
        try:
            cross_link, orienteer1, orienteer2 = validate_types(
                fcgui.Selection.getSelection(),
                ['Cross::Link', 'PartDesign::CoordinateSystem', 'Any'])
            selection_ok = True
            selection_link = True
        except RuntimeError:
            pass

        if not selection_ok:
            try:
                cross_joint, orienteer1, orienteer2 = validate_types(
                    fcgui.Selection.getSelection(),
                    ['Cross::Joint', 'PartDesign::CoordinateSystem', 'PartDesign::CoordinateSystem'])
                selection_ok = True
                selection_joint = True
            except RuntimeError:
                pass

        if not selection_ok:
            message('Select either\n'
                    '  a) a CROSS::Link, a local frame LCS (first orienteer), any (second orienteer) \n'
                    '  b) a CROSS::Joint, a local frame LCS (first orienteer), a LCS (second orienteer).\n',
                    gui=True)
            return

        if selection_link:
            doc.openTransaction(tr("Set link's mounted placement"))
            move_local_placement(doc, cross_link, 'MountedPlacement', orienteer1, orienteer2)
            doc.commitTransaction()
        elif selection_joint:
            doc.openTransaction(tr("Set joint's origin"))
            move_local_placement(doc, cross_joint, 'Origin', orienteer1, orienteer2)
            doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('SetCROSSPlacementInAbsoluteCoordinates', _SetCROSSPlacementInAbsoluteCoordinatesCommand())
