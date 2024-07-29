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


def get_transform(
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


class _SetCROSSPlacementInAbsoluteCoordinatesCommand:
    """Command to set the placement of a Link or a Joint.

    Command to set the mounted placement of a Link or the Origin of a Joint in absolute coordinates
    (improved version of Set Placement).

    """

    def GetResources(self):
        return {'Pixmap': 'set_cross_placement_in_absolute_coordinates.svg',
                'MenuText': tr('Set placement - in absolute coordinates'),
                'Accel': 'S, P',
                'ToolTip': tr('Set the mounted placement of a link or the origin of a joint.\n'
                              'Select either\n'
                              '  a) a CROSS::Link, a LCS, and any orienteer (LCS or other)\n'
                              '  b) a CROSS::Joint, the child LCS, and the'
                              ' parent LCS on the same link.\n'
                              '  c) a CROSS::Joint, the LCS on the parent link,'
                              ' and the LCS on the child link.\n'
                              '\n'
                              'This will move origin/mounted placement to position\n'
                              'that equal current position + difference between orienteers.\n'
                              'That give you opotunity to move or bind element not only by it origin (zero coordinates) \n'
                              'but also by orienteer at any position.'
                              'F.e. if you select: joint, LCS somethere at this joint link and any LCS where you want\n'
                              'it will move first LCS to second LCS position and origin of joint respectively to this move\n'
                              ' (like first LCS and origin in binded system).  \n'
                              '\n'
                              'Uses absolute coordinates and works in any case.\n'
                              'Use it instead of old Set placement tool (that works only in local frame).\n',
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
                    '  a) a CROSS::Link, a LCS, and any orienteer (LCS or other)\n'
                    '  b) a CROSS::Joint, the child LCS, and the'
                    ' parent LCS on the same link.\n'
                    '  c) a CROSS::Joint, the LCS on the parent link,'
                    ' and the LCS on the child link.\n',
                    gui=True)
            return

        if selection_link:
            placement1, placement2 = get_transform(orienteer1, orienteer2)
            doc.openTransaction(tr("Set link's mounted placement"))

            old_mounted_placement = cross_link.MountedPlacement
            cross_link.MountedPlacement = fc.Placement(fc.Vector(0,0,0), fc.Rotation(0,0,0), fc.Vector(0,0,0)) # set zero origin
            doc.recompute() # trigger compute element placement based on zero origin
            cross_link_basic_placement = cross_link.Placement
            cross_link.MountedPlacement = old_mounted_placement
            doc.recompute()

            cross_link.MountedPlacement = cross_link_basic_placement.inverse() * placement2
            doc.recompute()

            placement1_new, placement2_new = get_transform(orienteer1, orienteer2)
            placements_new_diff = placement1_new.inverse() * placement2_new
            cross_link.MountedPlacement = cross_link.MountedPlacement * placements_new_diff

            doc.commitTransaction()
        elif selection_joint:
            placement1, placement2 = get_transform(orienteer1, orienteer2)
            doc.openTransaction(tr("Set joint's origin"))

            old_origin = cross_joint.Origin
            cross_joint.Origin = fc.Placement(fc.Vector(0,0,0), fc.Rotation(0,0,0), fc.Vector(0,0,0)) # set zero origin
            doc.recompute() # trigger compute element placement based on zero origin
            cross_joint_basic_placement = cross_joint.Placement
            cross_joint.Origin = old_origin
            doc.recompute()

            cross_joint.Origin = cross_joint_basic_placement.inverse() * placement2
            doc.recompute()

            placement1_new, placement2_new = get_transform(orienteer1, orienteer2)
            placements_new_diff = placement1_new.inverse() * placement2_new
            cross_joint.Origin = cross_joint.Origin * placements_new_diff

            doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('SetCROSSPlacementInAbsoluteCoordinates', _SetCROSSPlacementInAbsoluteCoordinatesCommand())
