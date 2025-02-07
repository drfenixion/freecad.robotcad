from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from ..gui_utils import tr
from ..wb_utils import get_placement_of_orienteer, rotate_placement, set_placement_fast


# Stubs and type hints.
from ..joint import Joint
from ..link import Link
DO = fc.DocumentObject
CrossLink = Link
CrossJoint = Joint
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


class _SetCROSSPlacementFastSensorCommand:
    """Command to set "Set placement - fast", and after use axis orietation correction for sensor (front of sensor is positive x-oriented).
    """

    def GetResources(self):
        return {
            'Pixmap': 'set_cross_placement_fast_sensor.svg',
            'MenuText': tr('Set placement - sensor'),
            'Accel': 'P, S',
            'ToolTip': tr(
                'Set the Origin of a joint and Mounted Placement of link.\n'
                '\n'
                'Select (with Ctlr): \n'
                '    1) subelement (face, edge, vertex, LCS) of body (of Real) of robot link (first reference)\n'
                '    2) subelement (face, edge, vertex, LCS) of body (of Real) of robot link (second reference)\n'
                '\n'
                'Works same way as "Set placement - fast", and after uses axis orietation correction for sensor\n'
                '(front of sensor is positive x-oriented)\n',
            ),
        }

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())

    def Activated(self):
        doc = fc.activeDocument()
        joint, child_link, parent_link = set_placement_fast()

        doc.openTransaction(tr("Set placement - sensor"))
        # parent joint must be positive x-oriented (x axis is forward of sensor)
        x = 0
        y = 270
        z = 0
        # rotate joint
        joint.Origin = rotate_placement(joint.Origin, x, y, z)
        doc.recompute()
        # rotate link
        orienteer1_sub_obj, *_ = fcgui.Selection.getSelectionEx()
        orienteer2_placement = get_placement_of_orienteer(orienteer1_sub_obj, lcs_concentric_reversed = True)
        orienteer2_to_link_diff = child_link.Placement.inverse() * orienteer2_placement
        child_link.MountedPlacement = rotate_placement(child_link.MountedPlacement, x, y, z, orienteer2_to_link_diff.Base)
        doc.recompute()
        doc.commitTransaction()


fcgui.addCommand('SetCROSSPlacementFastSensor', _SetCROSSPlacementFastSensorCommand())
