"""Calculate mass, center of mass and inertia of a single selected object.

Uses the "advanced" inertia calculation method (see `inertia_calculator.py`):
materials of the object (and of the parts it contains) and the optional
RobotCad override properties are used. The result is printed to the console.

"""

from __future__ import annotations

from typing import Optional

import numpy as np
from numpy.typing import NDArray

import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecad_utils import error
from ..gui_utils import tr
from ..wb_utils import is_joint, is_link, is_robot
from .calculate_object_mass_and_inertia_dialog import CalculateObjectMassAndInertiaDialog
from .calculate_object_mass_and_inertia_dialog import FRAME_GLOBAL, FRAME_LOCAL, TARGET_FRAMES
from .inertia_calculator import LOG_LEVEL_INFO, LOG_LEVEL_WARN
from .inertia_calculator import calculate, log_message, mass_com_inertia, transform_to_frame

DO = fc.DocumentObject

_FRAME_DESCRIPTIONS = {
    FRAME_GLOBAL: 'global frame (document root)',
    FRAME_LOCAL: 'local frame of the object',
}

def _format_matrix(M: NDArray) -> list[str]:
    return ['    [' + ', '.join(f'{v: .6e}' for v in row) + ' ]' for row in M]


def format_inertial_properties(
        mass: float,
        center_of_mass: fc.Vector,
        inertia: NDArray,
        header: str = '',
) -> str:
    """Return mass, center of mass and inertia matrix with units as multi-line text.

    Parameters
    ----------
    - mass: mass in kg.
    - center_of_mass: center of mass in mm.
    - inertia: 3x3 inertia matrix about the center of mass in kg*mm^2.
    - header: first line of the text.

    """
    lines = []
    if header:
        lines.append(header)
    lines.append(f'  Mass: {mass:.6g} kg')
    lines.append(
        f'  Center of mass: x = {center_of_mass.x:.6g} mm,'
        f' y = {center_of_mass.y:.6g} mm, z = {center_of_mass.z:.6g} mm',
    )
    lines.append('  Inertia matrix about the center of mass, kg*mm^2:')
    lines += _format_matrix(inertia)
    return '\n'.join(lines)


def _selected_object_and_global_placement() -> tuple[Optional[DO], Optional[fc.Placement]]:
    """Return the single selected object and its placement in the global frame.

    The global placement includes the placement of the object and of all its
    containers (App::Part, Assembly...) from the root of the document.
    Return (None, None) if not exactly one object is selected.

    """
    selection = fcgui.Selection.getSelectionEx('', 0)
    if len(selection) != 1 or len(selection[0].SubElementNames) > 1:
        return None, None
    root = selection[0].Object
    subname = selection[0].SubElementNames[0] if selection[0].SubElementNames else ''
    # Remove a selected face, edge or vertex: 'Body.Box.Face6' -> 'Body.Box.'.
    if subname and not subname.endswith('.'):
        subname = subname.rsplit('.', maxsplit=1)[0] + '.' if '.' in subname else ''

    if not subname:
        placement = root.Placement if hasattr(root, 'Placement') else fc.Placement()
        return root, placement

    # Cf. DocumentObjectPyImp.cpp::getSubObject: retType 0 is the object,
    # retType 3 is the placement of the sub-object in the frame of the
    # parent of `root`, i.e. including the placement of `root`.
    obj = root.getSubObject(subname, 0)
    placement = root.getSubObject(subname, 3)
    if obj is None or not isinstance(placement, fc.Placement):
        return None, None
    return obj, placement


class _CalculateObjectMassAndInertiaCommand:
    def GetResources(self) -> dict:
        return {
            'Pixmap': 'calculate_mass_and_inertia.svg',
            'MenuText': tr('Calculate: mass and inertia of the selected object'),
            'ToolTip': tr(
                'Select one object (body, Part or Assembly container, link to a body and press this button.\n'
                '\n'
                'Tool will calculate mass, center of mass and inertia matrix of the object and print result in Report View.\n'
                'It utilizes "advanced" method (see "InertiaCalculationMethod" property for more information) which takes\n'
                'in account the materials assigned to objects and RobotCad Override Inertial Properties.\n'
                '\n'
                'See tool output log in Report View for more information.'
            ),
        }

    def IsActive(self) -> bool:
        return len(fcgui.Selection.getSelection()) == 1

    def Activated(self) -> None:
        # get object and it global placement
        obj, global_placement = _selected_object_and_global_placement()
        # check if object is valid
        if obj is None:
            error('Select exactly one object (not a sub-element).', gui=True)
            return
        if is_link(obj) or is_robot(obj) or is_joint(obj):
            error(
                f'Object "{obj.Label}" is a CROSS Link, Robot or Joint.'
                ' Select a body or a container, or use "Calculate mass and inertia" tool.',
                gui=True,
            )
            return
        # request parameters
        dialog = CalculateObjectMassAndInertiaDialog(f'{obj.Label} ({obj.Name})')
        if not dialog.exec():
            return
        # calculate inertia properties
        log_message(dialog.log_level, LOG_LEVEL_INFO, 0, f'Calculate inertial properties of "{obj.Label}" ({obj.FullName}), frame: {_FRAME_DESCRIPTIONS[dialog.target_frame]}.')
        try:
            JJ = calculate(obj, ignore_placement=True, level=1, output_level=dialog.log_level)
            if dialog.target_frame == FRAME_GLOBAL:
                JJ = transform_to_frame(JJ, global_placement)
        except (ValueError, KeyError, TypeError) as e:
            error(f'Object "{obj.Label}": {e}', gui=True)
            return
        mass, center_of_mass, inertia = mass_com_inertia(JJ)
        # check result for sanity
        if mass <= 0.0:
            error(f'Object "{obj.Label}": calculated mass is zero. Assign materials to the parts of the object (or set the RobotCad override properties)  and check the console log (select the "{LOG_LEVEL_INFO}" log level).', gui=True,) 
            return
        # print result
        fc.Console.PrintMessage(format_inertial_properties(mass, center_of_mass, inertia, header=f'Mass and inertia of "{obj.Label}" ({obj.FullName}), expressed in the {_FRAME_DESCRIPTIONS[dialog.target_frame]}:') + '\n')


fcgui.addCommand('CalculateObjectMassAndInertia', _CalculateObjectMassAndInertiaCommand())
