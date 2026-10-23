"""Calculate mass, center of mass and inertia of a single selected object.

Uses the "advanced" inertia calculation method (see `inertia_calculator.py`):
materials of the object (and of the parts it contains) and the optional
RobotCad override properties are used. The result is displayed in a
non-modal dockable window (and printed to the console).

"""

from __future__ import annotations

import itertools
import math
import traceback
from typing import Optional

import numpy as np
from numpy.typing import NDArray

import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecad_utils import error
from ..gui_utils import tr
from ..wb_utils import is_joint, is_link, is_robot
from .calculate_object_mass_and_inertia_dialog import CalculateObjectMassAndInertiaDialog
from .calculate_object_mass_and_inertia_dialog import FRAME_GLOBAL, FRAME_LOCAL
from .inertia_calculator import LOG_LEVEL_INFO
from .inertia_calculator import calculate, log_message, mass_com_inertia, transform_to_frame

DO = fc.DocumentObject

_FRAME_DESCRIPTIONS = {
    FRAME_GLOBAL: 'global frame (document root)',
    FRAME_LOCAL: 'local frame of the object',
}

def _format_matrix(M: NDArray) -> list[str]:
    return ['    [' + ', '.join(f'{v: .6e}' for v in row) + ' ]' for row in M]

def _format_fc_rotational_matrix(M: fc.Matrix) -> list[str]:
    return ['    [' + ', '.join(f'{v: .6e}' for v in M.row(k)) + ' ]' for k in range(3)]

def principal_axes(inertia: NDArray) -> tuple[NDArray, fc.Rotation]:
    """Return the principal moments and the principal axes of an inertia matrix.

    Parameters
    ----------
    - inertia: 3x3 symmetric inertia matrix about the center of mass.

    Return `(moments, rotation)`: the principal moments in ascending order and
    the rotation whose x, y and z axes are the corresponding principal axes,
    expressed in the frame of `inertia`, i.e.
    `inertia = R @ diag(moments) @ R.T` with `R` the matrix of `rotation`.
    The axes form a right-handed frame. The sign of each axis is free; the
    rotation with the smallest angle is returned (identity for a diagonal
    matrix with ascending moments). For equal moments the axes are not unique.

    """
    J = np.asarray(inertia, dtype=float)
    moments, Q = np.linalg.eigh(0.5 * (J + J.T))
    # The proper rotation (det = +1) with the largest trace, i.e. the
    # smallest rotation angle, among the sign choices of the axes.
    best = None
    for signs in itertools.product((1.0, -1.0), repeat=3):
        R = Q * np.array(signs)
        if np.linalg.det(R) > 0.0 and (best is None or np.trace(R) > np.trace(best)):
            best = R
    R = best
    rotation = fc.Rotation(fc.Matrix(
        R[0, 0], R[0, 1], R[0, 2], 0.0,
        R[1, 0], R[1, 1], R[1, 2], 0.0,
        R[2, 0], R[2, 1], R[2, 2], 0.0,
        0.0, 0.0, 0.0, 1.0,
    ))
    return moments, rotation


def _rounded(x: float) -> float:
    """`x` rounded to 6 decimals, without negative zero (e.g. -1e-17 -> 0.0)."""
    return round(x, 6) + 0.0


def _format_vector(v: fc.Vector) -> str:
    return f'({_rounded(v.x): .6f}, {_rounded(v.y): .6f}, {_rounded(v.z): .6f})'


def format_inertial_properties(
        mass: float,
        center_of_mass: fc.Vector,
        inertia: NDArray,
        header: str = '',
        axes_rotation: Optional[fc.Rotation] = None,
) -> str:
    """Return mass, center of mass and inertia matrix with units as multi-line text.

    Parameters
    ----------
    - mass: mass in kg.
    - center_of_mass: center of mass in mm.
    - inertia: 3x3 inertia matrix about the center of mass in kg*mm^2.
    - header: first line of the text.
    - axes_rotation: if given, `inertia` is expressed in the principal axes
                     of inertia (see `principal_axes()`) and `axes_rotation`
                     is the rotation from the axes of the frame of
                     `center_of_mass` to the principal axes; it is printed
                     before the matrix.

    """
    lines = []
    if header:
        lines.append(header)
    lines.append(f'  Mass: {mass:.6g} kg')
    lines.append(
        f'  Center of mass: x = {center_of_mass.x:.6g} mm,'
        f' y = {center_of_mass.y:.6g} mm, z = {center_of_mass.z:.6g} mm',
    )
    if axes_rotation is None:
        lines.append( '  Inertia matrix about the center of mass, kg*mm^2:')
    else:
        yaw, pitch, roll = (_rounded(angle) for angle in axes_rotation.toEuler())
        lines.append( '  Rotation to principal axes:')
        lines += _format_fc_rotational_matrix(axes_rotation.toMatrix())
        lines.append(f'    axis {_format_vector(axes_rotation.Axis)}, angle {_rounded(math.degrees(axes_rotation.Angle)):.6g} deg,')
        lines.append(f'    yaw-pitch-roll: {yaw:.6g}, {pitch:.6g}, {roll:.6g} deg.')
        lines.append( '  Inertia matrix about the center of mass in the principal axes, kg*mm^2:')
    lines += _format_matrix(inertia)
    return '\n'.join(lines)


def _selected_object_and_global_placement() -> tuple[Optional[DO], Optional[fc.Placement]]:
    """Return the single selected object and its placement in the global frame.

    The global placement includes the placement of the object and of all its
    containers (App::Part, Assembly...) from the root of the document. If a
    sub-element (face, edge...) is selected, the object it belongs to is used.
    Return (None, None) if not exactly one object is selected or if the
    selection path cannot be resolved.

    """
    selection = fcgui.Selection.getSelectionEx('', 0)
    if len(selection) != 1 or len(selection[0].SubElementNames) > 1:
        return None, None
    # `root` is the object selected in the tree (possibly a container),
    # `subname` the path from `root` to the selected object or sub-element,
    # e.g. 'Body.Box.' or 'Body.Box.Face6'. It is empty if `root` is selected.
    root = selection[0].Object
    subname = selection[0].SubElementNames[0] if selection[0].SubElementNames else ''

    if not subname:
        placement = root.Placement if hasattr(root, 'Placement') else fc.Placement()
        return root, placement

    # retType 2: (last object of the path, accumulated matrix, geometry). The
    # last object is the one owning the element if the path ends with an
    # element name. The matrix is the transform of that object in the frame of
    # the parent of `root`, i.e. it includes the placement of `root`.
    result = root.getSubObject(subname, 2)
    if not result:
        return None, None
    obj, matrix, _ = result
    if not isinstance(obj, fc.DocumentObject) or not isinstance(matrix, fc.Matrix):
        return None, None
    return obj, fc.Placement(matrix)


class _CalculationError(Exception):
    """The calculation cannot be done, the message is for the user."""


def _calculate_report(target_frame: str, use_principal_axes: bool, log_level: str) -> str:
    """Calculate the selected object and return the result as text.

    Raise `_CalculationError` if the selection is not valid or if the
    calculation fails.

    """
    # get object and its global placement
    obj, global_placement = _selected_object_and_global_placement()
    # check if object is valid
    if obj is None:
        raise _CalculationError('Select exactly one object.')
    if is_link(obj) or is_robot(obj) or is_joint(obj):
        raise _CalculationError(
            f'Object "{obj.Label}" is a CROSS Link, Robot or Joint.'
            ' Select a body or a container, or use "Calculate mass and inertia" tool.',
        )
    # calculate inertia properties
    log_message(log_level, LOG_LEVEL_INFO, 0, f'Calculate inertial properties of "{obj.Label}" ({obj.FullName}), frame: {_FRAME_DESCRIPTIONS[target_frame]}.')
    try:
        JJ = calculate(obj, ignore_placement=True, level=1, output_level=log_level)
        if target_frame == FRAME_GLOBAL:
            JJ = transform_to_frame(JJ, global_placement)
    except (ValueError, KeyError, TypeError) as e:
        raise _CalculationError(f'Object "{obj.Label}": {e}') from e
    except Exception as e:
        # Unexpected error (e.g. an object with an unusual structure): show it
        # in the window instead of losing it in the Qt slot.
        fc.Console.PrintError(traceback.format_exc())
        raise _CalculationError(f'Object "{obj.Label}": unexpected error: {e}') from e
    mass, center_of_mass, inertia = mass_com_inertia(JJ)
    # check result for sanity
    if mass <= 0.0:
        raise _CalculationError(
            f'Object "{obj.Label}": calculated mass is zero. Assign materials to the parts of the object'
            ' (or set the RobotCad override properties) and check the console log'
            f' (select the "{LOG_LEVEL_INFO}" log level).',
        )
    axes_rotation = None
    if use_principal_axes:
        moments, axes_rotation = principal_axes(inertia)
        inertia = np.diag(moments)
    return format_inertial_properties(
        mass,
        center_of_mass,
        inertia,
        header=(
            f'Selected body: "{obj.Label}" ({obj.FullName})\n'
            f'Inertia frame: {_FRAME_DESCRIPTIONS[target_frame]}'
        ),
        axes_rotation=axes_rotation,
    )


class _CalculateObjectMassAndInertiaCommand:
    def GetResources(self) -> dict:
        return {
            'Pixmap': 'calculate_object_mass_and_inertia.svg',
            'MenuText': tr('Calculate: mass and inertia of the selected object'),
            'ToolTip': tr(
                'Open the window to calculate mass and inertia of an object.\n'
                'Select one object (body, Part or Assembly container, link to a body)\n'
                'and press "Calculate" in the window. The window stays open: you can change\n'
                'the selection, assign materials and calculate again.\n'
                '\n'
                'Tool will calculate mass, center of mass and inertia matrix of the object and show result in the window.\n'
                'With "Principal axes" checked, the inertia matrix is expressed in the principal axes\n'
                'of inertia and the rotation from the target frame axes to them is shown.\n'
                'It utilizes "advanced" method (see robot "InertiaCalculationMethod" property for more information) which takes\n'
                'into account the materials assigned to objects and the RobotCad override properties\n'
                '(see "Override inertial properties" tool).\n'
                '\n'
                'See tool output log in Report View for more information.'
            ),
        }

    def IsActive(self) -> bool:
        # The window can be opened without selection, the selection is
        # checked when "Calculate" is pressed.
        return True

    def Activated(self) -> None:
        # The window is not modal and stays open: the calculation is done at
        # each click on "Calculate", for the object selected at that moment.
        CalculateObjectMassAndInertiaDialog.open(on_calculate=self._on_calculate)

    @staticmethod
    def _on_calculate(dialog: CalculateObjectMassAndInertiaDialog) -> None:
        try:
            text = _calculate_report(dialog.target_frame, dialog.principal_axes, dialog.log_level)
        except _CalculationError as e:
            dialog.show_error(str(e))
            error(str(e))
            return
        dialog.show_result(text)
        fc.Console.PrintMessage(text + '\n')


fcgui.addCommand('CalculateObjectMassAndInertia', _CalculateObjectMassAndInertiaCommand())
