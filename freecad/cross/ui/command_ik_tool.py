"""
command_ik_tool.py  -  RobotCAD workbench command
==================================================

SHORT DESCRIPTION (shown in FreeCAD menu / toolbar tooltip)
------------------------------------------------------------
Compute Forward and Inverse Kinematics for the selected robot directly
from RobotCAD native structures, with a live 3-D animation of the result.

DEVELOPER DESCRIPTION
---------------------
This module registers the FreeCAD Gui command ``IKTool`` inside the RobotCAD
workbench.  When the user clicks the toolbar icon (or presses I, K) with a
Cross::Robot object selected, a modal dialog opens:

IKToolDialog
├── Target group  - three QDoubleSpinBox fields (X, Y, Z in millimetres)
│                   pre-filled with the current FK end-effector position
├── [Solve IK]    - runs RoboTool iterative Jacobian pseudo-inverse solver
├── Results table - one row per active joint:
│     columns: Joint name | Original (rad/mm) | Solved (rad/mm)
├── Status label  - convergence message + achieved position + error
├── [Apply to Model]   - writes solved_joint_positions → joint_obj.Position, recomputes
│                        document → live 3-D animation
├── [Restore Original] - writes original_joint_positions back → restores initial pose
└── [Close]

State management
~~~~~~~~~~~~~~~~
original_joint_positions is captured once (on the first Solve click) so the user can
always return to the pose that existed before the dialog was opened.
Subsequent Solve calls do NOT overwrite original_joint_positions.

Integration with RobotCAD
~~~~~~~~~~~~~~~~~~~~~~~~~
- Reads the robot via  robot_obj.Proxy.get_joints() / get_links()
  (standard RobotCAD Proxy API).
- joint_obj.Position  - float in radians (revolute/continuous) or
  millimetres (prismatic).  Read and written directly.
- Units: all Cartesian coordinates in millimetres; joint angles in radians.
- doc.openTransaction / commitTransaction wraps every write so the action
  is undoable with Ctrl-Z.
- doc.recompute() + fcgui.updateGui() after every write triggers the live
  animation in the 3-D viewport.

RoboTool modules used
~~~~~~~~~~~~~~~~~~~~~
from ..kinematics.models             import Robot, Joint, Link
from ..kinematics.kinematics         import forward_kinematics
from ..kinematics.inverse_kinematics import inverse_kinematics

These must live under  freecad/cross/kinematics/  (i.e. your RoboTool
engine packaged inside RobotCAD).

File location (suggested)
~~~~~~~~~~~~~~~~~~~~~~~~~
freecad/cross/ui/command_ik_tool.py

Corresponding icon
~~~~~~~~~~~~~~~~~~
freecad/cross/resources/ik_solver.svg   (64 * 64 px, transparent background)

Author  : RoboTool / RobotCAD integration
License : MIT
"""

from __future__ import annotations

import math
import io
from contextlib import redirect_stdout

import numpy as np
import FreeCAD as fc
import FreeCADGui as fcgui

# FreeCAD 1.0 conda env ships PySide6; system FreeCAD may use PySide2.
# Import whichever is available so the command works in both setups.
try:
    from PySide6 import QtCore, QtWidgets
    _PYSIDE6 = True
except ImportError:
    from PySide2 import QtCore, QtWidgets
    _PYSIDE6 = False

from ..gui_utils import tr
from ..wb_utils import is_robot
from ..kinematics.models import Robot, Joint, Link
from ..kinematics.kinematics import forward_kinematics
from ..kinematics.inverse_kinematics import inverse_kinematics


# ─────────────────────────────────────────────────────────────────────────────
# Adapter helpers
# ─────────────────────────────────────────────────────────────────────────────

def _placement_to_xyz_rpy(
    placement: fc.Placement,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """
    Convert a FreeCAD Placement to (xyz_mm, rpy_rad) tuples.

    Parameters
    ----------
    placement : fc.Placement
        FreeCAD placement containing position and rotation.

    Returns
    -------
    tuple[tuple[float, float, float], tuple[float, float, float]]
        Position (x, y, z) in mm and orientation (roll, pitch, yaw) in radians.
    """
    position_vector = placement.Base
    xyz = (position_vector.x, position_vector.y, position_vector.z)
    yaw, pitch, roll = placement.Rotation.toEuler()
    rpy = (roll, pitch, yaw)
    return xyz, rpy


def _get_axis_from_placement(placement: fc.Placement) -> tuple[float, float, float]:
    """
    Return the joint axis direction by rotating the local Z vector by the joint frame.

    Parameters
    ----------
    placement : fc.Placement
        FreeCAD placement of the joint origin.

    Returns
    -------
    tuple[float, float, float]
        Joint axis (x, y, z) in the parent frame.
    """
    axis_vector = placement.Rotation.multVec(fc.Vector(0, 0, 1))
    return (axis_vector.x, axis_vector.y, axis_vector.z)


def _build_robot(
    robot_obj: fc.App.DocumentObject,
) -> tuple[Robot, list[fc.App.DocumentObject]]:
    """
    Build a RoboTool Robot instance from a RobotCAD Cross::Robot document object.

    Parameters
    ----------
    robot_obj : fc.App.DocumentObject
        The selected Cross::Robot object from the FreeCAD document tree.

    Returns
    -------
    tuple[Robot, list[fc.App.DocumentObject]]
        The populated RoboTool Robot model and the list of active (non-fixed)
        Cross::Joint document objects in kinematic chain order.
    """
    robot_instance = Robot(name=robot_obj.Label)

    for link_obj in robot_obj.Proxy.get_links():
        robot_instance.links.append(Link(name=link_obj.Label))

    active_joint_objects = []
    for joint_obj in robot_obj.Proxy.get_joints():
        xyz, rpy = _placement_to_xyz_rpy(joint_obj.Origin)
        axis = _get_axis_from_placement(joint_obj.Origin)

        robot_instance.joints.append(Joint(
            name           = joint_obj.Label,
            joint_type     = joint_obj.Type,
            parent         = joint_obj.Parent,
            child          = joint_obj.Child,
            origin_xyz     = xyz,
            origin_rpy     = rpy,
            axis           = axis,
            limit_lower    = getattr(joint_obj, 'LowerLimit', None),
            limit_upper    = getattr(joint_obj, 'UpperLimit', None),
            velocity_limit = getattr(joint_obj, 'Velocity',   None),
            effort_limit   = getattr(joint_obj, 'Effort',     None),
        ))
        if joint_obj.Type != 'fixed':
            active_joint_objects.append(joint_obj)

    return robot_instance, active_joint_objects


def _apply_joint_positions(
    joint_objects: list[fc.App.DocumentObject],
    joint_values: np.ndarray,
    doc: fc.App.Document,
    transaction_label: str = 'IK Tool',
) -> None:
    """
    Write joint values to the document and trigger a live 3D viewport update.

    Parameters
    ----------
    joint_objects : list[fc.App.DocumentObject]
        Non-fixed Cross::Joint objects to update.
    joint_values : np.ndarray
        Joint positions to write (radians for revolute/continuous, mm for prismatic).
    doc : fc.App.Document
        The active FreeCAD document.
    transaction_label : str, optional
        Label shown in the undo history (default: 'IK Tool').
    """
    doc.openTransaction(tr(transaction_label))
    for joint_obj, value in zip(joint_objects, joint_values):
        joint_obj.Position = float(value)
    doc.commitTransaction()
    doc.recompute()
    fcgui.updateGui()


# ─────────────────────────────────────────────────────────────────────────────
# Qt Dialog
# ─────────────────────────────────────────────────────────────────────────────

class IKToolDialog(QtWidgets.QDialog):
    """Modal dialog for the IK Tool command.

    ┌─ IK Tool ──────────────────────────────────────────────┐
    │  Target position (mm)                                  │
    │  X: [______]  Y: [______]  Z: [______]                 │
    │                                                        │
    │               [  ⚙  Solve IK  ]                        │
    │                                                        │
    │  Results ──────────────────────────────────────────────│
    │  ┌─────────────┬──────────────┬──────────────┐         │
    │  │ Joint       │ Original     │ Solved       │         │
    │  ├─────────────┼──────────────┼──────────────┤         │
    │  │ joint_1     │  0.0000 rad  │  1.2345 rad  │         │
    │  └─────────────┴──────────────┴──────────────┘         │
    │                                                        │
    │  Status: Converged. Error: 0.000000 mm                 │
    │                                                        │
    │  [ ▶ Apply to Model ]  [ ↩ Restore Original ]  [Close] │
    └────────────────────────────────────────────────────────┘
    """

    def __init__(
        self,
        robot_obj: fc.App.DocumentObject,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle(tr('IK Tool'))
        self.setMinimumWidth(500)

        # Remove the "?" help button; enum path differs between PySide2 and PySide6.
        _no_help = getattr(QtCore.Qt, 'WindowContextHelpButtonHint', None) \
                or getattr(QtCore.Qt.WindowType, 'WindowContextHelpButtonHint', 0)
        self.setWindowFlags(self.windowFlags() & ~_no_help)

        self._doc        = fc.activeDocument()
        self._robot_obj  = robot_obj
        self._robot      = None
        self._joint_objs = []
        self._original_joint_positions = None  # Captured on first Solve; never overwritten.
        self._solved_joint_positions   = None

        self._build_ui()
        self._load_robot()

    def _build_ui(self) -> None:
        root_layout = QtWidgets.QVBoxLayout(self)
        root_layout.setSpacing(10)
        root_layout.setContentsMargins(14, 14, 14, 14)

        # Target position input group
        grp_target = QtWidgets.QGroupBox(tr('Target position (mm)'))
        lay_target = QtWidgets.QHBoxLayout(grp_target)
        self._spins = {}
        for axis in ('X', 'Y', 'Z'):
            lay_target.addWidget(QtWidgets.QLabel(f'<b>{axis}</b>'))
            spin_box = QtWidgets.QDoubleSpinBox()
            spin_box.setRange(-10000.0, 10000.0)
            spin_box.setDecimals(3)
            spin_box.setSingleStep(1.0)
            spin_box.setFixedWidth(100)
            self._spins[axis] = spin_box
            lay_target.addWidget(spin_box)
            if axis != 'Z':
                lay_target.addSpacing(8)
        root_layout.addWidget(grp_target)

        # Solve button
        self._btn_solve = QtWidgets.QPushButton(tr('⚙  Solve IK'))
        self._btn_solve.setFixedHeight(34)
        self._btn_solve.clicked.connect(self._on_solve)
        root_layout.addWidget(self._btn_solve)

        # Results table
        grp_results = QtWidgets.QGroupBox(tr('Results'))
        lay_results = QtWidgets.QVBoxLayout(grp_results)
        self._table = QtWidgets.QTableWidget(0, 3)
        self._table.setHorizontalHeaderLabels(
            [tr('Joint'), tr('Original (rad/mm)'), tr('Solved (rad/mm)')]
        )
        self._table.horizontalHeader().setStretchLastSection(True)

        if _PYSIDE6:
            _no_edit = QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers
            _no_sel  = QtWidgets.QAbstractItemView.SelectionMode.NoSelection
        else:
            _no_edit = QtWidgets.QAbstractItemView.NoEditTriggers
            _no_sel  = QtWidgets.QAbstractItemView.NoSelection

        self._table.setEditTriggers(_no_edit)
        self._table.setSelectionMode(_no_sel)
        self._table.setAlternatingRowColors(True)
        self._table.setMinimumHeight(130)
        lay_results.addWidget(self._table)
        root_layout.addWidget(grp_results)

        # Status label
        self._lbl_status = QtWidgets.QLabel(
            tr('Ready. Enter a target and press Solve IK.')
        )
        self._lbl_status.setWordWrap(True)
        root_layout.addWidget(self._lbl_status)

        # Action buttons
        lay_btns = QtWidgets.QHBoxLayout()
        self._btn_apply = QtWidgets.QPushButton(tr('▶  Apply to Model'))
        self._btn_apply.setEnabled(False)
        self._btn_apply.clicked.connect(self._on_apply)

        self._btn_restore = QtWidgets.QPushButton(tr('↩  Restore Original'))
        self._btn_restore.setEnabled(False)
        self._btn_restore.clicked.connect(self._on_restore)

        btn_close = QtWidgets.QPushButton(tr('Close'))
        btn_close.clicked.connect(self.reject)

        lay_btns.addWidget(self._btn_apply)
        lay_btns.addWidget(self._btn_restore)
        lay_btns.addStretch()
        lay_btns.addWidget(btn_close)
        root_layout.addLayout(lay_btns)

    # ── Robot loading + FK pre-fill ──────────────────────────

    def _load_robot(self) -> None:
        """Build the RoboTool model from the selected robot and pre-fill the FK position."""
        try:
            self._robot, self._joint_objs = _build_robot(self._robot_obj)
        except Exception as exc:
            self._lbl_status.setText(f'Error loading robot: {exc}')
            self._btn_solve.setEnabled(False)
            return

        n_joints = len(self._joint_objs)
        if n_joints == 0:
            self._lbl_status.setText(tr('No active joints found in the selected robot.'))
            self._btn_solve.setEnabled(False)
            return

        q0 = np.array([float(j.Position) for j in self._joint_objs])

        # Pre-fill target spinboxes with the current FK end-effector position.
        try:
            fk_matrix = forward_kinematics(self._robot, q0)
            fk_position = fk_matrix[:3, 3]
            for axis, value in zip(('X', 'Y', 'Z'), fk_position):
                self._spins[axis].setValue(float(value))
        except Exception:
            pass

        # Populate the results table with current joint values.
        self._table.setRowCount(n_joints)
        for i, joint_obj in enumerate(self._joint_objs):
            self._table.setItem(i, 0, QtWidgets.QTableWidgetItem(joint_obj.Label))
            self._table.setItem(i, 1, QtWidgets.QTableWidgetItem(f'{q0[i]:.4f}'))
            self._table.setItem(i, 2, QtWidgets.QTableWidgetItem('—'))
        self._table.resizeColumnsToContents()

        self._lbl_status.setText(
            tr(f'Robot loaded: {n_joints} active joint(s). Adjust target and press Solve IK.')
        )

    def _on_solve(self) -> None:
        """Run the IK solver and display the results in the table."""
        if not self._joint_objs:
            return

        try:
            target = np.array([self._spins[axis].value() for axis in ('X', 'Y', 'Z')])
            q0     = np.array([float(j.Position) for j in self._joint_objs])

            # Capture original pose on the first solve; do not overwrite on re-solve.
            if self._original_joint_positions is None:
                self._original_joint_positions = q0.copy()
                self._btn_restore.setEnabled(True)

            # Capture solver print output so it can be shown in the status label.
            log_buffer = io.StringIO()
            with redirect_stdout(log_buffer):
                solved_joint_positions = inverse_kinematics(
                    self._robot,
                    target_position=target,
                    initial_guess=q0,
                )
            solver_log = log_buffer.getvalue().strip()

            if solved_joint_positions is None:
                for i in range(len(self._joint_objs)):
                    self._table.setItem(i, 2, QtWidgets.QTableWidgetItem(tr('Unreachable')))
                self._lbl_status.setText(
                    tr('IK did not converge. Target may be outside the workspace.')
                    + (f'\n{solver_log}' if solver_log else '')
                )
                self._btn_apply.setEnabled(False)
                return

            self._solved_joint_positions = solved_joint_positions

            # Update table with original and solved joint values.
            for i in range(len(self._joint_objs)):
                self._table.setItem(i, 1, QtWidgets.QTableWidgetItem(f'{self._original_joint_positions[i]:.4f}'))
                self._table.setItem(i, 2, QtWidgets.QTableWidgetItem(f'{self._solved_joint_positions[i]:.4f}'))
            self._table.resizeColumnsToContents()

            # Verify the solution with FK and compute the residual error.
            try:
                fk_matrix       = forward_kinematics(self._robot, self._solved_joint_positions)
                achieved        = fk_matrix[:3, 3]
                residual_error  = float(np.linalg.norm(target - achieved))
                status_details  = (
                    f'Target:   [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}] mm\n'
                    f'Achieved: [{achieved[0]:.3f}, {achieved[1]:.3f}, {achieved[2]:.3f}] mm\n'
                    f'Error:    {residual_error:.6f} mm'
                )
            except Exception as exc:
                status_details = f'(FK verification failed: {exc})'

            self._lbl_status.setText(
                (f'{solver_log}\n{status_details}') if solver_log else status_details
            )
            self._btn_apply.setEnabled(True)

        except Exception as exc:
            self._lbl_status.setText(f'Solver error: {exc}')
            self._btn_apply.setEnabled(False)

    # ── Slot: Apply solution ──────────────────────────────────

    def _on_apply(self) -> None:
        """Write the solved joint positions to the document and animate the model."""
        if self._solved_joint_positions is None:
            return
        _apply_joint_positions(
            self._joint_objs, self._solved_joint_positions, self._doc,
            transaction_label='IK Tool – apply',
        )
        self._lbl_status.setText(
            self._lbl_status.text() + tr('\n✔ Solution applied to model.')
        )

    # ── Slot: Restore original pose ───────────────────────────

    def _on_restore(self) -> None:
        """Write the original joint positions back to the document."""
        if self._original_joint_positions is None:
            return
        _apply_joint_positions(
            self._joint_objs, self._original_joint_positions, self._doc,
            transaction_label='IK Tool – restore',
        )
        self._lbl_status.setText(tr('↩ Original joint positions restored.'))


# ─────────────────────────────────────────────────────────────────────────────
# FreeCAD Gui Command
# ─────────────────────────────────────────────────────────────────────────────

class _IKToolCommand:
    """FreeCAD Gui command that opens the IK Tool dialog.
    Registers as ``IKTool``.  Only active when a Cross::Robot is selected.
    """

    def GetResources(self) -> dict:
        return {
            'Pixmap'  : 'ik_solver.svg',
            'MenuText': tr('IK Tool'),
            'Accel'   : 'I, K',
            'ToolTip' : tr(
                'Forward & Inverse Kinematics Tool (RoboTool)\n'
                '\n'
                'Select a robot, then open this tool to:\n'
                '  • Inspect the current end-effector position (FK).\n'
                '  • Enter a Cartesian target (X, Y, Z in millimetres).\n'
                '  • Solve IK via iterative Jacobian pseudo-inverse.\n'
                '  • See solved joint angles in a results table.\n'
                '  • Apply the solution → live 3-D animation.\n'
                '  • Restore the original pose at any time.\n'
                '\n'
                'Requires: a Cross::Robot selected in the scene.\n'
                'Algorithm: Jacobian pseudo-inverse, step-limited,\n'
                '           up to 1 000 iterations, tol = 1 * 10⁻⁴ mm.\n'
            ),
        }

    def IsActive(self) -> bool:
        """Return True only when a Cross::Robot object is selected."""
        selection = fcgui.Selection.getSelection()
        return bool(selection) and is_robot(selection[0])

    def Activated(self) -> None:
        robot_obj = fcgui.Selection.getSelection()[0]

        # parent=None avoids the PySide2/6 type error with QMainWindow.
        # WA_DeleteOnClose=False + self._dialog reference prevent the SEGFAULT
        # caused by Python GC destroying the C++ object before exec_() returns.
        dialog = IKToolDialog(robot_obj, parent=None)
        if _PYSIDE6:
            dialog.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose, False)
        else:
            dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose, False)
        self._dialog = dialog  # keep Python refcount > 0 during exec_()
        try:
            dialog.exec() if _PYSIDE6 else dialog.exec_()
        finally:
            self._dialog = None
            try:
                dialog.deleteLater()
            except RuntimeError:
                pass  # C++ object already gone — safe to ignore


fcgui.addCommand('IKTool', _IKToolCommand())