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
├── [Apply to Model]   - writes q_solved → joint_obj.Position, recomputes
│                        document → live 3-D animation
├── [Restore Original] - writes q_original back → restores initial pose
└── [Close]

State management
~~~~~~~~~~~~~~~~
_q_original is captured once (on the first Solve click) so the user can
always return to the pose that existed before the dialog was opened.
Subsequent Solve calls do NOT overwrite _q_original.

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

def _placement_to_xyz_rpy(placement):
    """Convert a FreeCAD Placement to (xyz_mm, rpy_rad) tuples.
    FreeCAD stores coordinates internally in millimetres, so no conversion
    is needed for the xyz part.
    """
    pos = placement.Base
    xyz = (pos.x, pos.y, pos.z)          # already in mm
    yaw, pitch, roll = placement.Rotation.toEuler()
    rpy = (math.radians(roll), math.radians(pitch), math.radians(yaw))
    return xyz, rpy


def _get_axis_from_placement(placement):
    """Return the joint axis (local Z rotated by the joint frame)."""
    axis = placement.Rotation.multVec(fc.Vector(0, 0, 1))
    return (axis.x, axis.y, axis.z)


def _build_robot(robot_obj) -> tuple[Robot, list]:
    """Build a RoboTool Robot from a RobotCAD Cross::Robot object.

    Returns
    -------
    robot      : RoboTool Robot instance
    joint_objs : list of non-fixed Cross::Joint objects in kinematic order
    """
    robot = Robot(name=robot_obj.Label)

    for link_obj in robot_obj.Proxy.get_links():
        robot.links.append(Link(name=link_obj.Label))

    joint_objs = []
    for joint_obj in robot_obj.Proxy.get_joints():
        xyz, rpy = _placement_to_xyz_rpy(joint_obj.Origin)
        axis     = _get_axis_from_placement(joint_obj.Origin)
        robot.joints.append(Joint(
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
            joint_objs.append(joint_obj)

    return robot, joint_objs


def _apply_q(joint_objs: list, q, doc, label: str = 'IK Tool') -> None:
    """Write joint positions to the document and trigger a live recompute.

    Wrapped in an undo transaction so the user can Ctrl-Z after closing
    the dialog.
    """
    doc.openTransaction(tr(label))
    for obj, val in zip(joint_objs, q):
        obj.Position = float(val)
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

    def __init__(self, robot_obj, parent=None):
        super().__init__(parent)
        self.setWindowTitle(tr('IK Tool'))
        self.setMinimumWidth(500)

        # Remove "?" help button — enum path differs between PySide2 and PySide6
        _no_help = getattr(QtCore.Qt, 'WindowContextHelpButtonHint', None) \
                or getattr(QtCore.Qt.WindowType, 'WindowContextHelpButtonHint', 0)
        self.setWindowFlags(self.windowFlags() & ~_no_help)

        # Internal state  ────────────────────────────────────
        self._doc        = fc.activeDocument()
        self._robot_obj  = robot_obj
        self._robot      = None
        self._joint_objs = []
        self._q_original = None   # saved on first Solve; never overwritten
        self._q_solved   = None

        # Build UI then load robot (order matters: UI widgets must exist first)
        self._build_ui()
        self._load_robot()

    # ── UI construction ──────────────────────────────────────

    def _build_ui(self):
        root = QtWidgets.QVBoxLayout(self)
        root.setSpacing(10)
        root.setContentsMargins(14, 14, 14, 14)

        # Target position group
        grp_target = QtWidgets.QGroupBox(tr('Target position (mm)'))
        lay_target = QtWidgets.QHBoxLayout(grp_target)
        self._spins = {}
        for axis in ('X', 'Y', 'Z'):
            lay_target.addWidget(QtWidgets.QLabel(f'<b>{axis}</b>'))
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-10000.0, 10000.0)
            sp.setDecimals(3)
            sp.setSingleStep(1.0)
            sp.setFixedWidth(100)
            self._spins[axis] = sp
            lay_target.addWidget(sp)
            if axis != 'Z':
                lay_target.addSpacing(8)
        root.addWidget(grp_target)

        # Solve button
        self._btn_solve = QtWidgets.QPushButton(tr('⚙  Solve IK'))
        self._btn_solve.setFixedHeight(34)
        self._btn_solve.clicked.connect(self._on_solve)
        root.addWidget(self._btn_solve)

        # Results table
        grp_results = QtWidgets.QGroupBox(tr('Results'))
        lay_results = QtWidgets.QVBoxLayout(grp_results)
        self._table = QtWidgets.QTableWidget(0, 3)
        self._table.setHorizontalHeaderLabels(
            [tr('Joint'), tr('Original (rad/mm)'), tr('Solved (rad/mm)')]
        )
        self._table.horizontalHeader().setStretchLastSection(True)
        # Enum path differs between PySide2 and PySide6
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
        root.addWidget(grp_results)

        # Status label
        self._lbl_status = QtWidgets.QLabel(
            tr('Ready. Enter a target and press Solve IK.')
        )
        self._lbl_status.setWordWrap(True)
        root.addWidget(self._lbl_status)

        # Action buttons row
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
        root.addLayout(lay_btns)

    # ── Robot loading + FK pre-fill ──────────────────────────

    def _load_robot(self):
        """Build the RoboTool model and pre-fill the target with current FK pos."""
        try:
            self._robot, self._joint_objs = _build_robot(self._robot_obj)
        except Exception as exc:
            self._lbl_status.setText(f'Error loading robot: {exc}')
            self._btn_solve.setEnabled(False)
            return

        n = len(self._joint_objs)
        if n == 0:
            self._lbl_status.setText(tr('No active joints found in robot.'))
            self._btn_solve.setEnabled(False)
            return

        q0 = np.array([float(j.Position) for j in self._joint_objs])

        # Pre-fill target spinboxes with current FK end-effector position
        try:
            T   = forward_kinematics(self._robot, q0)
            pos = T[:3, 3]
            for ax, val in zip(('X', 'Y', 'Z'), pos):
                self._spins[ax].setValue(float(val))
        except Exception:
            pass

        # Populate table (Solved column shows "—" until first Solve)
        self._table.setRowCount(n)
        for i, obj in enumerate(self._joint_objs):
            self._table.setItem(i, 0, QtWidgets.QTableWidgetItem(obj.Label))
            self._table.setItem(i, 1, QtWidgets.QTableWidgetItem(f'{q0[i]:.4f}'))
            self._table.setItem(i, 2, QtWidgets.QTableWidgetItem('—'))
        self._table.resizeColumnsToContents()

        self._lbl_status.setText(
            tr(f'Robot loaded: {n} active joint(s). Adjust target and press Solve IK.')
        )

    # ── Slot: Solve IK ────────────────────────────────────────

    def _on_solve(self):
        """Run the IK solver and populate the results table."""
        if not self._joint_objs:
            return

        try:
            target_pos = np.array([self._spins[a].value() for a in ('X', 'Y', 'Z')])
            q0 = np.array([float(j.Position) for j in self._joint_objs])

            # Save original state only on the first Solve call
            if self._q_original is None:
                self._q_original = q0.copy()
                self._btn_restore.setEnabled(True)

            # Run solver, capturing any printed convergence logs
            buf = io.StringIO()
            with redirect_stdout(buf):
                q_solved = inverse_kinematics(
                    self._robot,
                    target_pos=target_pos,
                    initial_guess=q0,
                )
            log = buf.getvalue().strip()

            # Guard: solver returns None when it cannot converge
            if q_solved is None:
                for i in range(len(self._joint_objs)):
                    self._table.setItem(
                        i, 2, QtWidgets.QTableWidgetItem(tr('Unreachable'))
                    )
                self._lbl_status.setText(
                    tr('IK did not converge. Target may be outside workspace.')
                    + (f'\n{log}' if log else '')
                )
                self._btn_apply.setEnabled(False)
                return

            self._q_solved = q_solved

            # Update table with original and solved values
            for i in range(len(self._joint_objs)):
                self._table.setItem(
                    i, 1, QtWidgets.QTableWidgetItem(f'{self._q_original[i]:.4f}')
                )
                self._table.setItem(
                    i, 2, QtWidgets.QTableWidgetItem(f'{q_solved[i]:.4f}')
                )
            self._table.resizeColumnsToContents()

            # Compute achieved position and residual error
            try:
                T_res    = forward_kinematics(self._robot, q_solved)
                achieved = T_res[:3, 3]
                err      = float(np.linalg.norm(target_pos - achieved))
                detail   = (
                    f'Target:   [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] mm\n'
                    f'Achieved: [{achieved[0]:.3f}, {achieved[1]:.3f}, {achieved[2]:.3f}] mm\n'
                    f'Error:    {err:.4f} mm'
                )
            except Exception as exc:
                detail = f'(FK check failed: {exc})'

            self._lbl_status.setText(
                (f'{log}\n{detail}') if log else detail
            )
            self._btn_apply.setEnabled(True)

        except Exception as exc:
            self._lbl_status.setText(f'Unexpected solver error: {exc}')
            self._btn_apply.setEnabled(False)

    # ── Slot: Apply solution ──────────────────────────────────

    def _on_apply(self):
        """Apply the solved joint configuration to the FreeCAD model."""
        if self._q_solved is None:
            return
        _apply_q(self._joint_objs, self._q_solved, self._doc, label='IK Tool – apply')
        self._lbl_status.setText(
            self._lbl_status.text() + tr('\n✔ Solution applied to model.')
        )

    # ── Slot: Restore original pose ───────────────────────────

    def _on_restore(self):
        """Restore the joint configuration that existed before the dialog opened."""
        if self._q_original is None:
            return
        _apply_q(self._joint_objs, self._q_original, self._doc, label='IK Tool – restore')
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
                '           up to 1 000 iterations, tol = 1 × 10⁻⁶ mm.\n'
            ),
        }

    def IsActive(self) -> bool:
        """Active only when a Cross::Robot object is selected."""
        sel = fcgui.Selection.getSelection()
        return bool(sel) and is_robot(sel[0])

    def Activated(self) -> None:
        robot_obj = fcgui.Selection.getSelection()[0]

        # parent=None avoids the PySide2/6 type error with QMainWindow.
        # WA_DeleteOnClose=False + self._dialog reference prevent the SEGFAULT
        # caused by Python GC destroying the C++ object before exec_() returns.
        dialog = IKToolDialog(robot_obj, parent=None)
        # WA_DeleteOnClose enum path differs PySide2 vs PySide6
        if _PYSIDE6:
            dialog.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose, False)
        else:
            dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose, False)
        self._dialog = dialog       # keep Python refcount > 0 during exec_()
        try:
            dialog.exec() if _PYSIDE6 else dialog.exec_()
        finally:
            self._dialog = None
            try:
                dialog.deleteLater()
            except RuntimeError:
                pass                # C++ object already gone — safe to ignore

fcgui.addCommand('IKTool', _IKToolCommand())