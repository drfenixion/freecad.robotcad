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

def _placement_to_xyz_rpy(placement: fc.Placement) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """
    Convert a FreeCAD Placement object to internal (xyz_mm, rpy_rad) parameter tuples.

    Parameters:
    -----------
    placement : fc.Placement
        The native FreeCAD object containing transformation matrices.
    Returns:
    --------
    tuple[tuple[float, float, float], tuple[float, float, float]]
        A tuple containing positional (x, y, z) in mm and orientation (roll, pitch, yaw) in radians.
    """
    position_vector = placement.Base
    coordinates_xyz = (position_vector.x, position_vector.y, position_vector.z)
    yaw, pitch, roll = placement.Rotation.toEuler()
    orientations_rpy = (math.radians(roll), math.radians(pitch), math.radians(yaw))
    return coordinates_xyz, orientations_rpy


def _get_axis_from_placement(placement: fc.Placement) -> tuple[float, float, float]:
    """
    Extract the joint transformation axis (local Z rotated by the joint orientation frame).

    Parameters:
    -----------
    placement : fc.Placement
        The FreeCAD spatial placement coordinate reference.

    Returns:
    --------
    tuple[float, float, float]
        The relative 3D orientation components (x, y, z) representing the joint axis vector.
    """
    axis_vector = placement.Rotation.multVec(fc.Vector(0, 0, 1))
    return (axis_vector.x, axis_vector.y, axis_vector.z)


def _build_robot(robot_obj: fc.App.DocumentObject) -> tuple[Robot, list[fc.App.DocumentObject]]:
    """
    Build a structural RoboTool Robot instance from a native RobotCAD Cross::Robot object.

    Parameters:
    -----------
    robot_obj : fc.App.DocumentObject
        The current active RobotCAD robot object selected in the FreeCAD tree view.
    Returns:
    --------
    tuple[Robot, list[fc.App.DocumentObject]]
        A tuple containing the mapped mathematical RoboTool Robot instance, and a sequential 
        list of active non-fixed Cross::Joint document objects.
    """
    robot_instance = Robot(name=robot_obj.Label)

    for link_obj in robot_obj.Proxy.get_links():
        robot_instance.links.append(Link(name=link_obj.Label))

    active_joint_objects = []
    for joint_obj in robot_obj.Proxy.get_joints():
        coordinates_xyz, orientations_rpy = _placement_to_xyz_rpy(joint_obj.Origin)
        joint_axis_vector = _get_axis_from_placement(joint_obj.Origin)
        
        robot_instance.joints.append(Joint(
            name           = joint_obj.Label,
            joint_type     = joint_obj.Type,
            parent         = joint_obj.Parent,
            child          = joint_obj.Child,
            origin_xyz     = coordinates_xyz,
            origin_rpy     = orientations_rpy,
            axis           = joint_axis_vector,
            limit_lower    = getattr(joint_obj, 'LowerLimit', None),
            limit_upper    = getattr(joint_obj, 'UpperLimit', None),
            velocity_limit = getattr(joint_obj, 'Velocity',   None),
            effort_limit   = getattr(joint_obj, 'Effort',     None),
        ))
        if joint_obj.Type != 'fixed':
            active_joint_objects.append(joint_obj)

    return robot_instance, active_joint_objects

def _apply_joint_positions(joint_objects: list[fc.App.DocumentObject], configuration_vector: np.ndarray, active_document: fc.App.Document, transaction_label: str = 'IK Tool') -> None:
    """
    Write calculated joint positions back to the active document and trigger a live 3D recompute.

    Parameters:
    -----------
    joint_objects : list[fc.App.DocumentObject]
        List of non-fixed FreeCAD joint objects to update.
    configuration_vector : np.ndarray
        Vector of calculated joint parameter values (radians/mm).
    active_document : fc.App.Document
        The current open FreeCAD document session target.
    transaction_label : str, optional
        Undo history marker label (default is 'IK Tool').
    """
    active_document.openTransaction(tr(transaction_label))
    for internal_object, configuration_value in zip(joint_objects, configuration_vector):
        internal_object.Position = float(configuration_value)
    active_document.commitTransaction()
    active_document.recompute()
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

    def __init__(self, robot_obj: fc.App.DocumentObject, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle(tr('IK Tool'))
        self.setMinimumWidth(500)

        # Remove "?" help button — enum path path differs between PySide2 and PySide6
        _no_help = getattr(QtCore.Qt, 'WindowContextHelpButtonHint', None) \
                or getattr(QtCore.Qt.WindowType, 'WindowContextHelpButtonHint', 0)
        self.setWindowFlags(self.windowFlags() & ~_no_help)

        # Internal state configurations ───────────────────────
        self._doc        = fc.activeDocument()
        self._robot_obj  = robot_obj
        self._robot      = None
        self._joint_objs = []
        self._q_original = None   # captured on first execution tracking
        self._q_solved   = None

        # Build UI then load robot configurations
        self._build_ui()
        self._load_robot()

    def _build_ui(self) -> None:
        root_layout = QtWidgets.QVBoxLayout(self)
        root_layout.setSpacing(10)
        root_layout.setContentsMargins(14, 14, 14, 14)

        # Target position spatial parameters group
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

        # Action execution solve button
        self._btn_solve = QtWidgets.QPushButton(tr('⚙  Solve IK'))
        self._btn_solve.setFixedHeight(34)
        self._btn_solve.clicked.connect(self._on_solve)
        root_layout.addWidget(self._btn_solve)

        # Results visualization matrix grid table
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

        # Convergence logging tracking label status
        self._lbl_status = QtWidgets.QLabel(
            tr('Ready. Enter a target and press Solve IK.')
        )
        self._lbl_status.setWordWrap(True)
        root_layout.addWidget(self._lbl_status)

        # Operations action panel layout execution buttons
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
        """Map the backend mathematical components model and pre-fill targets."""
        try:
            self._robot, self._joint_objs = _build_robot(self._robot_obj)
        except Exception as exc:
            self._lbl_status.setText(f'Error loading robot configuration models: {exc}')
            self._btn_solve.setEnabled(False)
            return

        total_joints = len(self._joint_objs)
        if total_joints == 0:
            self._lbl_status.setText(tr('No active mechanical joints found in target robot.'))
            self._btn_solve.setEnabled(False)
            return

        initial_configuration_values = np.array([float(joint.Position) for joint in self._joint_objs])

        # Core Forward Kinematics extraction to initialize spin box coordinates
        try:
            transformation_matrix = forward_kinematics(self._robot, initial_configuration_values)
            cartesian_position = transformation_matrix[:3, 3]
            for axis_key, dimension_value in zip(('X', 'Y', 'Z'), cartesian_position):
                self._spins[axis_key].setValue(float(dimension_value))
        except Exception:
            pass

        # Populate internal table view parameters rows
        self._table.setRowCount(total_joints)
        for i, obj in enumerate(self._joint_objs):
            self._table.setItem(i, 0, QtWidgets.QTableWidgetItem(obj.Label))
            self._table.setItem(i, 1, QtWidgets.QTableWidgetItem(f'{initial_configuration_values[i]:.4f}'))
            self._table.setItem(i, 2, QtWidgets.QTableWidgetItem('—'))
        self._table.resizeColumnsToContents()

        self._lbl_status.setText(
            tr(f'Robot loaded: {total_joints} active joint(s). Adjust target and press Solve IK.')
        )

    def _on_solve(self) -> None:
        """Execute core solver optimizations and render output computations."""
        if not self._joint_objs:
            return

        try:
            target_cartesian_position = np.array([self._spins[axis].value() for axis in ('X', 'Y', 'Z')])
            initial_configuration_values = np.array([float(joint.Position) for joint in self._joint_objs])

            # Store immutable backup state of original configurations
            if self._q_original is None:
                self._q_original = initial_configuration_values.copy()
                self._btn_restore.setEnabled(True)

            # Redirect iterative print tracking streams 
            output_stream_buffer = io.StringIO()
            with redirect_stdout(output_stream_buffer):
                calculated_solution = inverse_kinematics(
                    self._robot,
                    target_position=target_cartesian_position,
                    initial_guess=initial_configuration_values,
                )
            solver_logs = output_stream_buffer.getvalue().strip()

            if calculated_solution is None:
                for i in range(len(self._joint_objs)):
                    self._table.setItem(i, 2, QtWidgets.QTableWidgetItem(tr('Unreachable')))
                self._lbl_status.setText(
                    tr('IK did not converge. Target location may be located outside workspace boundaries.')
                    + (f'\n{solver_logs}' if solver_logs else '')
                )
                self._btn_apply.setEnabled(False)
                return

            self._q_solved = calculated_solution

            # Sync table values matrices states
            for i in range(len(self._joint_objs)):
                self._table.setItem(i, 1, QtWidgets.QTableWidgetItem(f'{self._q_original[i]:.4f}'))
                self._table.setItem(i, 2, QtWidgets.QTableWidgetItem(f'{calculated_solution[i]:.4f}'))
            self._table.resizeColumnsToContents()

            # Residual calculation verification loops
            try:
                result_transformation = forward_kinematics(self._robot, calculated_solution)
                achieved_cartesian = result_transformation[:3, 3]
                residual_error_norm = float(np.linalg.norm(target_cartesian_position - achieved_cartesian))
                details_report = (
                    f'Target:   [{target_cartesian_position[0]:.3f}, {target_cartesian_position[1]:.3f}, {target_cartesian_position[2]:.3f}] mm\n'
                    f'Achieved: [{achieved_cartesian[0]:.3f}, {achieved_cartesian[1]:.3f}, {achieved_cartesian[2]:.3f}] mm\n'
                    f'Error:    {residual_error_norm:.6f} mm'
                )
            except Exception as exc:
                details_report = f'(FK verification loop crashed: {exc})'

            self._lbl_status.setText(
                (f'{solver_logs}\n{details_report}') if solver_logs else details_report
            )
            self._btn_apply.setEnabled(True)

        except Exception as exc:
            self._lbl_status.setText(f'Unexpected runtime execution solver crash: {exc}')
            self._btn_apply.setEnabled(False)

    # ── Slot: Apply solution ──────────────────────────────────

    def _on_apply(self) -> None:
        """Apply tracking computed solutions directly to the visual active document model trees."""
        if self._q_solved is None:
            return
        _apply_joint_positions(self._joint_objs, self._q_solved, self._doc, transaction_label='IK Tool – apply')
        self._lbl_status.setText(
            self._lbl_status.text() + tr('\n✔ Solution applied to model.')
        )

    # ── Slot: Restore original pose ───────────────────────────

    def _on_restore(self) -> None:
        """Revert kinematics matrix structures to original historical presets."""
        if self._q_original is None:
            return
        _apply_joint_positions(self._joint_objs, self._q_original, self._doc, transaction_label='IK Tool – restore')
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
        """Active evaluation validation logic checks checking if selection is a robot node."""
        selection_nodes = fcgui.Selection.getSelection()
        return bool(selection_nodes) and is_robot(selection_nodes[0])

    def Activated(self) -> None:
        selected_robot_node = fcgui.Selection.getSelection()[0]

        # parent=None avoids the PySide2/6 type error with QMainWindow.
        # WA_DeleteOnClose=False + self._dialog reference prevent the SEGFAULT
        # caused by Python GC destroying the C++ object before exec_() returns.
        dialog_window = IKToolDialog(selected_robot_node, parent=None)
        # WA_DeleteOnClose enum path differs PySide2 vs PySide6
        if _PYSIDE6:
            dialog_window.setAttribute(QtCore.Qt.WidgetAttribute.WA_DeleteOnClose, False)
        else:
            dialog_window.setAttribute(QtCore.Qt.WA_DeleteOnClose, False)
        self._dialog = dialog_window       # keep Python refcount > 0 during exec_()
        try:
            dialog_window.exec() if _PYSIDE6 else dialog_window.exec_()
        finally:
            self._dialog = None
            try:
                dialog_window.deleteLater()
            except RuntimeError:
                pass                # C++ object already gone — safe to ignore

fcgui.addCommand('IKTool', _IKToolCommand())