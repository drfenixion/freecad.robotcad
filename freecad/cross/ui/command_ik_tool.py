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
Cross::Robot object selected, a non-modal dialog opens:

IKToolDialog
├── End Effector selector  - ComboBox listing all leaf links (tips) of the
│                            robot's kinematic tree.  Selecting one isolates
│                            the serial sub-chain from that link to the root,
│                            ignoring all other branches (wheels, legs, etc.).
├── Target position group  - three QDoubleSpinBox fields (X, Y, Z in mm)
│                            pre-filled with the current FK end-effector position
│                            expressed in WORLD (global) coordinates.
├── Target orientation group - three QDoubleSpinBox fields (Roll, Pitch, Yaw in deg)
│                              expressed in WORLD coordinates; pre-filled with the
│                              current end-effector orientation (FK result).
├── [Solve IK]    - runs RoboTool iterative Jacobian DLS solver using
│                   INCREMENTAL interpolation (N_STEPS intermediate targets)
│                   to guarantee smooth, continuous motion in the viewport.
│                   Each intermediate step is applied to the model in real time.
├── Results table - one row per joint in the sub-chain:
│     columns: Joint name | Original (rad/mm) | Solved (rad/mm)
├── Status label  - convergence message + achieved position + errors
│                   (position error in mm, orientation error in deg)
├── [Apply to Model]   - writes q_solved → joint_obj.Position,
│                        doc.recompute() for live 3-D animation
├── [Restore Original] - writes q_original back → restores initial pose
└── [Close]

Incremental IK (smooth motion)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Instead of solving IK once for the full target displacement, the solver
interpolates N_STEPS = 20 intermediate targets between the current EF
position and the desired target, applying each step to the model in real
time.  This guarantees:

- Continuous, smooth motion in the 3D viewport (no large jumps)
- Configuration continuity: each step uses the previous step's joint
    positions as the initial guess, keeping the robot on the same
    kinematic branch
- Graceful degradation: if any intermediate step fails to converge,
    the robot stops at the last valid configuration

Coordinate system convention
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
All spinbox values (target position + orientation) are expressed in the
WORLD (global) FreeCAD coordinate system.

RoboTool's FK/IK solver operates in the robot's LOCAL frame.
Two conversion steps are applied:

1.  Pre-fill  (FK local → world):
        T_world_ee = T_world_robot @ T_robot_ee
        spinbox ← T_world_ee[:3, 3]   (position)
        spinbox ← RPY(T_world_ee[:3,:3])  (orientation)

2.  Solve  (world target → local per step, IK, result in joint space):
        p_local      = R_world^T @ (p_world_target − p_world_robot_base)
        R_tgt_local  = R_world^T @ R_tgt_world
        q_solved ← IK(p_local_interp, R_tgt_local)

IK algorithm
~~~~~~~~~~~~
6-DOF Damped Least Squares (DLS) with SVD-based adaptive damping:

    full_error = [w_pos * Δp,  w_ori * Δω]      shape (6,)
    J          = compute_jacobian(robot, q)       shape (6, N)
    Δq         = Vᵀ diag(σ/(σ²+λ²)) Uᵀ · full_error

    λ selected based on condition number κ(J):
        κ > 100 → λ = 0.1  (near singularity)
        κ > 10  → λ = 0.01
        κ ≤ 10  → λ = 0.001 (well-conditioned)

Singularity detection: Yoshikawa index w = sqrt(det(J J^T)) < 1e-3
triggers automatic perturbation of the initial configuration.

Dialog layout
~~~~~~~~~~~~~
┌─ IK Tool ──────────────────────────────────────────────┐
│  End Effector: [tip_link ▼]                            │
│                                                        │
│  Target position (mm) [world frame]                    │
│  X: [______]  Y: [______]  Z: [______]                 │
│                                                        │
│  Target orientation (deg) [world frame]                │
│  R: [______]  P: [______]  Y: [______]                 │
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
│  Status: Converged. 20/20 steps.                       │
│  Target:   [x, y, z] mm  (world)                       │
│  Achieved: [x, y, z] mm  (world)                       │
│  Error:    0.000001 mm                                 │
│  Error orientation: 0.06 deg                           │
│                                                        │
│  [ ▶ Apply to Model ]  [ ↩ Restore Original ]  [Close] │
└────────────────────────────────────────────────────────┘

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

# Number of interpolation steps for smooth incremental motion.
# Higher = smoother but slower per Solve click.
_N_STEPS = 20


# ─────────────────────────────────────────────────────────────────────────────
# Kinematic-tree helpers
# ─────────────────────────────────────────────────────────────────────────────

def _build_parent_map(
    robot_obj: fc.App.DocumentObject,
) -> tuple[dict[str, str], dict[str, fc.App.DocumentObject]]:
    """Build child→parent link map and child→joint_obj lookup."""
    child_to_parent: dict[str, str] = {}
    child_to_joint:  dict[str, fc.App.DocumentObject] = {}
    for joint_obj in robot_obj.Proxy.get_joints():
        child_to_parent[joint_obj.Child] = joint_obj.Parent
        child_to_joint[joint_obj.Child]  = joint_obj
    return child_to_parent, child_to_joint


def _find_leaf_links(robot_obj: fc.App.DocumentObject) -> list[str]:
    """Return all leaf (end-effector candidate) link names in the robot tree."""
    all_links:    set[str] = {lnk.Label for lnk in robot_obj.Proxy.get_links()}
    parent_links: set[str] = {j.Parent for j in robot_obj.Proxy.get_joints()}
    leaf_links = sorted(all_links - parent_links)
    return leaf_links if leaf_links else sorted(all_links)


def _get_chain_to_root(
    robot_obj:         fc.App.DocumentObject,
    end_effector_link: str,
    child_to_parent:   dict[str, str],
    child_to_joint:    dict[str, fc.App.DocumentObject],
) -> list[fc.App.DocumentObject]:
    """Walk backwards from end_effector_link to root, returning joints root→EE."""
    if end_effector_link not in child_to_parent:
        return []
    chain_joints: list[fc.App.DocumentObject] = []
    current_link = end_effector_link
    while current_link in child_to_parent:
        joint_obj = child_to_joint[current_link]
        if joint_obj.Type != 'fixed':
            chain_joints.append(joint_obj)
        current_link = child_to_parent[current_link]
    chain_joints.reverse()
    return chain_joints


# ─────────────────────────────────────────────────────────────────────────────
# Adapter helpers
# ─────────────────────────────────────────────────────────────────────────────

def _placement_to_xyz_rpy(
    placement: fc.Placement,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Convert a FreeCAD Placement to (xyz_mm, rpy_rad) tuples."""
    p = placement.Base
    yaw, pitch, roll = placement.Rotation.toEuler()
    return (
        (p.x, p.y, p.z),
        (math.radians(roll), math.radians(pitch), math.radians(yaw)),
    )


def _get_axis_from_placement(placement: fc.Placement) -> tuple[float, float, float]:
    """Return joint axis by rotating local Z by the joint frame rotation."""
    v = placement.Rotation.multVec(fc.Vector(0, 0, 1))
    return (v.x, v.y, v.z)


def _build_robot_for_chain(
    robot_obj:    fc.App.DocumentObject,
    chain_joints: list[fc.App.DocumentObject],
) -> Robot:
    """Build a RoboTool Robot for the isolated serial sub-chain."""
    robot_instance = Robot(name=robot_obj.Label)
    if not chain_joints:
        return robot_instance
    robot_instance.links.append(Link(name=chain_joints[0].Parent))
    for joint_obj in chain_joints:
        xyz, rpy = _placement_to_xyz_rpy(joint_obj.Origin)
        axis     = _get_axis_from_placement(joint_obj.Origin)
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
        robot_instance.links.append(Link(name=joint_obj.Child))
    return robot_instance


def _get_robot_global_transform(robot_obj: fc.App.DocumentObject) -> np.ndarray:
    """Return the 4×4 homogeneous transform of the robot base in world coordinates."""
    placement = (
        robot_obj.getGlobalPlacement()
        if hasattr(robot_obj, 'getGlobalPlacement')
        else robot_obj.Placement
    )
    p = placement.Base
    m = placement.Rotation.toMatrix()
    T = np.eye(4)
    T[0, 0] = m.A11;  T[0, 1] = m.A12;  T[0, 2] = m.A13
    T[1, 0] = m.A21;  T[1, 1] = m.A22;  T[1, 2] = m.A23
    T[2, 0] = m.A31;  T[2, 1] = m.A32;  T[2, 2] = m.A33
    T[0, 3] = p.x;    T[1, 3] = p.y;    T[2, 3] = p.z
    return T


def _apply_joint_positions(
    robot_obj:         fc.App.DocumentObject,
    joint_objects:     list[fc.App.DocumentObject],
    joint_values:      np.ndarray,
    doc:               fc.App.Document,
    transaction_label: str = 'IK Tool',
) -> None:
    """Write joint values to the document and trigger a live 3D viewport update."""
    joint_variables: dict = robot_obj.Proxy.joint_variables
    doc.openTransaction(tr(transaction_label))
    for joint_obj, value in zip(joint_objects, joint_values):
        prop_name = joint_variables.get(joint_obj)
        if prop_name is None:
            continue
        native_value = (
            float(value) if joint_obj.Type == 'prismatic'
            else math.degrees(float(value))
        )
        setattr(robot_obj, prop_name, native_value)
    doc.commitTransaction()
    doc.recompute()
    fcgui.updateGui()


# ─────────────────────────────────────────────────────────────────────────────
# Qt Dialog
# ─────────────────────────────────────────────────────────────────────────────

class IKToolDialog(QtWidgets.QDialog):
    """Non-modal dialog for the IK Tool command — see module docstring for layout."""

    def __init__(
        self,
        robot_obj: fc.App.DocumentObject,
        parent:    QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle(tr('IK Tool'))
        self.setMinimumWidth(520)

        _no_help = (
            getattr(QtCore.Qt.WindowType, 'WindowContextHelpButtonHint', None)
            or getattr(QtCore.Qt, 'WindowContextHelpButtonHint', 0)
        )
        self.setWindowFlags(self.windowFlags() & ~_no_help)

        self._doc          = fc.activeDocument()
        self._robot_obj    = robot_obj
        self._robot        = None
        self._joint_objs   = []
        self.original_joint_positions = None
        self.solved_joint_positions   = None

        self._child_to_parent: dict[str, str] = {}
        self._child_to_joint:  dict[str, fc.App.DocumentObject] = {}

        self._build_ui()
        self._init_tree_maps()
        self._populate_ee_combo()

    # ── UI construction ───────────────────────────────────────

    def _build_ui(self) -> None:
        root_layout = QtWidgets.QVBoxLayout(self)
        root_layout.setSpacing(10)
        root_layout.setContentsMargins(14, 14, 14, 14)

        # End Effector selector
        grp_ee = QtWidgets.QGroupBox(tr('End Effector'))
        lay_ee = QtWidgets.QHBoxLayout(grp_ee)
        lay_ee.addWidget(QtWidgets.QLabel(tr('Select tip link:')))
        self._combo_ee = QtWidgets.QComboBox()
        self._combo_ee.setMinimumWidth(200)
        self._combo_ee.currentIndexChanged.connect(self._on_ee_changed)
        lay_ee.addWidget(self._combo_ee)
        lay_ee.addStretch()
        root_layout.addWidget(grp_ee)

        # Target position
        grp_target = QtWidgets.QGroupBox(tr('Target position (mm) — world frame'))
        lay_target = QtWidgets.QHBoxLayout(grp_target)
        self._spins: dict[str, QtWidgets.QDoubleSpinBox] = {}
        for axis in ('X', 'Y', 'Z'):
            lay_target.addWidget(QtWidgets.QLabel(f'<b>{axis}</b>'))
            spin_box = QtWidgets.QDoubleSpinBox()
            spin_box.setRange(-10_000.0, 10_000.0)
            spin_box.setDecimals(3)
            spin_box.setSingleStep(1.0)
            spin_box.setFixedWidth(100)
            self._spins[axis] = spin_box
            lay_target.addWidget(spin_box)
            if axis != 'Z':
                lay_target.addSpacing(8)
        root_layout.addWidget(grp_target)

        # Target orientation RPY
        grp_rotation = QtWidgets.QGroupBox(tr('Target orientation (deg) — world frame'))
        lay_rotation = QtWidgets.QHBoxLayout(grp_rotation)
        for axis in ('Roll', 'Pitch', 'Yaw'):
            lay_rotation.addWidget(QtWidgets.QLabel(f'<b>{axis[0]}</b>'))
            spin_box = QtWidgets.QDoubleSpinBox()
            spin_box.setRange(-180.0, 180.0)
            spin_box.setDecimals(2)
            spin_box.setSingleStep(1.0)
            spin_box.setFixedWidth(100)
            self._spins[axis] = spin_box
            lay_rotation.addWidget(spin_box)
            if axis != 'Yaw':
                lay_rotation.addSpacing(8)
        root_layout.addWidget(grp_rotation)

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
            tr('Ready. Select an End Effector, enter a target and press Solve IK.')
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

    # ── Tree initialisation ───────────────────────────────────

    def _init_tree_maps(self) -> None:
        try:
            self._child_to_parent, self._child_to_joint = _build_parent_map(
                self._robot_obj
            )
        except Exception as exc:
            self._lbl_status.setText(f'Error reading robot tree: {exc}')
            self._btn_solve.setEnabled(False)

    def _populate_ee_combo(self) -> None:
        leaf_links = _find_leaf_links(self._robot_obj)
        self._combo_ee.blockSignals(True)
        self._combo_ee.clear()
        for link_label in leaf_links:
            self._combo_ee.addItem(link_label)
        self._combo_ee.blockSignals(False)
        if leaf_links:
            self._load_chain(leaf_links[0])

    # ── End Effector change ───────────────────────────────────

    def _on_ee_changed(self, index: int) -> None:
        ee_label = self._combo_ee.currentText()
        if not ee_label:
            return
        self.original_joint_positions = None
        self.solved_joint_positions   = None
        self._btn_apply.setEnabled(False)
        self._btn_restore.setEnabled(False)
        self._load_chain(ee_label)

    def _load_chain(self, end_effector_label: str) -> None:
        """
        Isolate the serial sub-chain and rebuild the RoboTool model.
        Pre-fills both XYZ and RPY spinboxes with the current EF pose
        in world coordinates, ensuring orientation is preserved on
        position-only moves.
        """
        try:
            chain_joints = _get_chain_to_root(
                self._robot_obj, end_effector_label,
                self._child_to_parent, self._child_to_joint,
            )
        except Exception as exc:
            self._lbl_status.setText(f'Error isolating chain: {exc}')
            self._btn_solve.setEnabled(False)
            return

        if not chain_joints:
            self._lbl_status.setText(
                tr(f'No active joints found on the path to "{end_effector_label}". '
                'Try a different end effector.')
            )
            self._btn_solve.setEnabled(False)
            self._joint_objs = []
            self._robot      = None
            return

        try:
            self._robot = _build_robot_for_chain(self._robot_obj, chain_joints)
        except Exception as exc:
            self._lbl_status.setText(f'Error building robot model: {exc}')
            self._btn_solve.setEnabled(False)
            return

        self._joint_objs = chain_joints
        n_joints = len(chain_joints)

        try:
            q0 = np.array([float(j.Position) for j in self._joint_objs])
        except Exception:
            self._lbl_status.setText(
                tr("Solver error: Cannot access 'Position' of deleted object. "
                "Please close and reopen the IK Tool.")
            )
            self._joint_objs = []
            self._robot = None
            self._btn_solve.setEnabled(False)
            return

        # Pre-fill XYZ and RPY from current FK pose in world frame.
        # Pre-filling RPY prevents orientation drift on position-only moves.
        try:
            T_robot_ee  = forward_kinematics(self._robot, q0)
            T_world     = _get_robot_global_transform(self._robot_obj)
            T_world_ee  = T_world @ T_robot_ee
            fk_position = T_world_ee[:3, 3]
            fk_rotation = T_world_ee[:3, :3]

            for axis, value in zip(('X', 'Y', 'Z'), fk_position):
                self._spins[axis].setValue(float(value))

            sy    = math.sqrt(fk_rotation[0, 0] ** 2 + fk_rotation[1, 0] ** 2)
            roll  = math.atan2(fk_rotation[2, 1], fk_rotation[2, 2])
            pitch = math.atan2(-fk_rotation[2, 0], sy)
            yaw   = math.atan2(fk_rotation[1, 0], fk_rotation[0, 0])
            self._spins['Roll'].setValue(math.degrees(roll))
            self._spins['Pitch'].setValue(math.degrees(pitch))
            self._spins['Yaw'].setValue(math.degrees(yaw))
        except Exception:
            pass

        self._table.setRowCount(n_joints)
        for i, joint_obj in enumerate(self._joint_objs):
            self._table.setItem(i, 0, QtWidgets.QTableWidgetItem(joint_obj.Label))
            self._table.setItem(i, 1, QtWidgets.QTableWidgetItem(f'{q0[i]:.4f}'))
            self._table.setItem(i, 2, QtWidgets.QTableWidgetItem('—'))
        self._table.resizeColumnsToContents()

        self._btn_solve.setEnabled(True)
        self._lbl_status.setText(
            tr(f'Chain: {n_joints} joint(s) → "{end_effector_label}". '
            'Adjust target and press Solve IK.')
        )

    # ── Slot: Solve IK (incremental) ─────────────────────────

    def _on_solve(self) -> None:
        """
        Run incremental IK: interpolate N_STEPS targets between the current
        EF position and the desired target, applying each step to the model
        in real time.  This produces smooth, continuous motion and prevents
        large joint-space jumps between kinematic branches.
        """
        if not self._joint_objs or self._robot is None:
            return

        try:
            # ── Build R_target_world from RPY spinboxes ────────
            roll  = np.deg2rad(self._spins['Roll'].value())
            pitch = np.deg2rad(self._spins['Pitch'].value())
            yaw   = np.deg2rad(self._spins['Yaw'].value())

            Rx = np.array([
                [1, 0,             0            ],
                [0, np.cos(roll), -np.sin(roll)  ],
                [0, np.sin(roll),  np.cos(roll)  ],
            ])
            Ry = np.array([
                [ np.cos(pitch), 0, np.sin(pitch)],
                [ 0,             1, 0            ],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ])
            Rz = np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw),  np.cos(yaw), 0],
                [0,            0,           1],
            ])
            R_target_world = Rz @ Ry @ Rx

            # ── Read target position (world frame) ─────────────
            target_world = np.array([
                self._spins[axis].value() for axis in ('X', 'Y', 'Z')
            ])

            # ── Read current joint positions ───────────────────
            try:
                q0 = np.array([float(j.Position) for j in self._joint_objs])
            except Exception:
                self._lbl_status.setText(
                    tr("Solver error: Cannot access 'Position' of deleted object. "
                    "Please close and reopen the IK Tool.")
                )
                return

            if self.original_joint_positions is None:
                self.original_joint_positions = q0.copy()
                self._btn_restore.setEnabled(True)

            # ── World → local frame conversion ─────────────────
            T_world  = _get_robot_global_transform(self._robot_obj)
            R_world  = T_world[:3, :3]
            p_world  = T_world[:3, 3]

            target_local   = R_world.T @ (target_world - p_world)
            R_target_local = R_world.T @ R_target_world

            # ── Incremental IK: interpolate N_STEPS sub-targets ─
            # Each step uses the previous step's q as initial guess,
            # keeping the robot on the same kinematic branch and
            # producing smooth, continuous motion in the viewport.
            T_ee_start = forward_kinematics(self._robot, q0)
            pos_start  = T_ee_start[:3, 3]   # current EF position (local)

            q_current   = q0.copy()
            q_final     = None
            steps_done  = 0

            for step in range(_N_STEPS):
                alpha     = (step + 1) / _N_STEPS
                pos_interp = pos_start + alpha * (target_local - pos_start)

                log_buf = io.StringIO()
                with redirect_stdout(log_buf):
                    q_new = inverse_kinematics(
                        self._robot,
                        target_position=pos_interp,
                        initial_guess=q_current,
                        target_rotation=R_target_local,
                    )

                if q_new is None:
                    # Stop at last valid configuration — don't revert.
                    break

                # Apply this step to the model immediately (live animation).
                _apply_joint_positions(
                    self._robot_obj, self._joint_objs, q_new, self._doc,
                    transaction_label='IK Tool – step'
                )
                q_current  = q_new
                q_final    = q_new
                steps_done += 1

            if q_final is None:
                self._lbl_status.setText(
                    tr('IK did not converge on first step. '
                    'Target may be outside the workspace.')
                )
                self._btn_apply.setEnabled(False)
                return

            self.solved_joint_positions = q_final

            # ── Update results table ───────────────────────────
            for i in range(len(self._joint_objs)):
                self._table.setItem(
                    i, 1,
                    QtWidgets.QTableWidgetItem(f'{self.original_joint_positions[i]:.4f}')
                )
                self._table.setItem(
                    i, 2,
                    QtWidgets.QTableWidgetItem(f'{q_final[i]:.4f}')
                )
            self._table.resizeColumnsToContents()

            # ── FK verification in world frame ─────────────────
            try:
                T_solved      = forward_kinematics(self._robot, q_final)
                T_world_solved = T_world @ T_solved
                achieved_world = T_world_solved[:3, 3]
                R_achieved     = T_world_solved[:3, :3]

                pos_error = float(np.linalg.norm(target_world - achieved_world))
                cos_angle = np.clip(
                    (np.trace(R_achieved.T @ R_target_world) - 1) / 2,
                    -1.0, 1.0
                )
                partial = (
                    f' (partial: {steps_done}/{_N_STEPS} steps)'
                    if steps_done < _N_STEPS else f' ({steps_done}/{_N_STEPS} steps)'
                )
                status_details = (
                    f'Converged{partial}\n'
                    f'Target:   [{target_world[0]:.3f}, {target_world[1]:.3f}, {target_world[2]:.3f}] mm\n'
                    f'Achieved: [{achieved_world[0]:.3f}, {achieved_world[1]:.3f}, {achieved_world[2]:.3f}] mm\n'
                    f'Error:    {pos_error:.6f} mm\n'
                    f'Error orientation: {np.rad2deg(np.arccos(cos_angle)):.2f} deg'
                )
            except Exception as exc:
                status_details = f'(FK verification failed: {exc})'

            self._lbl_status.setText(status_details)
            self._btn_apply.setEnabled(True)

        except Exception as exc:
            self._lbl_status.setText(f'Solver error: {exc}')
            self._btn_apply.setEnabled(False)

    # ── Slot: Apply solution ──────────────────────────────────

    def _on_apply(self) -> None:
        """Write solved joint positions to the document and animate the model."""
        if self.solved_joint_positions is None:
            return
        _apply_joint_positions(
            self._robot_obj, self._joint_objs, self.solved_joint_positions,
            self._doc, transaction_label='IK Tool – apply',
        )
        self._lbl_status.setText(
            self._lbl_status.text() + tr('\n✔ Solution applied to model.')
        )

    # ── Slot: Restore original pose ───────────────────────────

    def _on_restore(self) -> None:
        """Write the original joint positions back to the document."""
        if self.original_joint_positions is None:
            return
        _apply_joint_positions(
            self._robot_obj, self._joint_objs, self.original_joint_positions,
            self._doc, transaction_label='IK Tool – restore',
        )
        self._lbl_status.setText(tr('↩ Original joint positions restored.'))


# ─────────────────────────────────────────────────────────────────────────────
# FreeCAD Gui Command
# ─────────────────────────────────────────────────────────────────────────────

class _IKToolCommand:
    """FreeCAD Gui command that opens the IK Tool dialog."""

    def GetResources(self) -> dict:
        return {
            'Pixmap'  : 'ik_solver.svg',
            'MenuText': tr('IK Tool'),
            'Accel'   : 'I, K',
            'ToolTip' : tr(
                'Forward & Inverse Kinematics Tool (RoboTool)\n'
                '\n'
                'Select a robot, then open this tool to:\n'
                '  • Choose the End Effector (tip link) for IK.\n'
                '  • Inspect the current end-effector pose (FK)\n'
                '    in WORLD coordinates (position + orientation).\n'
                '  • Enter a Cartesian target (X, Y, Z in mm) and\n'
                '    orientation (Roll, Pitch, Yaw in degrees).\n'
                '  • Solve IK with smooth incremental motion —\n'
                '    the robot moves continuously to the target\n'
                '    without large joint-space jumps.\n'
                '  • Apply the solution → live 3-D animation.\n'
                '  • Restore the original pose at any time.\n'
                '\n'
                'Requires: a Cross::Robot selected in the scene.\n'
                'Algorithm: Damped Least Squares (DLS), 6-DOF,\n'
                f'           {_N_STEPS} interpolation steps per solve.\n'
            ),
        }

    def IsActive(self) -> bool:
        selection = fcgui.Selection.getSelection()
        return bool(selection) and is_robot(selection[0])

    def Activated(self) -> None:
        robot_obj = fcgui.Selection.getSelection()[0]
        dialog = IKToolDialog(robot_obj, parent=None)

        if _PYSIDE6:
            _wa_del = QtCore.Qt.WidgetAttribute.WA_DeleteOnClose
        else:
            _wa_del = QtCore.Qt.WA_DeleteOnClose
        dialog.setAttribute(_wa_del, False)

        self._dialog = dialog
        dialog.show()


# ─────────────────────────────────────────────────────────────────────────────
# Registration
# ─────────────────────────────────────────────────────────────────────────────

fcgui.addCommand('IKTool', _IKToolCommand())