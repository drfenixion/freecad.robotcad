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
│                            robot's kinematic tree.
├── Target position group  - X, Y, Z in mm (world frame, pre-filled from FK)
├── Target orientation group - Roll, Pitch, Yaw in deg (world frame, pre-filled)
├── [Solve IK]    - incremental IK: N_STEPS targets interpolated with
│                   linear position + SLERP orientation.  Each step is
│                   applied to the model in real time (smooth animation).
│                   A temporary App::Origin marker (IK_EE_Marker) is
│                   created in the viewport at the EF position and updated
│                   after each solve so the user can verify world-frame
│                   coordinates visually.  The marker is removed on Close.
├── Results table - Joint | Original (rad/mm) | Solved (rad/mm)
├── Status label  - convergence info + position/orientation errors
├── [Apply to Model]   - write q_solved to document + recompute
├── [Restore Original] - write q_original back
└── [Close]            - removes EE marker from scene
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dialog
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
│  Status: Converged in N iterations.                    │
│  Target:   [x, y, z] mm  (world)                       │
│  Achieved: [x, y, z] mm  (world)                       │
│  Error:    0.000001 mm                                 │
│  Error orientation: 0.42 deg                           │
│                                                        │
│  [ ▶ Apply to Model ]  [ ↩ Restore Original ]  [Close] │
└────────────────────────────────────────────────────────┘

┌─ EF Position (world frame) ────────────────────────────┐
│  Pos X:  100.000 mm    Pos Y:  -48.000 mm    Pos Z:  588.000 mm  │
│  Roll:     0.00 °      Pitch:   -2.53 °      Yaw:   -16.49 °     │
└────────────────────────────────────────────────────────┘

Incremental IK with SLERP orientation interpolation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Position:    p(α) = p_start + α · (p_target − p_start)
Orientation: R(α) = R_start · exp(α · log(R_start^T · R_target))
            computed via Rodrigues rotation formula (SLERP)

EE Coordinate Marker
~~~~~~~~~~~~~~~~~~~~~
An App::Origin object named "IK_EE_Marker" is added to the document when
the dialog opens and positioned at the end-effector location after each
solve.  Its placement is expressed in WORLD coordinates.  The marker is
automatically removed when the dialog is closed.

Author  : RoboTool / RobotCAD integration
License : MIT
"""

from __future__ import annotations

import math

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
from ..kinematics.jacobians import compute_jacobian

# Default SLERP steps — overridable from the dialog spinbox.
_N_STEPS_DEFAULT = 50


# ─────────────────────────────────────────────────────────────────────────────
# Kinematic-tree helpers
# ─────────────────────────────────────────────────────────────────────────────

def _build_parent_map(
    robot_obj: fc.App.DocumentObject,
) -> tuple[dict[str, str], dict[str, fc.App.DocumentObject]]:
    child_to_parent: dict[str, str] = {}
    child_to_joint:  dict[str, fc.App.DocumentObject] = {}
    for joint_obj in robot_obj.Proxy.get_joints():
        child_to_parent[joint_obj.Child] = joint_obj.Parent
        child_to_joint[joint_obj.Child]  = joint_obj
    return child_to_parent, child_to_joint


def _find_leaf_links(robot_obj: fc.App.DocumentObject) -> list[str]:
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
    p = placement.Base
    yaw, pitch, roll = placement.Rotation.toEuler()
    return (
        (p.x, p.y, p.z),
        (math.radians(roll), math.radians(pitch), math.radians(yaw)),
    )


def _get_axis_from_placement(placement: fc.Placement) -> tuple[float, float, float]:
    v = placement.Rotation.multVec(fc.Vector(0, 0, 1))
    return (v.x, v.y, v.z)


def _build_robot_for_chain(
    robot_obj:    fc.App.DocumentObject,
    chain_joints: list[fc.App.DocumentObject],
) -> Robot:
    robot_instance = Robot(name=robot_obj.Label)
    if not chain_joints:
        return robot_instance
    robot_instance.links.append(Link(name=chain_joints[0].Parent))
    for joint_obj in chain_joints:
        xyz, rpy = _placement_to_xyz_rpy(joint_obj.Origin)
        axis     = _get_axis_from_placement(joint_obj.Origin)

        lower = getattr(joint_obj, 'LowerLimit', None)
        upper = getattr(joint_obj, 'UpperLimit', None)

        # RobotCAD leaves LowerLimit = UpperLimit = 0 when limits are not
        # configured. Passing 0/0 to the solver is interpreted as "joint is
        # locked at 0" — the solution gets clamped and the robot snaps back.
        # Treat the 0/0 case as unconstrained (None) instead.
        if lower is not None and upper is not None:
            try:
                if float(lower) == 0.0 and float(upper) == 0.0:
                    lower = None
                    upper = None
                elif joint_obj.Type != 'prismatic':
                    # RobotCAD stores revolute limits in degrees; convert to
                    # radians so the solver compares them against q in radians.
                    lower = math.radians(float(lower))
                    upper = math.radians(float(upper))
            except (TypeError, ValueError):
                lower = None
                upper = None

        robot_instance.joints.append(Joint(
            name           = joint_obj.Label,
            joint_type     = joint_obj.Type,
            parent         = joint_obj.Parent,
            child          = joint_obj.Child,
            origin_xyz     = xyz,
            origin_rpy     = rpy,
            axis           = axis,
            limit_lower    = lower,
            limit_upper    = upper,
            velocity_limit = getattr(joint_obj, 'Velocity',   None),
            effort_limit   = getattr(joint_obj, 'Effort',     None),
        ))
        robot_instance.links.append(Link(name=joint_obj.Child))
    return robot_instance


def _get_robot_global_transform(robot_obj: fc.App.DocumentObject) -> np.ndarray:
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


def _slerp_rotation(
    R_start:  np.ndarray,
    R_target: np.ndarray,
    alpha:    float,
) -> np.ndarray:
    """
    SLERP between two rotation matrices via Rodrigues formula.
    R(α) = R_start · exp(α · log(R_start^T · R_target))

    Edge cases handled:
    - theta ≈ 0   (nearly identical orientations): returns R_target directly.
    - theta ≈ π   (nearly opposite orientations): sin(theta) → 0, axis is
                  ill-defined by this formula. Returns R_start for alpha < 1,
                  R_target for alpha >= 1 to avoid NaN/inf in the axis vector.
                  A full antipodal SLERP would require quaternions; this is a
                  safe conservative fallback for the incremental-IK use case.
    """
    R_rel = R_start.T @ R_target
    theta = np.arccos(np.clip((np.trace(R_rel) - 1) / 2.0, -1.0, 1.0))

    # Bug fix #3: guard both degenerate cases (theta ≈ 0 and theta ≈ π)
    if theta < 1e-9:
        return R_target.copy()
    if np.pi - theta < 1e-6:
        # Antipodal case: axis is numerically undefined via skew-symmetric
        # extraction. Skip smooth interpolation and snap to nearest endpoint.
        return R_target.copy() if alpha >= 1.0 else R_start.copy()

    axis = np.array([
        R_rel[2, 1] - R_rel[1, 2],
        R_rel[0, 2] - R_rel[2, 0],
        R_rel[1, 0] - R_rel[0, 1],
    ]) / (2.0 * np.sin(theta))
    K = np.array([
        [ 0,        -axis[2],  axis[1]],
        [ axis[2],   0,       -axis[0]],
        [-axis[1],   axis[0],  0      ],
    ])
    a = alpha * theta
    R_step = np.eye(3) + np.sin(a) * K + (1.0 - np.cos(a)) * (K @ K)
    return R_start @ R_step


def _read_joint_positions(
    joint_objects: list[fc.App.DocumentObject],
) -> np.ndarray:
    """
    Read current joint positions from FreeCAD and return them in solver units:
      - revolute  → radians  (FreeCAD stores degrees, convert with math.radians)
      - prismatic → mm       (FreeCAD stores mm, pass through as-is)

    This is the inverse of the conversion done in _apply_joint_positions, which
    calls math.degrees() before writing revolute values back to the document.
    Without this conversion, q0 arrives at the IK solver in degrees while the
    solver expects radians — causing FK errors of ~57x and position drift > 100 mm.
    """
    q = np.empty(len(joint_objects))
    for i, j in enumerate(joint_objects):
        raw = float(j.Position)
        q[i] = raw if j.Type == 'prismatic' else math.radians(raw)
    return q


def _write_joint_values(
    robot_obj:     fc.App.DocumentObject,
    joint_objects: list[fc.App.DocumentObject],
    joint_values:  np.ndarray,
) -> None:
    """
    Write joint values to the FreeCAD document WITHOUT opening a transaction
    or calling recompute.  Used for animation steps inside a single outer
    transaction so that the undo stack gets one entry per Solve, not one
    per SLERP step.
    """
    joint_variables: dict = robot_obj.Proxy.joint_variables
    for joint_obj, value in zip(joint_objects, joint_values):
        prop_name = joint_variables.get(joint_obj)
        if prop_name is None:
            continue
        native_value = (
            float(value) if joint_obj.Type == 'prismatic'
            else math.degrees(float(value))
        )
        setattr(robot_obj, prop_name, native_value)


def _apply_joint_positions(
    robot_obj:         fc.App.DocumentObject,
    joint_objects:     list[fc.App.DocumentObject],
    joint_values:      np.ndarray,
    doc:               fc.App.Document,
    transaction_label: str = 'IK Tool',
) -> None:
    """
    Write joint values inside a single transaction and recompute.
    Used for Apply to Model, Restore Original, and the final step of Solve.
    """
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
    """Non-modal IK Tool dialog with EE coordinate marker."""

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

        # Bug fix #2: use the robot's own document instead of fc.activeDocument(),
        # which may point to a different document when multiple docs are open.
        self._doc          = robot_obj.Document
        self._robot_obj    = robot_obj
        self._robot        = None
        self._joint_objs   = []
        self.original_joint_positions = None
        self.solved_joint_positions   = None
        self._ee_marker    = None  # temporary App::Origin in the viewport
        self._solve_count  = 0     # increments with each successful Solve

        self._child_to_parent: dict[str, str] = {}
        self._child_to_joint:  dict[str, fc.App.DocumentObject] = {}

        self._build_ui()
        self._init_tree_maps()
        self._populate_ee_combo()

    # ── EE marker ────────────────────────────────────────────

    def _create_ee_marker(self) -> None:
        """
        Add a temporary App::Origin object to the scene at the EF location.
        The marker shows the world-frame coordinate axes at the end-effector,
        letting the user verify the actual EF position visually.
        It is removed automatically when the dialog is closed.
        """
        try:
            self._ee_marker = self._doc.addObject('App::Origin', 'IK_EE_Marker')
            if hasattr(self._ee_marker, 'ViewObject'):
                vo = self._ee_marker.ViewObject
                if hasattr(vo, 'AxisSize'):
                    vo.AxisSize = 30.0
                if hasattr(vo, 'PlaneSize'):
                    vo.PlaneSize = 20.0
            self._doc.recompute()
        except Exception:
            self._ee_marker = None

    def _update_ee_marker(self, T_world_ee: np.ndarray) -> None:
        """
        Move the EE marker to the current end-effector pose in world coordinates.

        Parameters
        ----------
        T_world_ee : (4,4) homogeneous transform of the EF in world frame.
        """
        if self._ee_marker is None:
            return
        try:
            p = T_world_ee[:3, 3]
            R = T_world_ee[:3, :3]
            # Build FreeCAD Matrix from rotation (column-major, 1-indexed)
            fc_matrix = fc.Matrix(
                R[0, 0], R[0, 1], R[0, 2], p[0],
                R[1, 0], R[1, 1], R[1, 2], p[1],
                R[2, 0], R[2, 1], R[2, 2], p[2],
                0,       0,       0,       1,
            )
            self._ee_marker.Placement = fc.Placement(fc_matrix)
            self._doc.recompute()
        except Exception:
            pass

    def _remove_ee_marker(self) -> None:
        """Remove the temporary EE marker from the document."""
        if self._ee_marker is not None:
            try:
                self._doc.removeObject(self._ee_marker.Name)
                self._doc.recompute()
            except Exception:
                pass
            self._ee_marker = None

    def closeEvent(self, event) -> None:
        """Remove the EE marker when the dialog is closed."""
        self._remove_ee_marker()
        super().closeEvent(event)

    def reject(self) -> None:
        """Handle Close button — remove marker then close."""
        self._remove_ee_marker()
        super().reject()

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
            spin_box.setToolTip(tr('Edit freely — press ⚙ Solve IK when ready.'))
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
            spin_box.setToolTip(tr('Edit freely — press ⚙ Solve IK when ready.'))
            self._spins[axis] = spin_box
            lay_rotation.addWidget(spin_box)
            if axis != 'Yaw':
                lay_rotation.addSpacing(8)
        root_layout.addWidget(grp_rotation)

        # "Refresh" button — reads current robot pose and fills spinboxes + display.
        btn_refresh = QtWidgets.QPushButton(tr('↺  Read current pose'))
        btn_refresh.setToolTip(tr(
            'Read the current joint positions from the model and update\n'
            'the target spinboxes and EF pose display.\n'
            'Use this after moving joints manually in FreeCAD.'
        ))
        btn_refresh.clicked.connect(self._refresh_ef_pose)
        root_layout.addWidget(btn_refresh)

        # Solver parameters
        grp_params = QtWidgets.QGroupBox(tr('Solver parameters'))
        lay_params = QtWidgets.QGridLayout(grp_params)

        lay_params.addWidget(QtWidgets.QLabel(tr('SLERP steps')), 0, 0)
        self._spin_steps = QtWidgets.QSpinBox()
        self._spin_steps.setRange(5, 200)
        self._spin_steps.setValue(_N_STEPS_DEFAULT)
        self._spin_steps.setToolTip(tr(
            'Number of intermediate targets interpolated between current\n'
            'pose and goal. More steps = smoother motion and better\n'
            'convergence for large displacements, but slower solve.'
        ))
        lay_params.addWidget(self._spin_steps, 0, 1)

        lay_params.addWidget(QtWidgets.QLabel(tr('Max iter / step')), 0, 2)
        self._spin_maxiter = QtWidgets.QSpinBox()
        self._spin_maxiter.setRange(100, 5000)
        self._spin_maxiter.setValue(1000)
        self._spin_maxiter.setSingleStep(100)
        self._spin_maxiter.setToolTip(tr(
            'Maximum DLS iterations per SLERP step.\n'
            'Increase if the solver stalls before converging.'
        ))
        lay_params.addWidget(self._spin_maxiter, 0, 3)

        lay_params.addWidget(QtWidgets.QLabel(tr('Ori weight')), 1, 0)
        self._spin_ori_weight = QtWidgets.QDoubleSpinBox()
        self._spin_ori_weight.setRange(0.0, 10.0)
        self._spin_ori_weight.setValue(0.3)
        self._spin_ori_weight.setSingleStep(0.01)
        self._spin_ori_weight.setDecimals(3)
        self._spin_ori_weight.setToolTip(tr(
            'Weight of orientation error relative to position error.\n'
            '0 = ignore orientation, 1 = equal weight.\n'
            'Lower values let the solver prioritize reaching the position.\n'
            'Recommended: 0.0–0.1 for arms with coaxial joints.'
        ))
        lay_params.addWidget(self._spin_ori_weight, 1, 1)

        lay_params.addWidget(QtWidgets.QLabel(tr('Max restarts')), 1, 2)
        self._spin_restarts = QtWidgets.QSpinBox()
        self._spin_restarts.setRange(0, 20)
        self._spin_restarts.setValue(5)
        self._spin_restarts.setToolTip(tr(
            'Random restarts when the DLS stalls.\n'
            'More restarts help escape local minima near singularities.'
        ))
        lay_params.addWidget(self._spin_restarts, 1, 3)

        lay_params.addWidget(QtWidgets.QLabel(tr('Tolerance (mm)')), 2, 0)
        self._spin_tol = QtWidgets.QDoubleSpinBox()
        self._spin_tol.setRange(1e-6, 10.0)
        self._spin_tol.setValue(0.001)
        self._spin_tol.setDecimals(4)
        self._spin_tol.setSingleStep(0.001)
        self._spin_tol.setToolTip(tr(
            'Position convergence threshold in mm.\n'
            'Solver stops when position error < this value.\n'
            'Tighter = more accurate but may not converge for distant targets.'
        ))
        lay_params.addWidget(self._spin_tol, 2, 1)

        lay_params.addWidget(QtWidgets.QLabel(tr('Pos weight')), 2, 2)
        self._spin_pos_weight = QtWidgets.QDoubleSpinBox()
        self._spin_pos_weight.setRange(0.1, 10.0)
        self._spin_pos_weight.setValue(1.0)
        self._spin_pos_weight.setSingleStep(0.1)
        self._spin_pos_weight.setDecimals(2)
        self._spin_pos_weight.setToolTip(tr(
            'Weight of position error in the DLS cost function.\n'
            'Increase to make the solver more aggressive on position.'
        ))
        lay_params.addWidget(self._spin_pos_weight, 2, 3)

        self._chk_verbose = QtWidgets.QCheckBox(tr('Verbose solver output'))
        self._chk_verbose.setChecked(False)
        self._chk_verbose.setToolTip(tr(
            'Print convergence details to the FreeCAD console.\n'
            'Useful for debugging; disable for normal use.'
        ))
        lay_params.addWidget(self._chk_verbose, 3, 0, 1, 4)

        root_layout.addWidget(grp_params)

        # Solve button
        self._btn_solve = QtWidgets.QPushButton(tr('⚙  Solve IK'))
        self._btn_solve.setFixedHeight(34)
        self._btn_solve.setDefault(True)
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

        # Solve log — accumulates results of every Solve call in this session.
        grp_log = QtWidgets.QGroupBox(tr('Solve log'))
        lay_log = QtWidgets.QVBoxLayout(grp_log)
        self._log = QtWidgets.QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(500)   # keep last ~500 lines
        self._log.setMinimumHeight(100)
        self._log.setMaximumHeight(160)
        mono = QtWidgets.QApplication.font()
        mono.setFamily('Courier New' if _PYSIDE6 else 'Courier New, monospace')
        mono.setPointSize(max(8, mono.pointSize() - 1))
        self._log.setFont(mono)
        lay_log_btns = QtWidgets.QHBoxLayout()
        btn_clear_log = QtWidgets.QPushButton(tr('Clear'))
        btn_clear_log.setFixedWidth(60)
        btn_clear_log.clicked.connect(self._log.clear)
        lay_log_btns.addStretch()
        lay_log_btns.addWidget(btn_clear_log)
        lay_log.addWidget(self._log)
        lay_log.addLayout(lay_log_btns)
        root_layout.addWidget(grp_log)

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

    # ── EF current pose display ────────────────────────────
        grp_ef_pose = QtWidgets.QGroupBox(tr('End Effector — current pose (world frame)'))
        grid_pose = QtWidgets.QGridLayout(grp_ef_pose)

        self._pose_labels: dict[str, QtWidgets.QLabel] = {}
        labels = [('X', 0, 0), ('Y', 0, 2), ('Z', 0, 4),
        ('Roll', 1, 0), ('Pitch', 1, 2), ('Yaw', 1, 4)]

        for name, row, col in labels:
            unit = 'mm' if name in ('X','Y','Z') else '°'
            grid_pose.addWidget(QtWidgets.QLabel(f'<b>{name}:</b>'), row, col)
            lbl = QtWidgets.QLabel(f'—  {unit}')
            lbl.setMinimumWidth(90)
            lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight if _PYSIDE6
                    else QtCore.Qt.AlignRight)
            self._pose_labels[name] = lbl
            grid_pose.addWidget(lbl, row, col + 1)

        root_layout.addWidget(grp_ef_pose)

    # ── Method update coordinates ───────────────────────────────────
    def _update_ef_pose_display(self, T_world_ee: np.ndarray) -> None:
        """
        Update the EF current pose display in the dialog with the given
        world-frame homogeneous transform of the end-effector.
        """
        P = T_world_ee[:3, 3]
        R = T_world_ee[:3, :3]
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        pitch = math.degrees(math.atan2(-R[2, 0], sy))
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        # Update labels in the dialog
        self._pose_labels['X'].setText(f'{P[0]:.3f} mm')
        self._pose_labels['Y'].setText(f'{P[1]:.3f} mm')
        self._pose_labels['Z'].setText(f'{P[2]:.3f} mm')
        self._pose_labels['Roll'].setText(f'{roll:.2f} °')
        self._pose_labels['Pitch'].setText(f'{pitch:.2f} °')
        self._pose_labels['Yaw'].setText(f'{yaw:.2f} °')


    # ── Solve log ─────────────────────────────────────────────

    def _log_solve_result(
        self,
        solve_n:       int,
        ee_label:      str,
        target_world:  np.ndarray,
        achieved_world: np.ndarray,
        R_achieved:    np.ndarray,
        R_target_world: np.ndarray,
        q_start:       np.ndarray,
        q_final:       np.ndarray,
        steps_done:    int,
        n_steps:       int,
    ) -> None:
        """Append a formatted solve summary to the log panel."""
        pos_error = float(np.linalg.norm(target_world - achieved_world))
        cos_angle = np.clip(
            (np.trace(R_achieved.T @ R_target_world) - 1) / 2, -1.0, 1.0
        )
        ori_error = float(np.rad2deg(np.arccos(cos_angle)))

        sep  = '─' * 52
        lines = [
            sep,
            f'Solve #{solve_n}  EF: {ee_label}  ({steps_done}/{n_steps} steps)',
            f'  Target   X={target_world[0]:10.3f}  Y={target_world[1]:10.3f}  Z={target_world[2]:10.3f}  mm',
            f'  Achieved X={achieved_world[0]:10.3f}  Y={achieved_world[1]:10.3f}  Z={achieved_world[2]:10.3f}  mm',
            f'  Pos error : {pos_error:.4f} mm      Ori error: {ori_error:.4f} deg',
            '  Joints:',
        ]
        for jobj, q0_i, qf_i in zip(self._joint_objs, q_start, q_final):
            unit   = 'mm' if jobj.Type == 'prismatic' else 'rad'
            delta  = qf_i - q0_i
            lines.append(
                f'    {jobj.Label:<20s}  {q0_i:+.4f} → {qf_i:+.4f}  (Δ {delta:+.4f}) {unit}'
            )
        self._log.appendPlainText('\n'.join(lines))
        # Scroll to bottom
        sb = self._log.verticalScrollBar()
        sb.setValue(sb.maximum())

    # ── Manual EF pose refresh ────────────────────────────────

    def _update_pose_display_from_model(self) -> None:
        """
        Read current joint positions, run FK, and update ONLY:
          - The EF pose display panel (X/Y/Z/Roll/Pitch/Yaw labels).
          - The IK_EE_Marker position in the viewport.

        Does NOT touch the target spinboxes — used after Solve so the
        user's intended target values are preserved for the next Solve.
        """
        if not self._joint_objs or self._robot is None:
            return
        try:
            q_current  = _read_joint_positions(self._joint_objs)
            T_world    = _get_robot_global_transform(self._robot_obj)
            T_world_ee = T_world @ forward_kinematics(self._robot, q_current)
        except Exception:
            return
        self._update_ef_pose_display(T_world_ee)
        if self._ee_marker is not None:
            self._update_ee_marker(T_world_ee)

    def _refresh_ef_pose(self) -> None:
        """
        Read current joint positions, run FK, and update BOTH:
          - The EF pose display panel.
          - The target spinboxes (syncs them to the current robot pose).

        Called when the user explicitly requests a pose sync:
          - When a chain is loaded (_load_chain).
          - After Restore Original (_on_restore).
          - When the user presses the ↺ Read current pose button.

        NOT called after Solve — use _update_pose_display_from_model() instead
        so the target spinboxes keep the user's intended target values.
        """
        if not self._joint_objs or self._robot is None:
            return
        try:
            q_current  = _read_joint_positions(self._joint_objs)
            T_world    = _get_robot_global_transform(self._robot_obj)
            T_world_ee = T_world @ forward_kinematics(self._robot, q_current)
        except Exception:
            return
        self._update_ef_pose_display(T_world_ee)
        if self._ee_marker is not None:
            self._update_ee_marker(T_world_ee)
        # Reset cached solution so the next Solve reads fresh from the model.
        self.solved_joint_positions = None
        # Sync spinboxes to actual pose.
        pos = T_world_ee[:3, 3]
        rot = T_world_ee[:3, :3]
        sy    = math.sqrt(rot[0, 0] ** 2 + rot[1, 0] ** 2)
        roll  = math.degrees(math.atan2(rot[2, 1], rot[2, 2]))
        pitch = math.degrees(math.atan2(-rot[2, 0], sy))
        yaw   = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
        for spin in self._spins.values():
            spin.blockSignals(True)
        self._spins['X'].setValue(float(pos[0]))
        self._spins['Y'].setValue(float(pos[1]))
        self._spins['Z'].setValue(float(pos[2]))
        self._spins['Roll'].setValue(roll)
        self._spins['Pitch'].setValue(pitch)
        self._spins['Yaw'].setValue(yaw)
        for spin in self._spins.values():
            spin.blockSignals(False)

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
            q0 = _read_joint_positions(self._joint_objs)
        except Exception:
            self._lbl_status.setText(
                tr("Solver error: Cannot access 'Position' of deleted object. "
                "Please close and reopen the IK Tool.")
            )
            self._joint_objs = []
            self._robot = None
            self._btn_solve.setEnabled(False)
            return
        try:
            if self._ee_marker is None:
                self._create_ee_marker()
            self._refresh_ef_pose()
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

    # ── Slot: Solve IK (incremental with SLERP) ──────────────

    def _on_solve(self) -> None:
        if not self._joint_objs or self._robot is None:
            return

        try:
            # Build R_target_world from RPY spinboxes
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

            target_world = np.array([
                self._spins[axis].value() for axis in ('X', 'Y', 'Z')
            ])

            # Use the cached solution from the previous Solve as q0 when
            # available. Reading j.Position from the FreeCAD model is unreliable
            # for chained solves — the document may not have flushed the previous
            # transaction yet, causing the robot to appear to reset to home.
            # Fall back to reading from the model only on the first Solve or
            # after the user explicitly resets with ↺ Read current pose.
            if self.solved_joint_positions is not None:
                q0 = self.solved_joint_positions.copy()
            else:
                try:
                    q0 = _read_joint_positions(self._joint_objs)
                except Exception:
                    self._lbl_status.setText(
                        tr("Solver error: Cannot access 'Position' of deleted object. "
                        "Please close and reopen the IK Tool.")
                    )
                    return

            if self.original_joint_positions is None:
                self.original_joint_positions = q0.copy()
                self._btn_restore.setEnabled(True)

            # World → local frame
            T_world        = _get_robot_global_transform(self._robot_obj)
            R_world        = T_world[:3, :3]
            p_world        = T_world[:3, 3]
            target_local   = R_world.T @ (target_world - p_world)
            R_target_local = R_world.T @ R_target_world

            # Current EF pose in local frame — read from the live model q0,
            # so each chained Solve starts the SLERP from where the robot actually is.
            T_ee_start = forward_kinematics(self._robot, q0)
            pos_start  = T_ee_start[:3, 3]
            R_start    = T_ee_start[:3, :3]

            # Incremental IK: linear position + SLERP orientation
            n_steps    = self._spin_steps.value()
            max_iter   = self._spin_maxiter.value()
            ori_weight = self._spin_ori_weight.value()
            restarts   = self._spin_restarts.value()
            tolerance  = self._spin_tol.value()
            pos_weight = self._spin_pos_weight.value()
            verbose    = self._chk_verbose.isChecked()

            q_current  = q0.copy()
            q_final    = None
            steps_done = 0

            # Pre-warm: if the initial config is singular, run one IK step toward
            # a tiny displacement (1% of total travel) to get a non-singular q
            # before the main SLERP loop. This prevents _escape_singularity from
            # landing on a random pose that becomes step 1 of the animation.
            J_check = compute_jacobian(self._robot, q_current)
            w_check = float(np.sqrt(max(0.0, np.linalg.det(
                J_check[:3, :] @ J_check[:3, :].T
            ))))
            if w_check < 1e-4:
                pos_prewarm = pos_start + 0.01 * (target_local - pos_start)
                try:
                    q_warm = inverse_kinematics(
                        self._robot,
                        target_position    = pos_prewarm,
                        initial_guess      = q_current,
                        target_rotation    = R_start,
                        weight_position    = pos_weight,
                        weight_orientation = 0.0,
                        max_iterations     = max_iter,
                        tolerance          = np.linalg.norm(target_local - pos_start) * 0.005,
                        max_restarts       = restarts,
                        verbose            = verbose,
                    )
                    if q_warm is not None:
                        q_current = q_warm
                except Exception:
                    pass

            # Single transaction for the whole Solve — one undo entry, no
            # partial revert between steps that was causing the robot to
            # reset to home when starting a second Solve.
            self._doc.openTransaction(tr('IK Tool – solve'))

            for step in range(n_steps):
                alpha      = (step + 1) / n_steps
                pos_interp = pos_start + alpha * (target_local - pos_start)
                R_interp   = _slerp_rotation(R_start, R_target_local, alpha)

                try:
                    q_new = inverse_kinematics(
                        self._robot,
                        target_position    = pos_interp,
                        initial_guess      = q_current,
                        target_rotation    = R_interp,
                        weight_position    = pos_weight,
                        weight_orientation = ori_weight,
                        max_iterations     = max_iter,
                        tolerance          = tolerance,
                        max_restarts       = restarts,
                        verbose            = verbose,
                    )
                except Exception as solver_exc:
                    self._lbl_status.setText(f'Solver exception at step {step+1}: {solver_exc}')
                    break

                if q_new is None:
                    break

                # Write values without recompute for smooth animation.
                # The viewport refreshes via updateGui() after each step.
                _write_joint_values(self._robot_obj, self._joint_objs, q_new)
                self._doc.recompute()
                fcgui.updateGui()

                q_current  = q_new
                q_final    = q_new
                steps_done += 1

            self._doc.commitTransaction()

            if q_final is None:
                self._lbl_status.setText(
                    tr('IK did not converge on first step. '
                    'Target may be outside the workspace.')
                )
                self._btn_apply.setEnabled(False)
                return

            self.solved_joint_positions = q_final

            # Update results table
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

            # Bug fix #1: removed duplicate/buggy block that used `q_new` (loop-local
            # variable, potentially None if last step failed) instead of `q_final`.
            # FK verification and display update now happen once below, using q_final.

            # FK verification + update EE marker to achieved world position
            try:
                T_solved        = forward_kinematics(self._robot, q_final)
                T_world_solved  = T_world @ T_solved
                achieved_world  = T_world_solved[:3, 3]
                R_achieved      = T_world_solved[:3, :3]

                if self._ee_marker is None:
                    self._create_ee_marker()

                self._update_ee_marker(T_world_solved)
                self._update_ef_pose_display(T_world_solved)

                # Update only the pose display panel — do NOT sync spinboxes
                # so the user's target values are preserved for the next Solve.

                pos_error = float(np.linalg.norm(target_world - achieved_world))
                cos_angle = np.clip(
                    (np.trace(R_achieved.T @ R_target_world) - 1) / 2,
                    -1.0, 1.0
                )
                step_info = (
                    f'partial: {steps_done}/{n_steps} steps'
                    if steps_done < n_steps
                    else f'{steps_done}/{n_steps} steps'
                )
                status_details = (
                    f'Converged ({step_info})\n'
                    f'Target:   [{target_world[0]:.3f}, {target_world[1]:.3f}, {target_world[2]:.3f}] mm\n'
                    f'Achieved: [{achieved_world[0]:.3f}, {achieved_world[1]:.3f}, {achieved_world[2]:.3f}] mm\n'
                    f'Error:    {pos_error:.6f} mm\n'
                    f'Error orientation: {np.rad2deg(np.arccos(cos_angle)):.2f} deg'
                )

                # Log full result to the solve log panel.
                self._solve_count += 1
                self._log_solve_result(
                    solve_n        = self._solve_count,
                    ee_label       = self._combo_ee.currentText(),
                    target_world   = target_world,
                    achieved_world = achieved_world,
                    R_achieved     = R_achieved,
                    R_target_world = R_target_world,
                    q_start        = q0,
                    q_final        = q_final,
                    steps_done     = steps_done,
                    n_steps        = n_steps,
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
        if self.original_joint_positions is None:
            return
        _apply_joint_positions(
            self._robot_obj, self._joint_objs, self.original_joint_positions,
            self._doc, transaction_label='IK Tool – restore',
        )
        self._lbl_status.setText(tr('↩ Original joint positions restored.'))
        self._refresh_ef_pose()


# ─────────────────────────────────────────────────────────────────────────────
# FreeCAD Gui Command
# ─────────────────────────────────────────────────────────────────────────────

class _IKToolCommand:

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
                '  • Inspect the current EF pose in WORLD coordinates.\n'
                '  • Enter a Cartesian target (X, Y, Z mm + RPY deg).\n'
                '  • Solve IK with smooth incremental motion (SLERP).\n'
                '  • A coordinate marker (IK_EE_Marker) shows the\n'
                '    actual EF position in the 3D viewport.\n'
                '  • Apply the solution or restore the original pose.\n'
                '\n'
                'Requires: a Cross::Robot selected in the scene.\n'
                'Algorithm: Damped Least Squares (DLS), 6-DOF,\n'
                f'           {_N_STEPS_DEFAULT} SLERP-interpolated steps.\n'
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


fcgui.addCommand('IKTool', _IKToolCommand())