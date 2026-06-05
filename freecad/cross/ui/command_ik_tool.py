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
├── End Effector selector  - ComboBox listing all leaf links (tips) of the
│                            robot's kinematic tree.  Selecting one isolates
│                            the serial sub-chain from that link to the root,
│                            ignoring all other branches (wheels, legs, etc.).
├── Target position group  - three QDoubleSpinBox fields (X, Y, Z in mm)
│                            pre-filled with the current FK end-effector position
├── Target orientation group - three QDoubleSpinBox fields (Roll, Pitch, Yaw in deg)
│                              defaults to zero (identity orientation)
├── [Solve IK]    - runs RoboTool iterative Jacobian pseudo-inverse solver
│                   on the ISOLATED sub-chain only (6-DOF: position + orientation)
├── Results table - one row per joint in the sub-chain:
│     columns: Joint name | Original (rad/mm) | Solved (rad/mm)
├── Status label  - convergence message + achieved position + errors
│                   (position error in mm, orientation error in deg)
├── [Apply to Model]   - writes q_solved → joint_obj.Position,
│                        robot_obj.touch() + doc.recompute(True) for live
│                        3-D animation of the FULL robot hierarchy
├── [Restore Original] - writes q_original back → restores initial pose
└── [Close]

IK algorithm
~~~~~~~~~~~~
6-DOF Damped Least Squares (DLS / Levenberg-Marquardt):

    full_error = [w_pos * Δp,  w_ori * Δω]      shape (6,)
    J          = compute_jacobian(robot, q)       shape (6, N)
    Δq         = Jᵀ (J Jᵀ + λ²I₆)⁻¹ · full_error

    Δp  = target_position  - current_position     (mm)
    Δω  = rotation_error(R_current, R_target)     (rad, axial vector)
        = 0.5 · skew_vex(R_currentᵀ · R_target)

When target_rotation is None the solver falls back to 3-DOF position-only
mode (original behaviour, fully backward-compatible).

State management
~~~~~~~~~~~~~~~~
original_joint_positions is captured once (on the first Solve click) so the
user can always return to the pose that existed before the dialog was opened.
Subsequent Solve calls do NOT overwrite original_joint_positions.

Sub-chain isolation  (KEY FIX for tree-structured robots)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
When a robot has a tree topology (e.g. a manipulator arm PLUS drive wheels),
passing all joints to the IK solver produces a mathematically ill-defined
Jacobian and the solver diverges.

_get_chain_to_root(robot_obj, end_effector_link_name) walks the parent→child
map backwards from the chosen end-effector link to the root link, collecting
only the joints that belong to that unique serial path.  Joints in sibling
branches (wheels, suspension, other arms) are completely excluded.

The returned joint list is topologically ordered root→EE, guaranteeing that
the Jacobian matrix is computed over a clean, full-rank serial chain.

recompute fix  (KEY FIX for "nothing happens on Apply")
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Changing joint_obj.Position marks only the joint sub-object as dirty.
The parent Cross::Robot object does NOT automatically learn that its
children changed, so a plain doc.recompute() is a no-op.

Solution:
    robot_obj.touch()        # mark the robot container as dirty
    doc.recompute(True)      # deep/forced recompute of the whole hierarchy

This forces FreeCAD to re-evaluate every Link's placement transform and
redraws the meshes in the 3-D viewport immediately.

Integration with RobotCAD
~~~~~~~~~~~~~~~~~~~~~~~~~
- Reads the robot via  robot_obj.Proxy.get_joints() / get_links()
    (standard RobotCAD Proxy API).
- joint_obj.Position  - float in radians (revolute/continuous) or
    millimetres (prismatic).  Read and written directly.
- Units: all Cartesian coordinates in millimetres; joint angles in radians.
- doc.openTransaction / commitTransaction wraps every write so the action
    is undoable with Ctrl-Z.
- robot_obj.touch() + doc.recompute(True) + fcgui.updateGui() after every
    write triggers the live animation in the 3-D viewport.

RoboTool modules used
~~~~~~~~~~~~~~~~~~~~~
from ..kinematics.models             import Robot, Joint, Link
from ..kinematics.kinematics         import forward_kinematics
from ..kinematics.inverse_kinematics import inverse_kinematics

These must live under  freecad/cross/kinematics/  (i.e. your RoboTool
engine packaged inside RobotCAD).

Dialog layout
~~~~~~~~~~~~~
┌─ IK Tool ──────────────────────────────────────────────┐
│  End Effector: [tip_link ▼]                            │
│                                                        │
│  Target position (mm)                                  │
│  X: [______]  Y: [______]  Z: [______]                 │
│                                                        │
│  Target orientation (deg)                              │
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
│  Target:   [x, y, z] mm                                │
│  Achieved: [x, y, z] mm                                │
│  Error:    0.000001 mm                                 │
│  Error orientation: 0.42 deg                           │
│                                                        │
│  [ ▶ Apply to Model ]  [ ↩ Restore Original ]  [Close] │
└────────────────────────────────────────────────────────┘

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
from collections import defaultdict

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
# Kinematic-tree helpers
# ─────────────────────────────────────────────────────────────────────────────

def _build_parent_map(
    robot_obj: fc.App.DocumentObject,
) -> tuple[dict[str, str], dict[str, fc.App.DocumentObject]]:
    """
    Build a child→parent link map and a name→joint_obj lookup for the robot.

    Parameters
    ----------
    robot_obj : fc.App.DocumentObject
        The Cross::Robot document object.

    Returns
    -------
    tuple[dict[str, str], dict[str, fc.App.DocumentObject]]
        child_to_parent  : maps each child link name to its parent link name.
        child_to_joint   : maps each child link name to the joint_obj that
        connects it to its parent.
    """
    child_to_parent: dict[str, str] = {}
    child_to_joint:  dict[str, fc.App.DocumentObject] = {}

    for joint_obj in robot_obj.Proxy.get_joints():
        parent_link = joint_obj.Parent  # str: parent link name
        child_link  = joint_obj.Child   # str: child link name
        child_to_parent[child_link] = parent_link
        child_to_joint[child_link]  = joint_obj

    return child_to_parent, child_to_joint


def _find_leaf_links(robot_obj: fc.App.DocumentObject) -> list[str]:
    """
    Return all leaf (end-effector candidate) link names in the robot tree.

    A leaf link is one that never appears as a *parent* of any joint.
    For a serial arm this is just the tip link; for a tree robot (arm +
    wheels) this returns all branch tips, letting the user pick the right one.

    Parameters
    ----------
    robot_obj : fc.App.DocumentObject
        The Cross::Robot document object.

    Returns
    -------
    list[str]
        Sorted list of leaf link label strings.
    """
    all_links:    set[str] = {lnk.Label for lnk in robot_obj.Proxy.get_links()}
    parent_links: set[str] = set()

    for joint_obj in robot_obj.Proxy.get_joints():
        parent_links.add(joint_obj.Parent)

    leaf_links = sorted(all_links - parent_links)
    # Fallback: if every link is also a parent (unusual), return all links.
    return leaf_links if leaf_links else sorted(all_links)


def _get_chain_to_root(
    robot_obj:            fc.App.DocumentObject,
    end_effector_link:    str,
    child_to_parent:      dict[str, str],
    child_to_joint:       dict[str, fc.App.DocumentObject],
) -> list[fc.App.DocumentObject]:
    """
    Walk backwards from *end_effector_link* to the root, collecting joints.

    This isolates the unique serial sub-chain for a chosen end-effector and
    guarantees that sibling branches (wheels, other arms) are excluded.
    The returned list is ordered root→EE (topological order), which is the
    order required by the RoboTool FK/IK solvers.

    Only non-fixed joints are included (fixed joints carry no DOF).

    Parameters
    ----------
    robot_obj : fc.App.DocumentObject
        The Cross::Robot document object (used only for its label in errors).
    end_effector_link : str
        Label of the chosen end-effector (leaf) link.
    child_to_parent : dict[str, str]
        Maps child link label → parent link label (from _build_parent_map).
    child_to_joint : dict[str, fc.App.DocumentObject]
        Maps child link label → the joint_obj that produced it.

    Returns
    -------
    list[fc.App.DocumentObject]
        Non-fixed joint document objects in root→EE order.

    Raises
    ------
    ValueError
        If end_effector_link is not found in the kinematic tree.
    """
    if end_effector_link not in child_to_parent:
        # The root link has no parent joint; if someone selected it, return [].
        return []

    # Walk child → parent, accumulating joints, then reverse for root→EE order.
    chain_joints: list[fc.App.DocumentObject] = []
    current_link = end_effector_link

    while current_link in child_to_parent:
        joint_obj = child_to_joint[current_link]
        if joint_obj.Type != 'fixed':
            chain_joints.append(joint_obj)
        current_link = child_to_parent[current_link]

    chain_joints.reverse()  # now root → EE
    return chain_joints


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
    rpy = (math.radians(roll), math.radians(pitch), math.radians(yaw))
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


def _build_robot_for_chain(
    robot_obj:   fc.App.DocumentObject,
    chain_joints: list[fc.App.DocumentObject],
) -> Robot:
    """
    Build a RoboTool Robot instance that represents only the given serial chain.

    Links are gathered as: root link + one child link per joint, in order.
    This guarantees the RoboTool model is a clean linear chain with no
    dangling branches, which is a prerequisite for a well-formed Jacobian.

    Parameters
    ----------
    robot_obj : fc.App.DocumentObject
        The Cross::Robot document object (used for the robot name).
    chain_joints : list[fc.App.DocumentObject]
        Non-fixed joints in root→EE topological order (from _get_chain_to_root).

    Returns
    -------
    Robot
        Populated RoboTool Robot model for the isolated sub-chain.
    """
    robot_instance = Robot(name=robot_obj.Label)

    if not chain_joints:
        return robot_instance

    # Add the root link (parent of the first joint in the chain).
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
        # Each joint contributes its child link.
        robot_instance.links.append(Link(name=joint_obj.Child))

    return robot_instance


def _apply_joint_positions(
    robot_obj:     fc.App.DocumentObject,
    joint_objects: list[fc.App.DocumentObject],
    joint_values:  np.ndarray,
    doc:           fc.App.Document,
    transaction_label: str = 'IK Tool',
) -> None:
    """
    Write joint values to the document and trigger a live 3D viewport update.

    Architecture note — why we write robot._deg properties, not joint.Position
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    joint_obj.Position is a read-only computed property
    (setEditorMode('Position', ['ReadOnly']) in joint_proxy.py).  It is
    calculated by FreeCAD during recompute from the robot-level actuator
    property (e.g. robot.Joint_base_eslabon1_deg).

    The correct write path, discovered by reading robot_proxy.py, is:

        1. Look up the actuator property name via
            robot_obj.Proxy.joint_variables[joint_obj]
            e.g. returns 'Joint_base_eslabon1_deg'.
        2. Convert the solver value (radians / mm) to the property native unit:
            revolute / continuous  ->  degrees  (property stores degrees)
            prismatic              ->  mm        (property stores mm)
        3. setattr(robot_obj, prop_name, converted_value)
        4. doc.recompute() — FreeCAD propagates deg->rad into
            joint.Position and redraws the 3-D viewport.

    No robot_obj.touch() is needed: writing the actuator property already
    marks the robot dirty.  doc.recompute(True) must NOT be used —
    FreeCAD 1.0 expects a sequence of objects or no argument.

    Parameters
    ----------
    robot_obj : fc.App.DocumentObject
        The Cross::Robot container object.
    joint_objects : list[fc.App.DocumentObject]
        Non-fixed Cross::Joint objects to update (sub-chain only).
    joint_values : np.ndarray
        Joint positions in solver units: radians for revolute/continuous,
        millimetres for prismatic.
    doc : fc.App.Document
        The active FreeCAD document.
    transaction_label : str, optional
        Label shown in the undo history (default: 'IK Tool').
    """
    joint_variables: dict = robot_obj.Proxy.joint_variables  # joint_obj -> prop_name

    doc.openTransaction(tr(transaction_label))
    for joint_obj, value in zip(joint_objects, joint_values):
        prop_name = joint_variables.get(joint_obj)
        if prop_name is None:
            # Joint not found in robot's variable map — skip silently.
            continue
        if joint_obj.Type == 'prismatic':
            # Solver value is in mm; property stores mm.
            native_value = float(value)
        else:
            # revolute / continuous: solver value is in radians;
            # property stores degrees.
            native_value = math.degrees(float(value))
        setattr(robot_obj, prop_name, native_value)
    doc.commitTransaction()

    doc.recompute()     # propagates _deg -> Position and redraws viewport
    fcgui.updateGui()


# ─────────────────────────────────────────────────────────────────────────────
# Qt Dialog
# ─────────────────────────────────────────────────────────────────────────────

class IKToolDialog(QtWidgets.QDialog):
    """Modal dialog for the IK Tool command — see module docstring for layout."""

    def __init__(
        self,
        robot_obj: fc.App.DocumentObject,
        parent:    QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle(tr('IK Tool'))
        self.setMinimumWidth(520)

        # Remove the "?" help button; enum path differs between PySide2 and PySide6.
        _no_help = (
            getattr(QtCore.Qt.WindowType, 'WindowContextHelpButtonHint', None)
            or getattr(QtCore.Qt, 'WindowContextHelpButtonHint', 0)
        )
        self.setWindowFlags(self.windowFlags() & ~_no_help)

        self._doc          = fc.activeDocument()
        self._robot_obj    = robot_obj
        self._robot        = None   # RoboTool Robot for the ACTIVE sub-chain
        self._joint_objs   = []     # joint objects in the active sub-chain
        self.original_joint_positions = None   # captured on first Solve; never overwritten
        self.solved_joint_positions   = None

        # Kinematic tree maps — built once, reused when EE selection changes.
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

        # ── End Effector selector ──────────────────────────────
        grp_ee = QtWidgets.QGroupBox(tr('End Effector'))
        lay_ee = QtWidgets.QHBoxLayout(grp_ee)
        lay_ee.addWidget(QtWidgets.QLabel(tr('Select tip link:')))
        self._combo_ee = QtWidgets.QComboBox()
        self._combo_ee.setMinimumWidth(200)
        self._combo_ee.currentIndexChanged.connect(self._on_ee_changed)
        lay_ee.addWidget(self._combo_ee)
        lay_ee.addStretch()
        root_layout.addWidget(grp_ee)

        # ── Target position input group ────────────────────────
        grp_target = QtWidgets.QGroupBox(tr('Target position (mm)'))
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

        # ── Target orientation RPY ─────────────────────────────
        grp_rotation = QtWidgets.QGroupBox(tr('Target orientation (deg)'))
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

        # ── Solve button ───────────────────────────────────────
        self._btn_solve = QtWidgets.QPushButton(tr('⚙  Solve IK'))
        self._btn_solve.setFixedHeight(34)
        self._btn_solve.clicked.connect(self._on_solve)
        root_layout.addWidget(self._btn_solve)

        # ── Results table ──────────────────────────────────────
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

        # ── Status label ───────────────────────────────────────
        self._lbl_status = QtWidgets.QLabel(
            tr('Ready. Select an End Effector, enter a target and press Solve IK.')
        )
        self._lbl_status.setWordWrap(True)
        root_layout.addWidget(self._lbl_status)

        # ── Action buttons ─────────────────────────────────────
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
        """Build child→parent and child→joint maps from the robot document object."""
        try:
            self._child_to_parent, self._child_to_joint = _build_parent_map(
                self._robot_obj
            )
        except Exception as exc:
            self._lbl_status.setText(f'Error reading robot tree: {exc}')
            self._btn_solve.setEnabled(False)

    def _populate_ee_combo(self) -> None:
        """Fill the End Effector ComboBox with all leaf links in the robot tree."""
        leaf_links = _find_leaf_links(self._robot_obj)
        self._combo_ee.blockSignals(True)
        self._combo_ee.clear()
        for link_label in leaf_links:
            self._combo_ee.addItem(link_label)
        self._combo_ee.blockSignals(False)

        # Trigger load for the default (first) selection.
        if leaf_links:
            self._load_chain(leaf_links[0])

    # ── End Effector change ───────────────────────────────────

    def _on_ee_changed(self, index: int) -> None:
        """
        Called when the user picks a different end-effector link.

        Resets the solved state, re-isolates the sub-chain, rebuilds the
        RoboTool model, and pre-fills the FK spinboxes for the new chain.
        """
        ee_label = self._combo_ee.currentText()
        if not ee_label:
            return

        # Reset solved state — the previous solution is for a different chain.
        self.original_joint_positions = None
        self.solved_joint_positions   = None
        self._btn_apply.setEnabled(False)
        self._btn_restore.setEnabled(False)

        self._load_chain(ee_label)

    def _load_chain(self, end_effector_label: str) -> None:
        """
        Isolate the serial sub-chain for *end_effector_label* and rebuild
        the RoboTool model and UI table for that chain.

        Parameters
        ----------
        end_effector_label : str
            Label of the selected end-effector (leaf) link.
        """
        # ── 1. Isolate the sub-chain (topological walk-back) ──
        try:
            chain_joints = _get_chain_to_root(
                self._robot_obj,
                end_effector_label,
                self._child_to_parent,
                self._child_to_joint,
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

        # ── 2. Build RoboTool model for the isolated chain ────
        try:
            self._robot = _build_robot_for_chain(self._robot_obj, chain_joints)
        except Exception as exc:
            self._lbl_status.setText(f'Error building robot model: {exc}')
            self._btn_solve.setEnabled(False)
            return

        self._joint_objs = chain_joints
        n_joints = len(chain_joints)

        # ── 3. Pre-fill spinboxes with current FK position ────
        q0 = np.array([float(j.Position) for j in self._joint_objs])
        try:
            fk_matrix   = forward_kinematics(self._robot, q0)
            fk_position = fk_matrix[:3, 3]
            for axis, value in zip(('X', 'Y', 'Z'), fk_position):
                self._spins[axis].setValue(float(value))
        except Exception:
            pass  # Spinboxes stay at 0 — not a fatal error.

        # ── 4. Populate results table with current joint values ──
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

    # ── Slot: Solve IK ────────────────────────────────────────

    def _on_solve(self) -> None:
        """Run the 6-DOF IK solver on the active sub-chain and display results."""
        if not self._joint_objs or self._robot is None:
            return

        try:
            # ── Build R_target from RPY spinboxes ──────────────
            roll  = np.deg2rad(self._spins['Roll'].value())
            pitch = np.deg2rad(self._spins['Pitch'].value())
            yaw   = np.deg2rad(self._spins['Yaw'].value())

            Rotation_x = np.array([
                [1, 0,           0          ],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll),  np.cos(roll)],
            ])
            Rotation_y = np.array([
                [ np.cos(pitch), 0, np.sin(pitch)],
                [ 0,             1, 0            ],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ])
            Rotation_z = np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw),  np.cos(yaw), 0],
                [0,            0,           1],
            ])
            Rotation_target = Rotation_z @ Rotation_y @ Rotation_x

            # ── Read Cartesian target ──────────────────────────
            target = np.array([self._spins[axis].value() for axis in ('X', 'Y', 'Z')])
            q0     = np.array([float(j.Position) for j in self._joint_objs])

            # Capture original pose on the first solve; do NOT overwrite on re-solve.
            if self.original_joint_positions is None:
                self.original_joint_positions = q0.copy()
                self._btn_restore.setEnabled(True)

            # ── Run solver (capture stdout for status label) ───
            log_buffer = io.StringIO()
            with redirect_stdout(log_buffer):
                q_solved = inverse_kinematics(
                    self._robot,
                    target_position=target,
                    initial_guess=q0,
                    target_rotation=Rotation_target,
                )
            solver_log = log_buffer.getvalue().strip()

            if q_solved is None:
                for i in range(len(self._joint_objs)):
                    self._table.setItem(
                        i, 2, QtWidgets.QTableWidgetItem(tr('Unreachable'))
                    )
                self._lbl_status.setText(
                    tr('IK did not converge. Target may be outside the workspace.')
                    + (f'\n{solver_log}' if solver_log else '')
                )
                self._btn_apply.setEnabled(False)
                return

            self.solved_joint_positions = q_solved

            # ── Update results table ───────────────────────────
            for i in range(len(self._joint_objs)):
                self._table.setItem(
                    i, 1, QtWidgets.QTableWidgetItem(f'{self.original_joint_positions[i]:.4f}')
                )
                self._table.setItem(
                    i, 2, QtWidgets.QTableWidgetItem(f'{q_solved[i]:.4f}')
                )
            self._table.resizeColumnsToContents()

            # ── FK verification + residual errors ─────────────
            try:
                fk_matrix      = forward_kinematics(self._robot, q_solved)
                achieved       = fk_matrix[:3, 3]
                residual_error = float(np.linalg.norm(target - achieved))
                cos_angle      = np.clip(
                    (np.trace(fk_matrix[:3, :3].T @ Rotation_target) - 1) / 2,
                    -1.0, 1.0
                )
                status_details = (
                    f'Target:   [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}] mm\n'
                    f'Achieved: [{achieved[0]:.3f}, {achieved[1]:.3f}, {achieved[2]:.3f}] mm\n'
                    f'Error:    {residual_error:.6f} mm\n'
                    f'Error orientation: {np.rad2deg(np.arccos(cos_angle)):.2f} deg'
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
        """Write solved joint positions to the document and animate the model."""
        if self.solved_joint_positions is None:
            return
        _apply_joint_positions(
            self._robot_obj,
            self._joint_objs,
            self.solved_joint_positions,
            self._doc,
            transaction_label='IK Tool – apply',
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
            self._robot_obj,
            self._joint_objs,
            self.original_joint_positions,
            self._doc,
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
                '  • Choose the End Effector (tip link) for IK.\n'
                '  • Inspect the current end-effector position (FK).\n'
                '  • Enter a Cartesian target (X, Y, Z in mm) and\n'
                '    orientation (Roll, Pitch, Yaw in degrees).\n'
                '  • Solve 6-DOF IK via iterative Jacobian DLS\n'
                '    on the isolated serial sub-chain only.\n'
                '  • See solved joint angles in a results table.\n'
                '  • Apply the solution → live 3-D animation.\n'
                '  • Restore the original pose at any time.\n'
                '\n'
                'Requires: a Cross::Robot selected in the scene.\n'
                'Algorithm: Damped Least Squares (DLS), 6-DOF,\n'
                '           up to 1 000 iterations, tol = 1 × 10⁻⁴ mm.\n'
            ),
        }

    def IsActive(self) -> bool:
        """Return True only when a Cross::Robot object is selected."""
        selection = fcgui.Selection.getSelection()
        return bool(selection) and is_robot(selection[0])

    def Activated(self) -> None:
        """Open the IK Tool dialog for the selected robot (non-modal)."""
        robot_obj = fcgui.Selection.getSelection()[0]

        # Non-modal window: show() instead of exec_() so the user can
        # continue interacting with RobotCAD while the dialog is open.
        #
        # parent=None avoids the PySide2/6 type error with QMainWindow.
        # WA_DeleteOnClose=False + self._dialog reference on the command
        # instance prevent the SEGFAULT caused by Python GC destroying the
        # C++ Qt object while the window is still visible.
        dialog = IKToolDialog(robot_obj, parent=None)

        if _PYSIDE6:
            _wa_del = QtCore.Qt.WidgetAttribute.WA_DeleteOnClose
        else:
            _wa_del = QtCore.Qt.WA_DeleteOnClose
        dialog.setAttribute(_wa_del, False)

        # Keep a strong Python reference so GC never collects the dialog.
        self._dialog = dialog
        dialog.show()


# ─────────────────────────────────────────────────────────────────────────────
# Registration
# ─────────────────────────────────────────────────────────────────────────────

fcgui.addCommand('IKTool', _IKToolCommand())