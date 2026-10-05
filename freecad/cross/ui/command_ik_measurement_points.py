"""
ik_measurement_points.py
========================
Helper script for visual measurement of the IK Tool end-effector position
AND orientation (RPY) in the FreeCAD viewport.

Creates reference points and orientation axes in the FreeCAD viewport:

    Position:
        - Origin_IK_Point  (green):   always at (0, 0, 0) — document origin
        - EF_IK_Point      (red):     current EF position in world frame

    Orientation axes (triad at EF, length = AXIS_LENGTH mm):
        - EF_X_IK_Point    (red):     tip of local X axis
        - EF_Y_IK_Point    (green):   tip of local Y axis
        - EF_Z_IK_Point    (blue):    tip of local Z axis
        - EF_X_IK_Edge     (red):     line segment along local X axis
        - EF_Y_IK_Edge     (green):   line segment along local Y axis
        - EF_Z_IK_Edge     (blue):    line segment along local Z axis

Measurement workflow:
    Part → Measure Linear:
        • Origin_IK_Point  →  EF_IK_Point     = EF world position
        • EF_IK_Point      →  EF_X_IK_Point   = X axis direction (AXIS_LENGTH mm if aligned)
        • EF_IK_Point      →  EF_Y_IK_Point   = Y axis direction
        • EF_IK_Point      →  EF_Z_IK_Point   = Z axis direction

Usage (FreeCAD Python console):
    IK_MEASURE = '/home/alejandro/robotcad_clean/freecad/cross/ui/command_ik_measurement_points.py'
    exec(open(IK_MEASURE).read())

Or with a specific end-effector label:
    EE_LABEL = 'l_palm003'
    exec(open(IK_MEASURE).read())
"""

import math
import importlib

import numpy as np
import Part
import FreeCAD as fc

# ── imports from RobotCAD/RobotTool ──────────────────────────────────────────
import freecad.cross.kinematics.kinematics as _fk_mod
import freecad.cross.ui.command_ik_tool as _ik_cmd

importlib.reload(_fk_mod)
importlib.reload(_ik_cmd)

from freecad.cross.kinematics.kinematics import compute_forward_kinematics_full
from freecad.cross.ui.command_ik_tool import (
    _build_parent_map,
    _get_chain_to_root,
    _build_robot_for_chain,
    _get_robot_global_transform,
    _read_joint_positions,
)

# ── configuration ─────────────────────────────────────────────────────────────
if 'EE_LABEL' not in dir():
    EE_LABEL = 'l_palm003'

# Length of each orientation axis arrow in mm
AXIS_LENGTH = 20.0

# Point/line names
ORIGIN_NAME = 'Origin_IK_Point'
EF_NAME     = 'EF_IK_Point'
EF_X_PT     = 'EF_X_IK_Point'
EF_Y_PT     = 'EF_Y_IK_Point'
EF_Z_PT     = 'EF_Z_IK_Point'
EF_X_EDGE   = 'EF_X_IK_Edge'
EF_Y_EDGE   = 'EF_Y_IK_Edge'
EF_Z_EDGE   = 'EF_Z_IK_Edge'

ALL_NAMES = [
    ORIGIN_NAME,
    EF_NAME,
    EF_X_PT, EF_Y_PT, EF_Z_PT,
    EF_X_EDGE, EF_Y_EDGE, EF_Z_EDGE,
]

# ── helpers ───────────────────────────────────────────────────────────────────
def rotation_matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """
    Extracts RPY angles from a rotation matrix using extrinsic XYZ convention (URDF standard).

    Parameters
    ----------
    R : np.ndarray
        3x3 rotation matrix.

    Returns
    -------
    tuple[float, float, float]
        (roll, pitch, yaw) in radians.
    """
    pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0]**2 + R[1, 0]**2))
    if abs(pitch) > math.pi / 2 - 1e-6:   # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw  = 0.0
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw  = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def _add_vertex(doc, name: str, pos: np.ndarray,
                color: tuple, size: float = 15.0) -> None:
    """Adds a colored vertex (point) to the FreeCAD document."""
    obj = doc.addObject('Part::Feature', name)
    obj.Shape = Part.Vertex(fc.Vector(float(pos[0]), float(pos[1]), float(pos[2])))
    obj.Label = name
    if hasattr(obj, 'ViewObject'):
        obj.ViewObject.PointSize  = size
        obj.ViewObject.PointColor = color


def _add_edge(doc, name: str,
            p0: np.ndarray, p1: np.ndarray,
            color: tuple, width: float = 3.0) -> None:
    """Adds a colored line edge between two points to the FreeCAD document."""
    v0 = fc.Vector(float(p0[0]), float(p0[1]), float(p0[2]))
    v1 = fc.Vector(float(p1[0]), float(p1[1]), float(p1[2]))
    obj = doc.addObject('Part::Feature', name)
    obj.Shape = Part.makeLine(v0, v1)
    obj.Label = name
    if hasattr(obj, 'ViewObject'):
        obj.ViewObject.LineColor  = color
        obj.ViewObject.LineWidth  = width


# ── setup ─────────────────────────────────────────────────────────────────────
doc = fc.activeDocument()
if doc is None:
    raise RuntimeError("No active FreeCAD document.")

robot_obj = next(
    (o for o in doc.Objects
        if hasattr(o, 'Proxy') and 'Robot' in type(o.Proxy).__name__),
    None,
)
if robot_obj is None:
    raise RuntimeError("No Cross::Robot object found in the document.")

child_to_parent, child_to_joint = _build_parent_map(robot_obj)
chain = _get_chain_to_root(
    robot_obj, EE_LABEL, child_to_parent, child_to_joint
)
if not chain:
    raise RuntimeError(f"No kinematic chain found to '{EE_LABEL}'.")

robot_model = _build_robot_for_chain(robot_obj, chain)
T_world     = _get_robot_global_transform(robot_obj)

# ── compute current EF pose ───────────────────────────────────────────────────
q      = _read_joint_positions(chain, robot_obj)
all_T  = compute_forward_kinematics_full(robot_model, q)
T_ef   = T_world @ all_T[-1]
pos    = T_ef[:3, 3]
R_ef   = T_ef[:3, :3]

# ── cross-check position against FreeCAD placement ───────────────────────────
link_ef = robot_obj.getObject(EE_LABEL)
use_fc_gt = False

if link_ef is not None:
    pos_fc = link_ef.Placement.Base
    diff = math.sqrt(
        (pos[0] - pos_fc.x) ** 2 +
        (pos[1] - pos_fc.y) ** 2 +
        (pos[2] - pos_fc.z) ** 2
    )
    if diff > 0.1:
        print(f"[WARNING] FK vs FreeCAD position diff = {diff:.4f} mm")
        print(f"  Solver : ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        print(f"  FreeCAD: ({pos_fc.x:.3f}, {pos_fc.y:.3f}, {pos_fc.z:.3f})")
        print(f"  → Falling back to FreeCAD placement as ground truth.")
        use_fc_gt = True

        # Position ground truth
        pos = np.array([pos_fc.x, pos_fc.y, pos_fc.z])

        # Orientation ground truth from FreeCAD placement matrix
        M    = link_ef.Placement.toMatrix()
        R_ef = np.array([
            [M.A11, M.A12, M.A13],
            [M.A21, M.A22, M.A23],
            [M.A31, M.A32, M.A33],
        ])
    else:
        print(f"[OK] FK matches FreeCAD placement (diff={diff:.4f} mm)")

# ── extract RPY ───────────────────────────────────────────────────────────────
roll, pitch, yaw       = rotation_matrix_to_rpy(R_ef)
roll_d, pitch_d, yaw_d = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# ── compute axis tip points ───────────────────────────────────────────────────
tip_x = pos + R_ef[:, 0] * AXIS_LENGTH   # local X  →  red
tip_y = pos + R_ef[:, 1] * AXIS_LENGTH   # local Y  →  green
tip_z = pos + R_ef[:, 2] * AXIS_LENGTH   # local Z  →  blue

# ── remove old objects ────────────────────────────────────────────────────────
for name in ALL_NAMES:
    existing = doc.getObject(name)
    if existing is not None:
        doc.removeObject(existing.Name)

# ── create origin point (green) ───────────────────────────────────────────────
_add_vertex(doc, ORIGIN_NAME, np.zeros(3), color=(0.0, 1.0, 0.0))

# ── create EF position point (red) ───────────────────────────────────────────
_add_vertex(doc, EF_NAME, pos, color=(1.0, 0.0, 0.0))

# ── create orientation axis edges (lines) ─────────────────────────────────────
_add_edge(doc, EF_X_EDGE, pos, tip_x, color=(1.0, 0.0, 0.0), width=4.0)   # red
_add_edge(doc, EF_Y_EDGE, pos, tip_y, color=(0.0, 1.0, 0.0), width=4.0)   # green
_add_edge(doc, EF_Z_EDGE, pos, tip_z, color=(0.0, 0.4, 1.0), width=4.0)   # blue

# ── create axis tip points (measurable with Measure Linear) ──────────────────
_add_vertex(doc, EF_X_PT, tip_x, color=(1.0, 0.2, 0.2), size=10.0)   # red
_add_vertex(doc, EF_Y_PT, tip_y, color=(0.2, 1.0, 0.2), size=10.0)   # green
_add_vertex(doc, EF_Z_PT, tip_z, color=(0.2, 0.6, 1.0), size=10.0)   # blue

doc.recompute()

# ── summary ───────────────────────────────────────────────────────────────────
gt_tag = ' [FreeCAD GT]' if use_fc_gt else ' [Solver FK]'
print()
print("=" * 62)
print(f"IK Measurement Points — EF: {EE_LABEL}{gt_tag}")
print("=" * 62)

print()
print("── POSITION ────────────────────────────────────────────────")
print(f"  🟢 {ORIGIN_NAME:<22s}  X={0:>8.3f}  Y={0:>8.3f}  Z={0:>8.3f} mm")
print(f"  🔴 {EF_NAME:<22s}  X={pos[0]:>8.3f}  Y={pos[1]:>8.3f}  Z={pos[2]:>8.3f} mm")

print()
print("── ORIENTATION (RPY extrinsic XYZ / URDF convention) ───────")
print(f"  Roll  (X): {roll_d:>+9.3f}°   ({roll:>+.4f} rad)")
print(f"  Pitch (Y): {pitch_d:>+9.3f}°   ({pitch:>+.4f} rad)")
print(f"  Yaw   (Z): {yaw_d:>+9.3f}°   ({yaw:>+.4f} rad)")

print()
print("── ORIENTATION AXES (triad, length={:.0f} mm) ────────────────".format(AXIS_LENGTH))
print(f"  🔴 {EF_X_PT:<22s}  X={tip_x[0]:>8.3f}  Y={tip_x[1]:>8.3f}  Z={tip_x[2]:>8.3f} mm")
print(f"  🟢 {EF_Y_PT:<22s}  X={tip_y[0]:>8.3f}  Y={tip_y[1]:>8.3f}  Z={tip_y[2]:>8.3f} mm")
print(f"  🔵 {EF_Z_PT:<22s}  X={tip_z[0]:>8.3f}  Y={tip_z[1]:>8.3f}  Z={tip_z[2]:>8.3f} mm")

print()
print("── JOINTS (current) ─────────────────────────────────────────")
jvars = robot_obj.Proxy.joint_variables
for j, prop in jvars.items():
    deg = getattr(robot_obj, prop)
    rad = j.Position
    print(f"  {j.Label:<42s}  {deg:>+8.3f}°  ({rad:>+.4f} rad)")

print()
r = math.sqrt(float(pos[0])**2 + float(pos[1])**2)
print(f"  Radial distance from Z axis: {r:.3f} mm (nominal: 48.000 mm)")

print()
print("── MEASUREMENT GUIDE ────────────────────────────────────────")
print("  Part → Measure Linear:")
print(f"    Origin_IK_Point → EF_IK_Point       = EF world position")
print(f"    EF_IK_Point     → EF_X_IK_Point     = local X axis ({AXIS_LENGTH:.0f} mm if zero roll/pitch/yaw)")
print(f"    EF_IK_Point     → EF_Y_IK_Point     = local Y axis")
print(f"    EF_IK_Point     → EF_Z_IK_Point     = local Z axis")
print("=" * 62)