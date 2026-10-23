"""
ik_validation.py
================
batch IK validation suite - three levels of orientation analysis.

Level 1 - Geodesic orientation error (mathematical ground truth)
    θ = arccos((trace(R_target^T @ R_achieved) - 1) / 2 )
    single number, convetion-independent, physically meaningful.

Level 2 - Per-axis RPY delta (develooper-friendly)
    Δroll, Δpitch, Δyaw between target and achieved.
    Inmediately tells you WHICH axis drifted.

Level 3 - Axis vector comparison (Euler - free orientation check)
    Compares X/Y/Z columns of R_target vs R_achieved.
    Angle between axis vectors, bypasses all Euler singularities.

Automatic angular sweep
    run_sweep() tests a single axis (roll/pitch/yaw) across its full
    range in configurable steps, producing a table suitable for plotting
    error vs angle to identify singularities boundaries and other issues.

Circular import note
--------------------
command_ik_tool is NOT imported at module level.
It is imported lazily inside run_validation() / run_sweep() only when
those functions are actually called, breaking the cycle:
    command_ik_tool → ik_validation → command_ik_tool

Usage (FreeCAD Python console):
    IK_VAL = '/home/alejandro/.local/share/FreeCAD/Mod/freecad.robotcad/freecad/cross/kinematics/ik_validation.py'
    exec(open(IK_VAL).read())

To add custom poses edit TEST_POSES.
To change thresholds edit PASS_THRESHOLD_POS / PASS_THRESHOLD_ORI.
"""

from __future__ import annotations
import math
import datetime
import importlib

import numpy as np
import FreeCAD as fc

# ── Pure-kinematics imports (no UI, no circular risk) ────────────────────────
from . import kinematics as _fk_mod
from . import inverse_kinematics as _ik_mod
from . import jacobians as _jac_mod

importlib.reload(_fk_mod)
importlib.reload(_ik_mod)
importlib.reload(_jac_mod)

from ..kinematics.kinematics import forward_kinematics
from ..kinematics.inverse_kinematics import inverse_kinematics
from ..kinematics.jacobians import compute_jacobian


# ─────────────────────────────────────────────────────────────────────────────
# _slerp_rotation — copied inline to avoid importing command_ik_tool at
# module level (_run_ik uses it at call time, not import time, but keeping
# it here avoids any future accidental top-level dependency).
# ─────────────────────────────────────────────────────────────────────────────

def _slerp_rotation(
    R_start: np.ndarray,
    R_target: np.ndarray,
    alpha: float,
) -> np.ndarray:
    """SLERP via Rodrigues. Copied from command_ik_tool to avoid circular import."""
    R_rel = R_start.T @ R_target
    theta = np.arccos(np.clip((np.trace(R_rel) - 1) / 2.0, -1.0, 1.0))
    if theta < 1e-9:
        return R_target.copy()
    if np.pi - theta < 1e-6:
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
    return R_start @ (np.eye(3) + np.sin(a) * K + (1.0 - np.cos(a)) * (K @ K))

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────

# Fallback EE label when running as standalone script (exec mode).
# When called from command_ik_tool, ee_label is always passed explicitly.
_DEFAULT_EE_LABEL = 'tip_link'

# Pass/fail thresholds
PASS_THRESHOLD_POS = 1.0    # mm
PASS_THRESHOLD_ORI = 1.0    # deg  (geodesic error, Level 1)

# Solver parameters — mirror the IK Tool defaults
N_STEPS    = 50
MAX_ITER   = 1000
ORI_WEIGHT = 0.3
POS_WEIGHT = 1.0
RESTARTS   = 5
TOLERANCE  = 0.001   # mm

def _generate_test_poses(
    robot_model,
    T_world:  np.ndarray,
    q_home:   np.ndarray,
    target_pose: tuple | None = None,
) -> list[tuple]:
    """
    Generate test poses dynamically from the active robot's geometry.

    Reads the actual home FK position and joint limits to produce poses
    that are always inside the reachable workspace of whatever robot is
    loaded — no hardcoded numbers.

    Strategy
    --------
    1. Home pose  — FK(q=0), the natural rest position.
    2. Working position — slightly retracted from home along Z to give
        room for orientation tests without hitting workspace boundaries.
    3. Pitch/Roll/Yaw sweeps at safe angles extracted from joint limits.
    4. Lateral offsets proportional to the home radial distance.
    5. Negative-Y mirror of home.
    6. Combined RPY at 30% of the most conservative single-axis limit.

    Returns
    -------
    List of (X, Y, Z, roll°, pitch°, yaw°, label) tuples.
    """
    # ── Home FK ───────────────────────────────────────────────────────────────
    T_home    = T_world @ forward_kinematics(robot_model, q_home)
    hx, hy, hz = T_home[:3, 3]
    hr, hp, hw  = _matrix_to_rpy(T_home[:3, :3])

    # Radial distance from Z axis — used for lateral offsets
    r_home = math.sqrt(hx**2 + hy**2)
    lateral_offset = max(20.0, r_home * 0.3)   # 30% of radius, min 20 mm

    # Working position: retract 10% along Z from home (stay in workspace)
    z_retract = hz - abs(hz) * 0.10
    # Clamp retraction to avoid going below a reasonable floor
    z_retract = max(z_retract, hz * 0.5)

    # ── Extract joint limits for orientation ranges ───────────────────────────
    active = [j for j in robot_model.joints if j.joint_type != 'fixed']

    def _safe_range(joint_idx: int, fallback_deg: float = 45.0) -> float:
        """Return 80% of the joint's range in degrees, or fallback if unconstrained."""
        if joint_idx >= len(active):
            return fallback_deg
        j = active[joint_idx]
        if j.limit_lower is None or j.limit_upper is None:
            return fallback_deg
        full_range_deg = math.degrees(j.limit_upper - j.limit_lower)
        # Use 40% of full range as a safe test angle (avoids limit clamping)
        return max(5.0, full_range_deg * 0.40)

    # Use last joints for orientation (they typically control orientation)
    n = len(active)
    roll_range  = _safe_range(n - 1, 45.0)
    pitch_range = _safe_range(n - 2, 45.0) if n >= 2 else 45.0
    yaw_range   = _safe_range(n - 3, 45.0) if n >= 3 else 45.0

    # Clamp to [-180, 180]
    roll_range  = min(roll_range,  180.0)
    pitch_range = min(pitch_range, 180.0)
    yaw_range   = min(yaw_range,   180.0)

    # Safe combined angle = 30% of most conservative limit
    combined = min(roll_range, pitch_range, yaw_range) * 0.30
    combined = max(5.0, combined)

    # Pitch steps — sample at 25%, 50%, 75%, ~90% of pitch range
    p25 = round(pitch_range * 0.25, 1)
    p50 = round(pitch_range * 0.50, 1)
    p75 = round(pitch_range * 0.75, 1)
    p90 = round(pitch_range * 0.90, 1)

    wx, wy = hx, hy   # working XY = same as home XY

    poses = [
        # ── Home ──────────────────────────────────────────────────────────────
        (hx,  hy,  hz,   hr,   hp,   hw,
        'Home (FK q=0)'),

        # ── Pitch sweep ───────────────────────────────────────────────────────
        (wx,  wy,  z_retract,  0,  p25,  0,
        f'Pitch +{p25}°  (25% range)'),
        (wx,  wy,  z_retract,  0,  p50,  0,
        f'Pitch +{p50}°  (50% range)'),
        (wx,  wy,  z_retract,  0,  p75,  0,
        f'Pitch +{p75}°  (75% range)'),
        (wx,  wy,  z_retract,  0,  p90,  0,
        f'Pitch +{p90}°  (90% — near limit)'),

        # ── Roll ──────────────────────────────────────────────────────────────
        (wx,  wy,  z_retract,  round(roll_range * 0.5, 1),  0,  0,
        f'Roll +{round(roll_range*0.5,1)}°  (50% range)'),

        # ── Yaw ───────────────────────────────────────────────────────────────
        (wx,  wy,  z_retract,  0,  0,  round(yaw_range * 0.5, 1),
        f'Yaw +{round(yaw_range*0.5,1)}°  (50% range)'),

        # ── Combined ──────────────────────────────────────────────────────────
        (wx,  wy,  z_retract,
        round(combined, 1), round(combined, 1), round(combined, 1),
        f'Combined RPY {round(combined,1)}°/{round(combined,1)}°/{round(combined,1)}°'),

        # ── Lateral offsets ───────────────────────────────────────────────────
        (hx + lateral_offset,  hy,  hz,  0,  0,  0,
        f'Lateral +X  ({lateral_offset:.0f} mm)'),
        (hx - lateral_offset,  hy,  hz,  0,  0,  0,
        f'Lateral -X  ({lateral_offset:.0f} mm)'),

        # ── Z retract (pure position, no rotation) ────────────────────────────
        (wx,  wy,  z_retract,  0,  0,  0,
        f'Z retract  (Z={z_retract:.1f} mm, identity ori)'),

        # ── Negative Y mirror ─────────────────────────────────────────────────
        (hx,  -abs(hy) if hy != 0 else -lateral_offset,  hz,  0,  0,  0,
        'Negative Y mirror'),
    ]

    # Round all numeric fields for clean display
    return [
        (round(float(x), 3), round(float(y), 3), round(float(z), 3),
        round(float(r), 3), round(float(p), 3), round(float(yw), 3),
        label)
        for x, y, z, r, p, yw, label in poses
    ]

# ─────────────────────────────────────────────────────────────────────────────
# Math helpers
# ─────────────────────────────────────────────────────────────────────────────

def _rpy_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Extrinsic XYZ RPY → rotation matrix (URDF convention). R = Rz @ Ry @ Rx."""
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    Rx = np.array([[1, 0,           0          ],
                    [0, math.cos(r), -math.sin(r)],
                    [0, math.sin(r),  math.cos(r)]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                    [ 0,           1, 0           ],
                    [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                    [math.sin(y),  math.cos(y), 0],
                    [0,            0,           1]])
    return Rz @ Ry @ Rx

def _matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """
    Rotation matrix → extrinsic XYZ RPY in degrees (URDF convention).
    Returns (roll, pitch, yaw).
    """
    pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0]**2 + R[1, 0]**2))
    if abs(pitch) > math.pi / 2 - 1e-6:   # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw  = 0.0
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw  = math.atan2(R[1, 0], R[0, 0])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def _geodesic_error_deg(R_target: np.ndarray, R_achieved: np.ndarray) -> float:
    """
    Level 1 — Geodesic angular distance between two rotation matrices.
    θ = arccos((trace(R_target^T @ R_achieved) - 1) / 2)
    Convention-independent, physically meaningful.
    """
    R_err  = R_target.T @ R_achieved
    cosval = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    return float(math.degrees(math.acos(cosval)))

def _rpy_delta(
    R_target: np.ndarray,
    R_achieved: np.ndarray,
) -> tuple[float, float, float]:
    """
    Level 2 — Per-axis RPY delta in degrees.
    Extracts RPY from both matrices and subtracts.
    Returns (Δroll, Δpitch, Δyaw).
    """
    r_t, p_t, y_t = _matrix_to_rpy(R_target)
    r_a, p_a, y_a = _matrix_to_rpy(R_achieved)

    def _wrap(d: float) -> float:
        """Wrap angle difference to [-180, 180]."""
        while d >  180.0: d -= 360.0
        while d < -180.0: d += 360.0
        return d

    return _wrap(r_a - r_t), _wrap(p_a - p_t), _wrap(y_a - y_t)

def _axis_errors(
    R_target: np.ndarray,
    R_achieved: np.ndarray,
) -> tuple[float, float, float]:
    """
    Level 3 — Per-axis vector angle errors in degrees.
    Compares columns of R_target vs R_achieved independently.
    Returns (err_x_deg, err_y_deg, err_z_deg).
    Each value is the angle between the target and achieved axis vector,
    completely independent of Euler convention.
    """
    def _angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
        cos_a = np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1)
        return float(math.degrees(math.acos(cos_a)))

    err_x = _angle_between(R_target[:, 0], R_achieved[:, 0])
    err_y = _angle_between(R_target[:, 1], R_achieved[:, 1])
    err_z = _angle_between(R_target[:, 2], R_achieved[:, 2])
    return err_x, err_y, err_z

def _check_joint_limits(robot_model, q: np.ndarray) -> list[str]:
    """Returns joint names at or beyond their configured limits."""
    violations = []
    active = [j for j in robot_model.joints if j.joint_type != 'fixed']
    for joint, qi in zip(active, q):
        lo, hi = joint.limit_lower, joint.limit_upper
        if lo is None or hi is None:
            continue
        if qi <= lo + 1e-3 or qi >= hi - 1e-3:
            violations.append(joint.name)
    return violations

def _manipulability(robot_model, q: np.ndarray) -> float:
    """Yoshikawa manipulability w = sqrt(det(Jv @ Jv^T)) for translation subspace."""
    J  = compute_jacobian(robot_model, q)
    Jv = J[:3, :]
    return float(np.sqrt(max(0.0, np.linalg.det(Jv @ Jv.T))))

def _run_ik(
    robot_model,
    T_world: np.ndarray,
    q0: np.ndarray,
    target_world: np.ndarray,
    R_target_world: np.ndarray,
) -> tuple[np.ndarray | None, int]:
    """Incremental SLERP IK from q0 to (target_world, R_target_world)."""
    R_world       = T_world[:3, :3]
    p_world       = T_world[:3, 3]
    target_local  = R_world.T @ (target_world - p_world)
    R_target_local = R_world.T @ R_target_world

    T_start    = forward_kinematics(robot_model, q0)
    pos_start  = T_start[:3, 3]
    R_start    = T_start[:3, :3]

    q_current  = q0.copy()
    q_final    = None
    steps_done = 0

    for step in range(N_STEPS):
        alpha      = (step + 1) / N_STEPS
        pos_interp = pos_start + alpha * (target_local - pos_start)
        R_interp   = _slerp_rotation(R_start, R_target_local, alpha)
        try:
            q_new = inverse_kinematics(
                robot_model,
                target_position    = pos_interp,
                initial_guess      = q_current,
                target_rotation    = R_interp,
                weight_position    = POS_WEIGHT,
                weight_orientation = ORI_WEIGHT,
                max_iterations     = MAX_ITER,
                tolerance          = TOLERANCE,
                max_restarts       = RESTARTS,
                verbose            = False,
                wrap_joints        = False,
            )
        except Exception:
            break
        if q_new is None:
            break
        q_current  = q_new
        q_final    = q_new
        steps_done += 1

    return q_final, steps_done

# ─────────────────────────────────────────────────────────────────────────────
# Result builder
# ─────────────────────────────────────────────────────────────────────────────

def _make_result(
    label:          str,
    target_pos:     np.ndarray,
    target_rpy_deg: tuple[float, float, float],
    achieved_pos:   np.ndarray | None,
    R_target:       np.ndarray,
    R_achieved:     np.ndarray | None,
    q_final:        np.ndarray | None,
    steps_done:     int,
    limit_viols:    list[str],
    manip:          float,
) -> dict:
    if q_final is None or R_achieved is None or achieved_pos is None:
        return dict(
            label=label, target_pos=target_pos, target_rpy_deg=target_rpy_deg,
            achieved_pos=None, achieved_rpy_deg=None,
            pos_error=float('inf'), ori_geodesic=float('inf'),
            d_roll=float('inf'), d_pitch=float('inf'), d_yaw=float('inf'),
            err_x_axis=float('inf'), err_y_axis=float('inf'), err_z_axis=float('inf'),
            steps_done=steps_done, reachable=False,
            limit_viols=[], manipulability=manip, passed=False,
        )

    pos_error    = float(np.linalg.norm(target_pos - achieved_pos))
    ori_geodesic = _geodesic_error_deg(R_target, R_achieved)
    d_roll, d_pitch, d_yaw = _rpy_delta(R_target, R_achieved)
    err_x, err_y, err_z    = _axis_errors(R_target, R_achieved)
    achieved_rpy            = _matrix_to_rpy(R_achieved)
    passed = (pos_error <= PASS_THRESHOLD_POS and ori_geodesic <= PASS_THRESHOLD_ORI)

    return dict(
        label=label, target_pos=target_pos, target_rpy_deg=target_rpy_deg,
        achieved_pos=achieved_pos, achieved_rpy_deg=achieved_rpy,
        pos_error=pos_error, ori_geodesic=ori_geodesic,
        d_roll=d_roll, d_pitch=d_pitch, d_yaw=d_yaw,
        err_x_axis=err_x, err_y_axis=err_y, err_z_axis=err_z,
        steps_done=steps_done, reachable=True,
        limit_viols=limit_viols, manipulability=manip, passed=passed,
    )

# ─────────────────────────────────────────────────────────────────────────────
# Report printer
# ─────────────────────────────────────────────────────────────────────────────

def _print_report(results: list[dict], ee_label: str) -> None:
    W    = 92
    sep  = '═' * W
    dash = '─' * W
    ts   = datetime.datetime.now().strftime('%Y-%m-%d  %H:%M:%S')

    passed = sum(1 for r in results if r['passed'])
    failed = len(results) - passed

    print()
    print(sep)
    print(f'  IK Validation Suite — EF: {ee_label}')
    print(f'  {ts}')
    print(f'  Thresholds:  pos ≤ {PASS_THRESHOLD_POS} mm  |  geodesic ori ≤ {PASS_THRESHOLD_ORI}°')
    print(sep)

    for r in results:
        tx, ty, tz    = r['target_pos']
        ro, pi, ya    = r['target_rpy_deg']
        status        = '✅ PASS' if r['passed'] else ('❌ UNREACHABLE' if not r['reachable'] else '❌ FAIL')

        print(f"\n  ┌─ {r['label']} {'─'*(W-6-len(r['label']))}")
        print(f"  │  Target    XYZ: ({tx:7.2f}, {ty:7.2f}, {tz:7.2f}) mm   "
            f"RPY: ({ro:7.2f}°, {pi:7.2f}°, {ya:7.2f}°)")

        if not r['reachable']:
            print(f"  │  {status}")
            print(f"  └{'─'*(W-3)}")
            continue

        ax, ay, az       = r['achieved_pos']
        ar, ap, aw       = r['achieved_rpy_deg']
        print(f"  │  Achieved  XYZ: ({ax:7.2f}, {ay:7.2f}, {az:7.2f}) mm   "
            f"RPY: ({ar:7.2f}°, {ap:7.2f}°, {aw:7.2f}°)")

        # Level 1 — geodesic
        print(f"  │")
        print(f"  │  ── Level 1: Geodesic orientation error ──────────────────")
        print(f"  │     θ = {r['ori_geodesic']:8.4f}°   |   Pos error = {r['pos_error']:8.4f} mm   |   {status}")

        # Level 2 — RPY delta
        print(f"  │")
        print(f"  │  ── Level 2: Per-axis RPY delta ──────────────────────────")
        print(f"  │     ΔRoll  = {r['d_roll']:+8.4f}°")
        print(f"  │     ΔPitch = {r['d_pitch']:+8.4f}°")
        print(f"  │     ΔYaw   = {r['d_yaw']:+8.4f}°")

        # Level 3 — axis vectors
        print(f"  │")
        print(f"  │  ── Level 3: Axis vector errors ───────────────────────────")
        print(f"  │     X-axis angle = {r['err_x_axis']:8.4f}°   "
            f"target={r['target_pos'].__class__.__name__}  "   # placeholder — axes printed below
            )
        # Print axis vectors for visual inspection
        if r['achieved_rpy_deg'] is not None:
            R_t = _rpy_to_matrix(*r['target_rpy_deg'])
            R_a = _rpy_to_matrix(*r['achieved_rpy_deg'])
            for idx, axis_name in enumerate(['X', 'Y', 'Z']):
                vt = R_t[:, idx]
                va = R_a[:, idx]
                err_field = [r['err_x_axis'], r['err_y_axis'], r['err_z_axis']][idx]
                print(f"  │     {axis_name}  target : ({vt[0]:+.4f}, {vt[1]:+.4f}, {vt[2]:+.4f})")
                print(f"  │       achieved: ({va[0]:+.4f}, {va[1]:+.4f}, {va[2]:+.4f})   "
                    f"angle = {err_field:.4f}°")

        # Extras
        print(f"  │")
        print(f"  │  Steps: {r['steps_done']}/{N_STEPS}   Manipulability: {r['manipulability']:.4f}"
            + (f"  ⚠ near singularity" if r['manipulability'] < 1e-3 else ""))
        if r['limit_viols']:
            print(f"  │  ⚠ Joint limits hit: {', '.join(r['limit_viols'])}")
        print(f"  └{'─'*(W-3)}")

    # Summary table
    print()
    print(dash)
    print(f'  {"Test":<28s}  {"PosErr":>7s}  {"GeoOri":>7s}  '
        f'{"ΔR":>7s}  {"ΔP":>7s}  {"ΔY":>7s}  {"Status"}')
    print(dash)
    for r in results:
        if not r['reachable']:
            print(f"  {r['label']:<28s}  {'∞':>7s}  {'∞':>7s}  "
                f"{'∞':>7s}  {'∞':>7s}  {'∞':>7s}  ❌ UNREACHABLE")
        else:
            print(f"  {r['label']:<28s}  {r['pos_error']:7.3f}  {r['ori_geodesic']:7.3f}  "
                f"{r['d_roll']:+7.3f}  {r['d_pitch']:+7.3f}  {r['d_yaw']:+7.3f}  "
                f"{'✅ PASS' if r['passed'] else '❌ FAIL'}")
    print(dash)
    print(f'  Results: {passed}/{len(results)} PASS  |  {failed}/{len(results)} FAIL')
    print()
    print('  Orientation metrics:')
    print('  · GeoOri = arccos((trace(R_t^T @ R_a)-1)/2)  — geodesic, convention-free')
    print('  · ΔR/ΔP/ΔY = per-axis RPY delta               — intuitive, Euler-dependent')
    print('  · Axis vector angles (Level 3)                — Euler-free per-axis check')
    print(sep)
    print()

# ─────────────────────────────────────────────────────────────────────────────
# Sweep report printer
# ─────────────────────────────────────────────────────────────────────────────

def _print_sweep_report(
    sweep_results: list[dict],
    sweep_axis:    str,
    ee_label:      str,
) -> None:
    W   = 80
    sep = '═' * W
    ts  = datetime.datetime.now().strftime('%Y-%m-%d  %H:%M:%S')

    print()
    print(sep)
    print(f'  IK Sweep — {sweep_axis} axis — EF: {ee_label}')
    print(f'  {ts}')
    print(sep)
    print(f'  {"Target°":>8s}  {"Achieved°":>9s}  {"Δ°":>7s}  '
        f'{"PosErr":>7s}  {"GeoOri":>7s}  {"w":>7s}  {"Steps":>5s}  Status')
    print('─' * W)

    for r in sweep_results:
        tgt = r['target_rpy_deg'][{'Roll': 0, 'Pitch': 1, 'Yaw': 2}[sweep_axis]]
        if not r['reachable']:
            print(f'  {tgt:>8.1f}  {"—":>9s}  {"—":>7s}  {"∞":>7s}  {"∞":>7s}  '
                f'{"—":>7s}  {"—":>5s}  ❌ UNREACHABLE')
            continue
        ach = r['achieved_rpy_deg'][{'Roll': 0, 'Pitch': 1, 'Yaw': 2}[sweep_axis]]
        delta_map = {'Roll': r['d_roll'], 'Pitch': r['d_pitch'], 'Yaw': r['d_yaw']}
        delta = delta_map[sweep_axis]
        status = '✅' if r['passed'] else '❌'
        print(f'  {tgt:>8.1f}  {ach:>9.3f}  {delta:>+7.3f}  '
            f'{r["pos_error"]:>7.3f}  {r["ori_geodesic"]:>7.3f}  '
            f'{r["manipulability"]:>7.4f}  {r["steps_done"]:>5d}  {status}')

    # ASCII chart — geodesic error vs target angle
    vals = [(r['target_rpy_deg'][{'Roll':0,'Pitch':1,'Yaw':2}[sweep_axis]],
            r['ori_geodesic'] if r['reachable'] else None)
            for r in sweep_results]
    finite = [v for _, v in vals if v is not None and math.isfinite(v)]
    if finite:
        max_err = max(finite) if max(finite) > 0 else 1.0
        print()
        print(f'  Geodesic error vs {sweep_axis} (° → higher = worse)')
        print('  ' + '─' * 52)
        for angle, err in vals:
            if err is None or not math.isfinite(err):
                bar = '∞'
            else:
                bar = '█' * int(round(err / max_err * 40))
            print(f'  {angle:>6.1f}° │{bar}  {err:.3f}°' if err is not None else
                f'  {angle:>6.1f}° │UNREACHABLE')
        print('  ' + '─' * 52)
        print(f'  Max geodesic error: {max_err:.4f}°')

    print(sep)
    print()

# ─────────────────────────────────────────────────────────────────────────────
# Setup helper — shared between run_validation and run_sweep
# ─────────────────────────────────────────────────────────────────────────────

def _setup(robot_obj, ee_label: str):
    """Resolve robot, build model, return (robot_model, T_world, q_home)."""
    from ..ui.command_ik_tool import (
        _build_parent_map,
        _get_chain_to_root,
        _build_robot_for_chain,
        _get_robot_global_transform,
        _read_joint_positions,
    )

    doc = fc.activeDocument()
    if doc is None:
        raise RuntimeError('No active FreeCAD document.')

    if robot_obj is None:
        robot_obj = next(
            (o for o in doc.Objects
            if hasattr(o, 'Proxy') and 'Robot' in type(o.Proxy).__name__),
            None,
        )
    if robot_obj is None:
        raise RuntimeError('No Cross::Robot object found in the document.')

    child_to_parent, child_to_joint = _build_parent_map(robot_obj)
    chain = _get_chain_to_root(robot_obj, ee_label, child_to_parent, child_to_joint)
    if not chain:
        raise RuntimeError(
            f"No kinematic chain found to '{ee_label}'. "
            f"Available leaf links: check _find_leaf_links() output."
        )

    robot_model = _build_robot_for_chain(robot_obj, chain)
    T_world     = _get_robot_global_transform(robot_obj)
    q_home      = _read_joint_positions(chain, robot_obj)

    return robot_model, T_world, q_home

# ─────────────────────────────────────────────────────────────────────────────
# Public API
# ─────────────────────────────────────────────────────────────────────────────

def run_validation(
    robot_obj=None,
    ee_label: str = _DEFAULT_EE_LABEL,
    target_pose: tuple | None = None # X, Y, Z, R°, P°, Y° of dialog 
) -> list[dict]:
    """
    Dynamically generate test poses from the active robot's geometry and
    run a three-level orientation validation report.

    No hardcoded positions or angles — all test poses are derived from:
    - The home FK position (q=0)
    - The actual joint limits of the loaded robot
    - The radial workspace distance from the Z axis

    Parameters
    ----------
    robot_obj : FreeCAD DocumentObject, optional
        Cross::Robot to validate. If None, uses the first robot found in
        the active document. When called from IKToolDialog, always passed
        explicitly so multi-robot documents work correctly.
    ee_label : str
        End-effector link label. Passed from the dialog's EE combo.
    """
    robot_model, T_world, q_home = _setup(robot_obj, ee_label)

    # Generate poses dynamically from this robot's geometry
    test_poses = _generate_test_poses(robot_model, T_world, q_home)

    robot_name = robot_obj.Label if robot_obj is not None else 'auto'
    print(f'\n[IK Validation] {len(test_poses)} poses (dynamic)  |  '
        f'EF: {ee_label}  |  robot: {robot_name}')

    # Print what was generated so the user can see the adapted suite
    print('  Generated test poses:')
    for pose in test_poses:
        tx, ty, tz, r, p, y, label = pose
        print(f'    ({tx:7.2f}, {ty:7.2f}, {tz:7.2f})  '
            f'RPY=({r:6.1f}°, {p:6.1f}°, {y:6.1f}°)  — {label}')
    print()

    results = []
    for pose in test_poses:
        tx, ty, tz, roll_d, pitch_d, yaw_d, label = pose
        target_world   = np.array([tx, ty, tz], dtype=float)
        R_target_world = _rpy_to_matrix(roll_d, pitch_d, yaw_d)

        q_final, steps_done = _run_ik(
            robot_model, T_world, q_home.copy(), target_world, R_target_world
        )

        if q_final is None:
            result = _make_result(
                label=label, target_pos=target_world,
                target_rpy_deg=(roll_d, pitch_d, yaw_d),
                achieved_pos=None, R_target=R_target_world, R_achieved=None,
                q_final=None, steps_done=steps_done,
                limit_viols=[], manip=_manipulability(robot_model, q_home),
            )
        else:
            T_f          = T_world @ forward_kinematics(robot_model, q_final)
            achieved_pos = T_f[:3, 3]
            R_achieved   = T_f[:3, :3]
            result = _make_result(
                label=label, target_pos=target_world,
                target_rpy_deg=(roll_d, pitch_d, yaw_d),
                achieved_pos=achieved_pos, R_target=R_target_world,
                R_achieved=R_achieved, q_final=q_final, steps_done=steps_done,
                limit_viols=_check_joint_limits(robot_model, q_final),
                manip=_manipulability(robot_model, q_final),
            )

        results.append(result)
        tick = '✅' if result['passed'] else ('💀' if not result['reachable'] else '❌')
        print(f'  {tick}  {label}')

    _print_report(results, ee_label)
    return results

def run_sweep(
    sweep_axis:   str   = 'Pitch',
    angle_min:    float = -90.0,
    angle_max:    float =  90.0,
    angle_step:   float =   5.0,
    base_pos:     tuple = (0, 48, 500),
    base_rpy:     tuple = (0, 0, 0),
    robot_obj           = None,
    ee_label:     str   = _DEFAULT_EE_LABEL,
) -> list[dict]:
    """
    Automatic angular sweep for singularity boundary mapping.

    Sweeps one rotation axis from angle_min to angle_max in angle_step steps,
    keeping position and the other two RPY angles fixed at base_pos / base_rpy.

    Parameters
    ----------
    sweep_axis  : 'Roll', 'Pitch', or 'Yaw'
    angle_min   : start angle in degrees
    angle_max   : end angle in degrees
    angle_step  : step size in degrees
    base_pos    : (X, Y, Z) mm — fixed position for all sweep poses
    base_rpy    : (Roll°, Pitch°, Yaw°) — base orientation; the sweep axis
                overrides the corresponding component
    robot_obj   : Cross::Robot object or None (auto-detect)
    ee_label    : end-effector link label

    Returns
    -------
    List of result dicts, one per sweep angle.
    """
    assert sweep_axis in ('Roll', 'Pitch', 'Yaw'), \
        "sweep_axis must be 'Roll', 'Pitch', or 'Yaw'"

    robot_model, T_world, q_home = _setup(robot_obj, ee_label)

    angles = list(np.arange(angle_min, angle_max + angle_step * 0.5, angle_step))
    print(f'\n[IK Sweep] {sweep_axis}  {angle_min}° → {angle_max}°  '
        f'step={angle_step}°  ({len(angles)} poses)  EF: {ee_label}')

    tx, ty, tz      = base_pos
    br, bp, by      = base_rpy
    axis_idx        = {'Roll': 0, 'Pitch': 1, 'Yaw': 2}[sweep_axis]
    target_world    = np.array([tx, ty, tz], dtype=float)

    sweep_results = []
    for angle in angles:
        rpy = [br, bp, by]
        rpy[axis_idx] = angle
        roll_d, pitch_d, yaw_d = rpy
        label          = f'{sweep_axis} {angle:+.1f}°'
        R_target_world = _rpy_to_matrix(roll_d, pitch_d, yaw_d)

        q_final, steps_done = _run_ik(
            robot_model, T_world, q_home.copy(), target_world, R_target_world
        )

        if q_final is None:
            result = _make_result(
                label=label, target_pos=target_world,
                target_rpy_deg=(roll_d, pitch_d, yaw_d),
                achieved_pos=None, R_target=R_target_world, R_achieved=None,
                q_final=None, steps_done=steps_done,
                limit_viols=[], manip=_manipulability(robot_model, q_home),
            )
        else:
            T_f          = T_world @ forward_kinematics(robot_model, q_final)
            achieved_pos = T_f[:3, 3]
            R_achieved   = T_f[:3, :3]
            result = _make_result(
                label=label, target_pos=target_world,
                target_rpy_deg=(roll_d, pitch_d, yaw_d),
                achieved_pos=achieved_pos, R_target=R_target_world,
                R_achieved=R_achieved, q_final=q_final, steps_done=steps_done,
                limit_viols=_check_joint_limits(robot_model, q_final),
                manip=_manipulability(robot_model, q_final),
            )

        sweep_results.append(result)
        sym = '✅' if result['passed'] else ('💀' if not result['reachable'] else '❌')
        geo = f"{result['ori_geodesic']:.3f}°" if result['reachable'] else '∞'
        print(f'  {sym}  {label:<14s}  geo={geo}')

    _print_sweep_report(sweep_results, sweep_axis, ee_label)
    return sweep_results

# ── Entry point (standalone exec mode only) ───────────────────────────────────
if __name__ == '__main__':
    validation_results = run_validation()