from typing import Optional
import numpy as np
from .kinematics import compute_forward_kinematics_full
from .jacobians import compute_jacobian
from .models import Robot


def rotation_error(current_rotation: np.ndarray, target_rotation: np.ndarray) -> np.ndarray:
    """
    Angular error vector between two rotation matrices.
    R_err = R_target · R_current^T  →  axial vector scaled by angle.
    """
    rotation_error_matrix = target_rotation @ current_rotation.T
    theta = np.arccos(np.clip((np.trace(rotation_error_matrix) - 1) / 2.0, -1.0, 1.0))
    if theta < 1e-9:
        return np.zeros(3)
    manipulability = np.array([
        rotation_error_matrix[2, 1] - rotation_error_matrix[1, 2],
        rotation_error_matrix[0, 2] - rotation_error_matrix[2, 0],
        rotation_error_matrix[1, 0] - rotation_error_matrix[0, 1],
    ]) / (2.0 * np.sin(theta))
    return theta * manipulability


def _manipulability(jacobian: np.ndarray) -> float:
    """Yoshikawa manipulability index on the linear (position) rows."""
    linear_jacobian = jacobian[:3, :]
    return float(np.sqrt(max(0.0, np.linalg.det(linear_jacobian @ linear_jacobian.T))))


def _escape_singularity(
    q: np.ndarray,
    robot: Robot,
    revolute_mask: np.ndarray,
    threshold: float = 1e-4,
    max_tries: int = 20,
    scale: float = 1.5,
    verbose: bool = False,
) -> np.ndarray:
    """
    Randomly perturb q until manipulability exceeds threshold.

    For robots with structurally coaxial joints the home configuration
    always has w ≈ 0. A single small perturbation is not enough — we try
    up to max_tries random perturbations of ±scale rad and keep the best.

    Joint wrapping is intentionally omitted here — for robots with
    negative-axis joints (axis=-Z, axis=-Y), wrapping to [-π, π] maps
    valid configurations to their physical opposite and corrupts the
    escape search.
    """
    best_q = q.copy()
    best_manipulability = _manipulability(compute_jacobian(robot, q))

    for _ in range(max_tries):
        q_try = q + np.random.uniform(-scale, scale, size=len(q))
        manipulability = _manipulability(compute_jacobian(robot, q_try))
        if manipulability > best_manipulability:
            best_manipulability = manipulability
            best_q = q_try.copy()
        if best_manipulability >= threshold:
            break

    if verbose:
        print(f"  escape_singularity: best manipulability={best_manipulability:.3e} after perturbations")
    return best_q


def inverse_kinematics(
    robot: Robot,
    target_position: np.ndarray,
    initial_guess: np.ndarray,
    target_rotation: Optional[np.ndarray] = None,
    weight_position: float = 1.0,
    weight_orientation: float = 0.05,
    max_iterations: int = 1000,
    tolerance: float = 1e-3,
    ori_tolerance_deg: float = 1.0,
    max_restarts: int = 5,
    verbose: bool = False,
    wrap_joints: bool = False,
) -> Optional[np.ndarray]:
    """
    Numerical Inverse Kinematics via Damped Least Squares (DLS).

    Step-size clamping
    ------------------
    Uses a *drive norm* that blends position and orientation errors so the
    solver keeps taking meaningful steps even after position has converged:

        drive_norm = max(position_error_norm,
                         ori_error_norm * ORI_DRIVE_SCALE)   # when use_orientation
        max_step   = clip(drive_norm * 0.05, 0.001, 0.3)

    This prevents the solver from clamping its step to 0.001 rad/mm when
    pos_error ≈ 0 but ori_error is still large (the classic ~100 ° stall).

    Convergence
    -----------
    Position-only mode  : converged when pos_error < tolerance.
    With orientation    : converged when BOTH
                            pos_error  < tolerance
                            ori_error  < ori_tolerance_deg  (default 1 °)

    The orientation threshold is intentionally loose because some robots
    (e.g. ATLAS with coaxial joints) cannot control all three rotation axes
    from every configuration. Tighten ori_tolerance_deg if you need stricter
    orientation accuracy and the robot has enough DOF.

    Singularity handling
    --------------------
    _escape_singularity() tries up to 20 random perturbations of ±1.5 rad
    and picks the configuration with highest manipulability before DLS.

    Joint wrapping (wrap_joints)
    ----------------------------
    Disabled by default. Wrapping revolute joints to [-π, π] causes
    incorrect solutions for robots with negative-axis joints (axis=-Z,
    axis=-Y): a valid angle such as 1.57 rad becomes -1.57 rad after
    wrapping, which is the physically opposite rotation. Only enable for
    robots with real joint limits where all rotation axes are positive.

    Parameters
    ----------
    robot              : Robot model.
    target_position    : (3,) XYZ in robot local frame [mm].
    initial_guess      : (N,) initial joint positions [rad / mm].
    target_rotation    : (3,3) rotation matrix or None for pos-only.
    weight_position    : position error weight (default 1.0).
    weight_orientation : orientation error weight (default 0.05).
    max_iterations     : DLS iterations per restart (default 1000).
    tolerance          : position convergence threshold [mm] (default 1e-3).
    ori_tolerance_deg  : orientation convergence threshold [deg] (default 1.0).
                        Only used when target_rotation is not None.
    max_restarts       : random restarts on stall (default 5).
    verbose            : print convergence/restart info (default False).
    wrap_joints        : wrap revolute joints to [-π, π] after each step
                        (default False — see note above).

    Returns
    -------
    (N,) joint positions if converged, None otherwise.
    """
    # Scale factor: how much ori_error_norm (rad) contributes to drive_norm (mm).
    # Chosen so that 0.1 rad (~6°) of orientation error produces the same drive
    # as ~10 mm of position error — keeps step sizes reasonable in both regimes.
    _ORI_DRIVE_SCALE = 100.0

    use_orientation = target_rotation is not None
    ori_tolerance_rad = np.deg2rad(ori_tolerance_deg)

    revolute_mask = np.array([
        j.joint_type in ("revolute", "continuous")
        for j in robot.joints
        if j.joint_type != "fixed"
    ], dtype=bool)

    for restart in range(max_restarts):
        if restart == 0:
            joint_positions = initial_guess.copy()
        else:
            joint_positions = initial_guess.copy()
            joint_positions += np.random.uniform(-1.5, 1.5, size=len(joint_positions))
            if verbose:
                print(f"IK restart {restart}, q={np.round(joint_positions, 3)}")

        # Escape singularity if needed.
        initial_jacobian = compute_jacobian(robot, joint_positions)
        initial_manipulability = _manipulability(initial_jacobian)
        if initial_manipulability < 1e-4:
            if verbose:
                print(f"Singular config (manipulability={initial_manipulability:.2e}) at restart {restart}, escaping...")
            joint_positions = _escape_singularity(
                joint_positions, robot, revolute_mask,
                threshold=1e-4, max_tries=20, scale=1.5, verbose=verbose,
            )

        previous_position_error = float("inf")
        stall_count = 0

        for iteration in range(max_iterations):
            all_transforms      = compute_forward_kinematics_full(robot, joint_positions)
            current_position    = all_transforms[-1][:3, 3]
            current_rotation    = all_transforms[-1][:3, :3]

            position_error      = target_position - current_position
            position_error_norm = float(np.linalg.norm(position_error))

            # ── Orientation error (always computed when use_orientation) ──────
            if use_orientation:
                orientation_error     = rotation_error(current_rotation, target_rotation)
                ori_error_norm        = float(np.linalg.norm(orientation_error))
            else:
                orientation_error = None
                ori_error_norm    = 0.0

            # ── Convergence check ─────────────────────────────────────────────
            # Position-only: exit as soon as pos_error < tolerance.
            # With orientation: require BOTH pos and ori to be within threshold.
            # This prevents the solver from returning early with pos_error ≈ 0
            # but ori_error ≈ 100 °, which was the original stall bug.
            pos_converged = position_error_norm < tolerance
            ori_converged = (not use_orientation) or (ori_error_norm < ori_tolerance_rad)

            if pos_converged and ori_converged:
                if verbose:
                    print(
                        f"Converged in {iteration} iters (restart {restart}), "
                        f"pos_err={position_error_norm:.4f} mm, "
                        f"ori_err={np.degrees(ori_error_norm):.4f} deg."
                    )
                return joint_positions

            # ── Build task error ──────────────────────────────────────────────
            if use_orientation:
                task_error = np.concatenate([
                    weight_position    * position_error,
                    weight_orientation * orientation_error,
                ])
            else:
                task_error = weight_position * position_error

            # ── Stall detection ───────────────────────────────────────────────
            position_error_change   = abs(previous_position_error - position_error_norm)
            stall_count = stall_count + 1 if position_error_change < 1e-10 else 0
            previous_position_error = position_error_norm
            if stall_count > 50:
                if verbose:
                    print(f"Stalled at iter {iteration}, pos_err={position_error_norm:.4f} mm.")
                break

            # ── DLS step ──────────────────────────────────────────────────────
            full_jacobian = compute_jacobian(robot, joint_positions)
            J = full_jacobian if use_orientation else full_jacobian[:3, :]
            U, S, Vh = np.linalg.svd(J, full_matrices=False)

            condition_number       = (S[0] / S[-1]) if S[-1] > 1e-12 else np.inf
            damping_lambda         = 0.1 if condition_number > 100 else (0.01 if condition_number > 10 else 0.001)
            damped_singular_values = S / (S ** 2 + damping_lambda ** 2)
            joint_delta            = Vh.T @ (damped_singular_values * (U.T @ task_error))

            # ── Step-size clamping ────────────────────────────────────────────
            # Bug fix: when pos_error ≈ 0 but ori_error is large, the old code
            # clamped max_step to 0.001 (its floor), making orientation corrections
            # impossibly slow (effectively stalling at ~100 ° for hundreds of iters).
            #
            # Fix: use a *drive_norm* that falls back to the orientation error
            # (scaled to mm-equivalent units) when position has already converged.
            # This keeps the step budget large enough to rotate the EF.
            if use_orientation:
                drive_norm = max(position_error_norm, ori_error_norm * _ORI_DRIVE_SCALE)
            else:
                drive_norm = position_error_norm

            step_norm = np.linalg.norm(joint_delta)
            max_step  = np.clip(drive_norm * 0.05, 0.001, 0.3)
            if step_norm > max_step:
                joint_delta = joint_delta * (max_step / step_norm)

            joint_positions += joint_delta
            if wrap_joints:
                joint_positions[revolute_mask] = (
                    np.mod(joint_positions[revolute_mask] + np.pi, 2 * np.pi) - np.pi
                )

    if verbose:
        print(f"IK did not converge after {max_restarts} restarts.")
    return None