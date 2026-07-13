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
    max_restarts: int = 5,
    verbose: bool = False,
    wrap_joints: bool = False,
) -> Optional[np.ndarray]:
    """
    Numerical Inverse Kinematics via Damped Least Squares (DLS).

    Step-size clamping
    ------------------
    Uses position error norm only [mm] — not the mixed pos+ori norm.

        max_step = clip(position_error_norm * 0.05, 0.001, 0.3)

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
    max_restarts       : random restarts on stall (default 5).
    verbose            : print convergence/restart info (default False).
    wrap_joints        : wrap revolute joints to [-π, π] after each step
                         (default False — see note above).

    Returns
    -------
    (N,) joint positions if converged, None otherwise.
    """
    use_orientation = target_rotation is not None

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

            if position_error_norm < tolerance:
                if verbose:
                    print(f"Converged in {iteration} iters (restart {restart}), "
                          f"position_error={position_error_norm:.4f} mm.")
                return joint_positions

            if use_orientation:
                orientation_error = rotation_error(current_rotation, target_rotation)
                task_error = np.concatenate([
                    weight_position    * position_error,
                    weight_orientation * orientation_error,
                ])
            else:
                task_error = weight_position * position_error

            position_error_change   = abs(previous_position_error - position_error_norm)
            stall_count = stall_count + 1 if position_error_change < 1e-10 else 0
            previous_position_error = position_error_norm
            if stall_count > 50:
                if verbose:
                    print(f"Stalled at iter {iteration}, pos_err={position_error_norm:.4f} mm.")
                break

            full_jacobian = compute_jacobian(robot, joint_positions)
            J = full_jacobian if use_orientation else full_jacobian[:3, :]
            U, S, Vh = np.linalg.svd(J, full_matrices=False)

            condition_number       = (S[0] / S[-1]) if S[-1] > 1e-12 else np.inf
            damping_lambda         = 0.1 if condition_number > 100 else (0.01 if condition_number > 10 else 0.001)
            damped_singular_values = S / (S ** 2 + damping_lambda ** 2)
            joint_delta            = Vh.T @ (damped_singular_values * (U.T @ task_error))

            step_norm = np.linalg.norm(joint_delta)
            max_step  = np.clip(position_error_norm * 0.05, 0.001, 0.3)
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