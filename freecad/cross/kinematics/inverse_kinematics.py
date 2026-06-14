from typing import Optional
import numpy as np
from .kinematics import compute_forward_kinematics_full
from .jacobians import compute_jacobian
from .models import Robot


def rotation_error(Rotation_current: np.ndarray, Rotation_target: np.ndarray) -> np.ndarray:
    """
    Computes the angular error vector between two rotation matrices.
    Extracts the axial vector of the rotation error matrix R_err = R_current^T · R_target.

    Parameters:
    -----------
    Rotation_current : np.ndarray
        Current end-effector rotation matrix, shape (3, 3).
    Rotation_target : np.ndarray
        Target end-effector rotation matrix, shape (3, 3).

    Returns:
    --------
    np.ndarray
        Angular error vector of shape (3,) in the world frame.
    """
    R_err = Rotation_target @ Rotation_current.T
    theta = np.arccos(np.clip((np.trace(R_err) - 1) / 2.0, -1.0, 1.0))
    
    if theta < 1e-9:
        return np.zeros(3)
    
    # Axial vector: (R_err - R_err.T) / (2*sin(theta)) is the unit axis u
    # The necessary angular velocity is theta * u
    w = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1]
    ]) / (2.0 * np.sin(theta))
    
    return theta * w



def inverse_kinematics(
    robot: Robot,
    target_position: np.ndarray,
    initial_guess: np.ndarray,
    target_rotation: Optional[np.ndarray] = None,
    weight_position: float = 1.0,
    weight_orientation: float = 0.1,
    max_iterations: int = 1000,
    tolerance: float = 1e-4,
    max_restarts: int = 5
) -> Optional[np.ndarray]:
    """
    Computes the numerical Inverse Kinematics (IK) to find the joint configurations
    required to reach a targeted Cartesian pose using Damped Least Squares (DLS).

    Supports both 3-DOF (position only) and 6-DOF (position + orientation) control.
    When target_rotation is None, the solver operates in position-only mode (original behavior).

    DLS formulation:
        full_error = [w_pos * Δp,  w_ori * Δω]      shape (3,) or (6,)
        J          = compute_jacobian(robot, q)       shape (3,N) or (6,N)
        Δq         = Jᵀ (J Jᵀ + λ²I)⁻¹ · full_error

    Step size is adaptive: max_step = clip(0.05 * error_norm, 0.05, 0.5).
    This allows large steps far from the target and fine steps near convergence.

    Singularity detection: if the initial configuration has manipulability
    w = sqrt(det(J[:3] J[:3]ᵀ)) < 1e-3, a random perturbation is applied
    before the first iteration to escape the degenerate configuration.

    Parameters:
    -----------
    robot : Robot
        The structural robot data model containing joint and link definitions.
    target_position : np.ndarray
        A 1D array of shape (3,) representing the target X, Y, Z coordinates.
    initial_guess : np.ndarray
        A 1D array representing the initial joint positions to start the optimization loop.
    target_rotation : Optional[np.ndarray]
        A (3, 3) rotation matrix representing the target end-effector orientation.
        If None, the solver ignores orientation and runs in position-only mode.
    weight_position : float, optional
        Scalar weight applied to the position error components (default 1.0).
    weight_orientation : float, optional
        Scalar weight applied to the orientation error components (default 0.1).
        Tune this to balance translational vs rotational convergence speed.
    max_iterations : int, optional
        Maximum number of optimization iterations per restart attempt (default 1000).
    tolerance : float, optional
        The maximum allowed Euclidean residual error for convergence (default 1e-4).
    max_restarts : int, optional
        Maximum number of random restarts allowed if the solver gets trapped in local
        minima or singularities (default 5).

    Returns:
    --------
    Optional[np.ndarray]
        A 1D array of calculated joint positions if convergence is successful,
        or None if the solver fails to converge within the limits.
    """
    use_orientation = target_rotation is not None
    error_dim = 6 if use_orientation else 3

    for restart_index in range(max_restarts):
        if restart_index == 0:
            current_joint_positions = initial_guess.copy()
            # Detect singular initial configuration and perturb to escape it.
            # w = sqrt(det(J_linear @ J_linear.T)) — Yoshikawa manipulability index.
            # w ≈ 0 means the robot is at or near a kinematic singularity where
            # the DLS step collapses to zero and the solver stalls immediately.
            J0 = compute_jacobian(robot, current_joint_positions)
            w0 = np.sqrt(max(0.0, np.linalg.det(J0[:3, :] @ J0[:3, :].T)))
            if w0 < 1e-3:
                current_joint_positions += np.random.uniform(-0.5, 0.5, size=len(current_joint_positions))
                print(f"Singular initial config detected (w={w0:.2e}), applying perturbation.")
        else:
            # Random restart to escape local minima or stall regions.
            current_joint_positions = initial_guess.copy()
            current_joint_positions += np.random.uniform(-1.0, 1.0, size=len(current_joint_positions))
            print(f"IK restart {restart_index} with perturbed joint positions = {np.round(current_joint_positions, 3)}")

        # Boolean mask to isolate joints subject to angular wrapping controls.
        revolute_joint_mask = np.array([
            joint.joint_type in ["revolute", "continuous"]
            for joint in robot.joints
            if joint.joint_type != "fixed"
        ], dtype=bool)

        previous_error_norm = float('inf')
        stall_counter = 0

        for iteration in range(max_iterations):
            all_transforms = compute_forward_kinematics_full(robot, current_joint_positions)
            current_position = all_transforms[-1][:3, 3]
            current_rotation = all_transforms[-1][:3, :3]

            position_error = target_position - current_position  # shape (3,)

            if use_orientation:
                orientation_error = rotation_error(current_rotation, target_rotation)  # shape (3,)
                full_error = np.concatenate([
                    weight_position    * position_error,
                    weight_orientation * orientation_error,
                ])  # shape (6,)
            else:
                full_error = position_error  # shape (3,) — original behavior

            current_error_norm = np.linalg.norm(full_error)

            # Check convergence criteria.
            if current_error_norm < tolerance:
                print(f"Converged in {iteration} iterations (restart {restart_index}).")
                return current_joint_positions

            # Stall detection — exit inner loop to trigger a new restart.
            if abs(previous_error_norm - current_error_norm) < 1e-10:
                stall_counter += 1
            else:
                stall_counter = 0
            previous_error_norm = current_error_norm

            if stall_counter > 30:
                print(f"Stalled at iteration {iteration}, error = {current_error_norm:.6f}. Triggering restart.")
                break

            # Select Jacobian rows based on mode: full 6×N or linear-only 3×N.
            full_jacobian = compute_jacobian(robot, current_joint_positions)
            Jacobian = full_jacobian if use_orientation else full_jacobian[:3, :]

            # SVD for damping factor calculation and pseudo-inverse stability.
            U, S, Vh = np.linalg.svd(Jacobian, full_matrices=False)

            # Damping factor λ selection based on condition number of the Jacobian.
            sigma_min = S[-1]
            sigma_max = S[0]
            condition_number = sigma_max / sigma_min if sigma_min > 1e-12 else np.inf
            if condition_number > 100:
                damping_lambda = 0.1
            elif condition_number > 10:
                damping_lambda = 0.01
            else:
                damping_lambda = 0.001

            # Factor damped
            damped_factors = S / (S**2 + damping_lambda**2)  # shape (error_dim,)

            # Calulation for Δq = Vhᵀ @ diag(σ/(σ²+λ²)) @ Uᵀ @ error
            joint_delta_step = Vh.T @ (damped_factors * (U.T @ full_error))

            # Clamp and wrapping 
            step_norm = np.linalg.norm(joint_delta_step)
            max_step = np.clip(current_error_norm * 0.1, 0.001, 0.5)
            if step_norm > max_step:
                joint_delta_step = joint_delta_step * (max_step / step_norm)

            current_joint_positions += joint_delta_step
            current_joint_positions[revolute_joint_mask] = (
            np.mod(current_joint_positions[revolute_joint_mask] + np.pi, 2 * np.pi) - np.pi
            )

    print(f"IK did not converge after {max_restarts} restarts.")
    return None