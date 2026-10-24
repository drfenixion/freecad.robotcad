from typing import Optional
import numpy as np
from .kinematics import compute_forward_kinematics_full
from .jacobians import compute_jacobian
from .models import Robot

def inverse_kinematics(
    robot: Robot, 
    target_position: np.ndarray, 
    initial_guess: np.ndarray, 
    max_iterations: int = 1000, 
    tolerance: float = 1e-4, 
    max_restarts: int = 5
) -> Optional[np.ndarray]:
    """
    Computes the numerical Inverse Kinematics (IK) to find the joint configurations 
    required to reach a targeted 3D Cartesian position using Damped Least Squares (DLS).

    Parameters:
    -----------
    robot : Robot
        The structural robot data model containing joint and link definitions.
    target_position : np.ndarray
        A 1D array of shape (3,) representing the target X, Y, Z coordinates in meters.
    initial_guess : np.ndarray
        A 1D array representing the initial joint positions to start the optimization loop.
    max_iterations : int, optional
        Maximum number of optimization iterations per restart attempt (default is 1000).
    tolerance : float, optional
        The maximum allowed Euclidean residual position error for convergence (default is 1e-4).
    max_restarts : int, optional
        Maximum number of random restarts allowed if the solver gets trapped in local 
        minima or singularities (default is 5).

    Returns:
    --------
    Optional[np.ndarray]
        A 1D array of calculated joint positions if convergence is successful, 
        or None if the solver fails to converge within the limits.
    """
    
    for restart_index in range(max_restarts):
        if restart_index == 0:
            current_joint_positions = initial_guess.copy()
        else:
            # Apply a random uniform perturbation to escape local minima or singularities
            current_joint_positions = initial_guess.copy()
            current_joint_positions += np.random.uniform(-0.3, 0.3, size=len(current_joint_positions))
            print(f"IK restart {restart_index} with perturbed joint positions = {np.round(current_joint_positions, 3)}")

        # Boolean mask to isolate joints subject to angular wrapping controls
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
            
            position_error = target_position - current_position
            current_error_norm = np.linalg.norm(position_error)

            # Check convergence criteria
            if current_error_norm < tolerance:
                print(f"Converged in {iteration} iterations (restart {restart_index}).")
                return current_joint_positions

            # Check if optimization has stalled
            if abs(previous_error_norm - current_error_norm) < 1e-10:
                stall_counter += 1
            else:
                stall_counter = 0
            previous_error_norm = current_error_norm

            if stall_counter > 30:
                print(f"Stalled at iteration {iteration}, error = {current_error_norm:.6f}. Triggering restart.")
                break  # Exit iteration loop to try a new random restart

            # Jacobian computation and linear velocity extraction
            full_jacobian = compute_jacobian(robot, current_joint_positions)
            linear_jacobian = full_jacobian[:3, :]

            # Damped Least Squares (Levenberg-Marquardt damping factor) to avoid singularity explosions
            damping_lambda = 0.01
            damping_identity = (damping_lambda ** 2) * np.eye(3)
            coefficient_matrix = linear_jacobian @ linear_jacobian.T + damping_identity
            
            # Compute joint step vector
            joint_delta_step = linear_jacobian.T @ np.linalg.solve(coefficient_matrix, position_error)

            # Restrict step size to ensure stability and smooth optimization transitions
            step_norm = np.linalg.norm(joint_delta_step)
            if step_norm > 0.1:
                joint_delta_step = joint_delta_step * (0.1 / step_norm)

            # Update configurations and wrap angular positions to [-pi, pi] limits
            current_joint_positions += joint_delta_step
            current_joint_positions[revolute_joint_mask] = (
                np.mod(current_joint_positions[revolute_joint_mask] + np.pi, 2 * np.pi) - np.pi
            )

    print(f"IK did not converge after {max_restarts} restarts.")
    return None