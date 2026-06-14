import numpy as np
from .models import Robot
from .kinematics import compute_forward_kinematics_full, joint_to_matrix

def compute_jacobian(robot: Robot, joint_positions: np.ndarray) -> np.ndarray:
    """
    Computes the standard 6xN analytical/geometric Jacobian matrix for the manipulator.

    Parameters:
    -----------
    robot : Robot
        The structural robot data model.
    joint_positions : np.ndarray
        Vector containing the current active joint positions.
    Returns:
    --------
    np.ndarray
        The computed 6xN geometric Jacobian matrix.
    """
    active_joints = [joint for joint in robot.joints if joint.joint_type != "fixed"]
    total_active_joints = len(active_joints)
    jacobian_matrix = np.zeros((6, total_active_joints))

    all_transforms = compute_forward_kinematics_full(robot, joint_positions)
    end_effector_position = all_transforms[-1][:3, 3]

    current_column = 0
    for i, joint in enumerate(robot.joints):
        if joint.joint_type == "fixed":
            continue

        # Frame of the parent link (before the URDF offset of this joint)
        T_parent = all_transforms[i]  # all_transforms[0] is base, [1] is after joint 0, etc.
        # Fixed URDF transform for this joint
        T_urdf = joint_to_matrix(joint)
        # Frame of the joint BEFORE the motion (includes fixed offset, excludes q[i])
        T_frame_articulation = T_parent @ T_urdf
        position_frame_articulation = T_frame_articulation[:3, 3]
        rotation_frame_articulation = T_frame_articulation[:3, :3]
        axis_joint_local = np.array(joint.axis) / np.linalg.norm(joint.axis)
        axis_z_transformed = rotation_frame_articulation @ axis_joint_local

        if joint.joint_type in ["revolute", "continuous"]:
            vector_difference_position = end_effector_position - position_frame_articulation
            jacobian_matrix[:3, current_column] = np.cross(axis_z_transformed, vector_difference_position)
            jacobian_matrix[3:, current_column] = axis_z_transformed
        elif joint.joint_type == "prismatic":
            jacobian_matrix[:3, current_column] = axis_z_transformed
            jacobian_matrix[3:, current_column] = np.zeros(3)

        current_column += 1

    return jacobian_matrix

def validate_jacobian_numerically(robot: Robot, joint_positions: np.ndarray, epsilon: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    """
    Validates the analytic geometric Jacobian matrix against a finite differences matrix approximation.

    Parameters:
    -----------
    robot : Robot
        The structural robot data model.
    joint_positions : np.ndarray
        Vector of the configuration state to perform the evaluation.
    epsilon : float, optional
        The scalar delta step size for central/forward difference calculation (default is 1e-6).
    Returns:
    --------
    tuple[np.ndarray, np.ndarray]
        A tuple containing (jacobian_analytic, jacobian_numeric) for structural evaluation.
    """
    jacobian_analytic = compute_jacobian(robot, joint_positions)
    total_active_joints = jacobian_analytic.shape[1]
    jacobian_numeric = np.zeros((6, total_active_joints))

    initial_transform = compute_forward_kinematics_full(robot, joint_positions)[-1]
    initial_position = initial_transform[:3, 3]
    initial_rotation = initial_transform[:3, :3]

    for i in range(total_active_joints):
        perturbed_positions = joint_positions.copy()
        perturbed_positions[i] += epsilon
        
        perturbed_transform = compute_forward_kinematics_full(robot, perturbed_positions)[-1]
        perturbed_position = perturbed_transform[:3, 3]
        perturbed_rotation = perturbed_transform[:3, :3]
        
        # Calculate linear velocity component via positional difference
        jacobian_numeric[:3, i] = (perturbed_position - initial_position) / epsilon  
        
        # Extract angular velocity component from the rotation difference matrix
        rotation_difference = perturbed_rotation @ initial_rotation.T
        jacobian_numeric[3:, i] = np.array([
            rotation_difference[2, 1] - rotation_difference[1, 2], 
            rotation_difference[0, 2] - rotation_difference[2, 0], 
            rotation_difference[1, 0] - rotation_difference[0, 1]
        ]) / (2 * epsilon)

    return jacobian_analytic, jacobian_numeric