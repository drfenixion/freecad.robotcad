from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np
from .models import Robot, Link, Joint

def joint_to_matrix(joint: Joint) -> np.ndarray:
    """
    Converts a joint origin (x, y, z and rpy) into a 4x4 homogeneous transformation matrix.
    Uses Extrinsic XYZ Euler angles convention.

    Parameters:
    -----------
    joint : Joint
        The target joint object containing origin spatial parameters.
    Returns:
    --------
    np.ndarray
        A 4x4 homogeneous transformation matrix representing spatial pose.
    """
    position_x, position_y, position_z = joint.origin_xyz
    roll, pitch, yaw = joint.origin_rpy

    # Rotation matrices for each spatial axis
    rotation_matrix_x = np.array([
        [1, 0, 0], 
        [0, np.cos(roll), -np.sin(roll)], 
        [0, np.sin(roll), np.cos(roll)]
    ])
    rotation_matrix_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)], 
        [0, 1, 0], 
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    rotation_matrix_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0], 
        [np.sin(yaw), np.cos(yaw), 0], 
        [0, 0, 1]
    ])

    # Combined rotation matrix R = Rz * Ry * Rx 
    combined_rotation = rotation_matrix_z @ rotation_matrix_y @ rotation_matrix_x

    # Construct 4x4 homogeneous transformation matrix 
    transform_homogeneous = np.eye(4)
    transform_homogeneous[:3, :3] = combined_rotation
    transform_homogeneous[:3, 3] = [position_x, position_y, position_z]
    return transform_homogeneous

def compute_forward_kinematics_full(robot: Robot, joint_positions: np.ndarray) -> List[np.ndarray]:
    """
    Computes all consecutive transformation matrices along the robot's kinematic chain.

    Parameters:
    -----------
    robot : Robot
        The structural robot data model.
    joint_positions : np.ndarray
        Vector of active joint values (angles in radians or displacements in meters).
    Returns:
    --------
    List[np.ndarray]
        List containing 4x4 transformation matrices for each coordinate frame from the base.
    """
    all_transforms = [np.eye(4)]  # Base (World / Link 0)
    joint_angles_index = 0  # Index tracker for active configurations inside joint_positions

    for joint in robot.joints:
        # 1. Fixed transformation matrix of the origin of the joint (of the URDF)
        transform_urdf = joint_to_matrix(joint)
        
        # 2. Internal motion matrix of the joint (based on current joint state)
        transform_motion = np.eye(4)

        if joint.joint_type in ["revolute", "continuous"]:
            current_angle = joint_positions[joint_angles_index] if joint_angles_index < len(joint_positions) else 0.0
            axis_norm = np.linalg.norm(joint.axis)
            if axis_norm < 1e-8:
                raise ValueError(f"Joint {joint.name} has an invalid axis with near-zero length.")
            normalized_axis = np.array(joint.axis) / axis_norm
            
            # Coupling skew-symmetric matrix for cross product operations
            skew_matrix = np.array([
                [0, -normalized_axis[2], normalized_axis[1]],
                [normalized_axis[2], 0, -normalized_axis[0]],
                [-normalized_axis[1], normalized_axis[0], 0]
            ])
            # Rodrigues' formula for the 3x3 rotation matrix
            rotation_motion = np.eye(3) + np.sin(current_angle) * skew_matrix + (1 - np.cos(current_angle)) * (skew_matrix @ skew_matrix)
            transform_motion[:3, :3] = rotation_motion
            joint_angles_index += 1

        elif joint.joint_type == "prismatic":
            current_translation = joint_positions[joint_angles_index] if joint_angles_index < len(joint_positions) else 0.0
            axis_norm = np.linalg.norm(joint.axis)
            if axis_norm < 1e-8:
                raise ValueError(f"Joint {joint.name} has an invalid axis with near-zero length.")
            normalized_axis = np.array(joint.axis) / axis_norm
            transform_motion[:3, 3] = normalized_axis * current_translation
            joint_angles_index += 1
        
        # Accumulate the transformation with respect to the base of the robot
        all_transforms.append(all_transforms[-1] @ transform_urdf @ transform_motion)

    return all_transforms

def forward_kinematics(robot: Robot, joint_positions: np.ndarray) -> np.ndarray:
    """
    Computes the final end-effector transformation matrix relative to the base frame.

    Parameters:
    -----------
    robot : Robot
        The structural robot data model.
    joint_positions : np.ndarray
        Vector of active joint positions.
    Returns:
    --------
    np.ndarray
        4x4 homogeneous transformation matrix of the terminal end-effector frame.
    """
    all_transforms = compute_forward_kinematics_full(robot, joint_positions)
    return all_transforms[-1]