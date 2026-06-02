from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np

from .models import Robot, Link, Joint

def joint_to_matrix(joint: Joint) -> np.ndarray:
    """
    Converts a joint origin (x,y,z and rpy) into a 4x4 homogeneous transformation matrix.
    Uses Extrinsic XYZ Euler angles convention.
    """
    x, y, z = joint.origin_xyz
    r, p, y_ang = joint.origin_rpy

    # Rotation matrices for each axis
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y_ang), -np.sin(y_ang), 0], [np.sin(y_ang), np.cos(y_ang), 0], [0, 0, 1]])

    # Combined rotation matrix R = Rz * Ry * Rx 
    R = Rz @ Ry @ Rx

    # Construct 4x4 homogeneous tranformation matrix 
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T
    
def compute_forward_kinematics_full(robot: Robot, q: np.ndarray) -> List[np.ndarray]:
    """
    Computes all transformation matrices along the kinematic chain.
    """
    all_transforms = [np.eye(4)]  # Base (World / Link 0)
    q_idx = 0  # Index for the joint angles in q

    for joint in robot.joints:
        # 1. Fixed transformation matrix of the origin of the joint (of the URDF)
        T_urdf = joint_to_matrix(joint)
        
        # 2. Internal motion matrix of the joint (based on q[i])
        T_motion = np.eye(4)

        if joint.joint_type in ["revolute", "continuous"]:
            qi = q[q_idx] if q_idx < len(q) else 0.0  # Default to 0 if not enough angles provided
            # Rotation using Rodrigues' formula on the joint axis
            norm = np.linalg.norm(joint.axis)
            if norm < 1e-8:
                raise ValueError(f"Joint {joint.name} has an invalid axis with near-zero length.")
            axis = np.array(joint.axis) / norm # only guard against zero-length axis, not against non-unit length, since we normalize it anyway
            # Coupling matrix by cross product
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            # Rodrigues' formula for the 3x3 rotation matrix
            R_motion = np.eye(3) + np.sin(qi) * K + (1 - np.cos(qi)) * (K @ K)
            T_motion[:3, :3] = R_motion
            q_idx += 1 # increase index for the next joint angle

        elif joint.joint_type == "prismatic":
            # Translation along the joint axis
            qi = q[q_idx] if q_idx < len(q) else 0.0 # Default to 0 if not enough angles provided
            norm = np.linalg.norm(joint.axis)
            if norm < 1e-8:
                raise ValueError(f"Joint {joint.name} has an invalid axis with near-zero length.")
            axis = np.array(joint.axis) / norm # only guard against zero-length axis, not against non-unit length, since we normalize it anyway
            T_motion[:3, 3] = axis * qi
            q_idx += 1 # increase index for the next joint angle

            #fixed joints do not contribute to motion, so T_motion remains identity
        
        # Accumulate the transformation with respect to the base of the robot
        all_transforms.append(all_transforms[-1] @ T_urdf @ T_motion)

    return all_transforms

def forward_kinematics(robot: Robot, q: np.ndarray) -> np.ndarray:
    """
    Computes the final transformation matrix by multiplying all joint transformations.
    Currently assumes a serial chain in the order defined in the URDF.
    """
    all_transforms = compute_forward_kinematics_full(robot, q)
    return all_transforms[-1]
