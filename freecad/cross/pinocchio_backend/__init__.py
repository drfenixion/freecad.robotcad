"""
Pinocchio backend
=================

Backend integration of Pinocchio for RobotCAD/RoboTool.

The purpose of this package is to provide an independent and robust
implementation of robot model processing and kinematic computations.
Pinocchio is used as the underlying numerical library for model handling,
forward kinematics, Jacobians, and related operations.

This backend serves two complementary purposes:

* provide reliable numerical processing for RobotCAD;
* provide an independent reference for validating RoboTool's own
  kinematic and inverse-kinematic implementations.

RoboTool and this backend are therefore kept conceptually separate:
RoboTool defines its own mathematical pipeline, while Pinocchio provides
an independent computational path against which results can be compared.

Package modules
---------------

models
    Robot model creation and model metadata.

kinematics
    Forward kinematics and frame transformations.

jacobians
    Joint and frame Jacobian computation.

ik
    Numerical inverse kinematics and solver utilities.
"""

from .models_pinocchio import (
    frame_names,
    load_model,
    load_model_from_xml_string,
    joint_limits,
    joint_names,
    frame_names,
    get_joint_id,
    get_frame_id,
    get_joint_type,
    robot_dof,
)

from .kinematics_pinocchio import (
    compute_fk,
    get_ee_transform,
    get_ee_rpy,
)

from .jacobians_pinocchio import (
    compute_jacobian,
    validate_jacobian_numerically,
)

from .inverse_kinematics_pinocchio import (
    inverse_kinematics,
)

__all__ = [
    # Model
    "load_model",
    "load_model_from_xml_string",
    "link_names",
    "joint_limits",
    "joint_names",
    "frame_names",
    "get_joint_id",
    "get_frame_id",
    "get_joint_type",
    "robot_dof",
    # Kinematics
    "compute_fk",
    "get_ee_transform",
    "get_ee_rpy",
    # Jacobians
    "compute_jacobian",
    "validate_jacobian_numerically",
    # IK
    "inverse_kinematics",
]