import pinocchio as pin
import numpy as np
from .kinematics_pinocchio import get_ee_transform

def compute_jacobian(model, data, q, ee_frame_name):
    """
    Compute the geometric Jacobian at a named end-effector frame.
    Uses LOCAL_WORLD_ALIGNED reference frame (linear and angular velocity
    expresed in the world frame, but the linear velocity is expressed at the
    origin of the end-effector frame).

    Parameters:
        model (pin.Model): The Pinocchio robot model.

        data (pin.Data): The Pinocchio data structure associated with the model.

        q (np.ndarray): The joint configuration vector (angles in radians or displacements in meters).

        ee_frame_name (str): The name of the end-effector frame for which to compute the Jacobian.
    
    Returns:
        np.ndarray: A 6xN Jacobian matrix, where N is the number of active joints.
            Rows 0:3 correspond to linear velocity, and rows 3:6 correspond to angular velocity.
    """
    frame_id = model.getFrameId(ee_frame_name)
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacement(model, data)
    jacobian = pin.getFrameJacobian(
        model,data,frame_id,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )
    return jacobian

def validate_jacobian_numerically(model, data, q, ee_frame_name, epsilon=1e-6):
    """
    Validate the analytic Jacobian against a finite differences approximation.
    Angular velocity column is extracted via the SO(3) logarithm of the
    relative rotation, which is the correct formulation on the manifold.
    
    Parameters:
        model (pin.Model): The Pinocchio robot model.

        data (pin.Data): The Pinocchio data structure associated with the model.

        q (np.ndarray): The joint configuration vector (angles in radians or displacements in meters).

        ee_frame_name (str): The name of the end-effector frame for which to compute the Jacobian.

        epsilon (float, optional): The small perturbation value for finite differences. Default is 1e-6.
    Returns:
        tuple[np.ndarray, np.ndarray]: (analytic_jacobian, numerical_jacobian),
            where both are 6xN matrices, with rows 0:3 corresponding to linear velocity,
            and rows 3:6 corresponding to angular velocity.
        """
    analytic_jacobian = compute_jacobian(model, data, q, ee_frame_name)
    frame_id = model.getFrameId(ee_frame_name)
    nv = model.nv
    numerical_jacobian = np.zeros((6,nv))

    #Reference SE3 at original configuration
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data)
    R_ref = data.oMf[frame_id].rotation.copy()
    p_ref = data.oMf[frame_id].translation.copy()

    for i in range(nv):
        q_plus = q.copy()
        q_plus[i] += epsilon
        pin.forwardKinematics(model, data, q_plus)
        pin.updateFramePlacement(model, data)
        t_plus = data.oMf[frame_id].translation.copy()
        R_plus = data.oMf[frame_id].rotation.copy()

        q_minus = q.copy()
        q_minus[i] -= epsilon
        pin.forwardKinematics(model, data, q_minus)
        pin.updateFramePlacement(model, data)
        t_minus = data.oMf[frame_id].translation.copy()
        R_minus = data.oMf[frame_id].rotation.copy()

        # Linear velocity: central difference on translation
        linear = (t_plus - t_minus) / (2 * epsilon)

        # Angular velocity: log of relative rotation (correct on SO(3))
        dR = R_plus @ R_minus.T
        angular = pin.log3(dR) / (2 * epsilon)

        numerical_jacobian[:3,i] = linear
        numerical_jacobian[3:,i] = angular

    return analytic_jacobian, numerical_jacobian