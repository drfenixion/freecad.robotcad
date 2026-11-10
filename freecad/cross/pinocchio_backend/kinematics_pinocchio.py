import pinocchio as pin 
import numpy as np

def compute_fk(model, data, q):
    """
    Computes the forward kinematics for a given robot model and joint configuration.

    Parameters:
    -----------
    model : pinocchio.Model
        The Pinocchio robot model.
    data : pinocchio.Data
        The Pinocchio data structure associated with the model.
    q : np.ndarray
        The joint configuration vector (angles in radians or displacements in meters).

    Returns:
    --------
    List[np.ndarray]
        A list of 4x4 homogeneous transformation matrices representing the pose of each link in the kinematic chain.
    """
    pin.forwardKinematics(model, data, q)
    return list(data.oMi)  # Returns a list of transformation matrices for each link

def get_ee_transform(model, data, q, ee_frame_name):
    """
    Computes the end-effector transformation matrix for a given robot model and joint configuration.

    Parameters:
    -----------
    model : pinocchio.Model
        The Pinocchio robot model.
    data : pinocchio.Data
        The Pinocchio data structure associated with the model.
    q : np.ndarray
        The joint configuration vector (angles in radians or displacements in meters).

    Returns:
    --------
    np.ndarray
        A 4x4 homogeneous transformation matrix representing the pose of the end-effector.
    """
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data)
    frame_id = model.getFrameId(ee_frame_name)  # Replace 'ee_frame_name' with the actual end-effector frame name
    se3 = data.oMf[frame_id]  # Get the SE3 object for the end-effector frame
    T = np.eye(4)
    T[:3,:3] = se3.rotation  # Set the rotation part
    T[:3,3] = se3.translation  # Set the translation part
    return T

def get_ee_rpy(model, data, q, ee_frame_name):
    """
    Computes the end-effector roll, pitch, and yaw angles for a given robot model and joint configuration.

    Parameters:
    -----------
    model : pinocchio.Model
        The Pinocchio robot model.
    data : pinocchio.Data
        The Pinocchio data structure associated with the model.
    q : np.ndarray
        The joint configuration vector (angles in radians or displacements in meters).
    ee_frame_name : str
        The name of the end-effector frame.

    Returns:
    --------
    np.ndarray
        A 1D array containing the roll, pitch, and yaw angles of the end-effector.
    """
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data)
    frame_id = model.getFrameId(ee_frame_name)  # Replace 'ee_frame_name' with the actual end-effector frame name
    se3 = data.oMf[frame_id]  # Get the SE3 object for the end-effector frame
    rpy = pin.rpy.matrixToRpy(se3.rotation)  # Convert rotation matrix to roll, pitch, yaw angles
    return se3.translation, rpy