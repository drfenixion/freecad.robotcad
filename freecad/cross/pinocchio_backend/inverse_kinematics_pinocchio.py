import pinocchio as pin
import numpy as np

def inverse_kinematics(
    model, data, q_init, ee_frame_name,
    target_pose, max_iterations=100, tolerance=1e-6,
    max_step_size=0.1, damping=1e-6, verbose=False
):
    """
    Damped Least Squares (DSL) inverse kinematics ussing Pinocchio.
    Error is computed as the SE3 logarithm of (current^{-1} * target),
    which is the correct formulation on the  SE(3)manifold.

    Parameters:
        model (pin.Model): The Pinocchio robot model.
        data(pin.Data): The Pinocchio data structure associated with the model.
        q_init (np.ndarray): The initial joint configuration vector (angles in radians or displacements
        ee_frame_name (str): The name of the end-effector frame for which to compute the IK.
        target_pose (pin.SE3): The desired end-effector pose as a Pinocchio SE3 object.
        max_iterations (int): Maximum number of iterations for the IK solver
        tolerance (float): The convergence tolerance for the error norm.
        max_step_size (float): The maximum allowed step size for joint updates.
        damping (float): The damping factor for the DLS method.
        verbose (bool): If True, prints debug information during the IK process.
    Returns:
        tuple[np.ndarray,bool]: (q_solution, converged)
    """
    q = q_init.copy()
    frame_id = model.getFrameId(ee_frame_name)

    for iteration in range(max_iterations):
        # FK + frame placements
        pin.forwardKinematics(model, data, q)
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)

        # Error: log6(current^{-1} * target) expresed in local frame
        current_pose = data.oMf[frame_id]
        # Compute error in SE3
        error_se3 = current_pose.actInv(target_pose)
        error_vector = pin.log6(error_se3).vector # shape (6,)

        error_norm = np.linalg.norm(error_vector)
        if verbose:
            print(f"Iteration {iteration:3d} | error norm: {error_norm:.6e}")

        if error_norm < tolerance:
            if verbose:
                print(f"Converged in {iteration} iterations with error norm {error_norm:.6e}")
            return q, True

        # Jacobian in LOCAL frame (consistent with log6 error above)
        J = pin.computeFrameJacobian(
            model, data, q, frame_id,
            pin.ReferenceFrame.LOCAL
        )

        #DLS: delta_q = (J^T * J + lambda^2 * I)^-1 * J^T * error
        JTJ = J.T @ J + (damping ** 2) * np.eye(model.nv)
        delta_q = np.linalg.solve(JTJ, J.T @ error_vector)

        # Clip step size 
        delta_q_norm = np.linalg.norm(delta_q)
        if delta_q_norm >max_step_size:
            delta_q *= max_step_size / delta_q_norm

        # Integrate on the manifold (correct for all joint types)
        q = pin.integrate(model, q, delta_q)

    if verbose:
        print(f"Maximum iterations ({max_iterations}) reached. Error: {error_norm:.6f}")
    return q, False