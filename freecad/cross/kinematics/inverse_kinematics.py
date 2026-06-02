import numpy as np
from .kinematics import compute_forward_kinematics_full
from .jacobians import compute_jacobian
from .models import Robot

def inverse_kinematics(robot, target_pos, initial_guess, max_iterations=1000, tolerance=1e-4, max_restarts=5):
    
    for restart in range(max_restarts):
        if restart == 0:
            q = initial_guess.copy()
        else:
            # Perturbación aleatoria para escapar de singularidades
            q = initial_guess.copy()
            q += np.random.uniform(-0.3, 0.3, size=len(q))
            print(f"IK restart {restart} with perturbed q={np.round(q, 3)}")

        revolute_mask = np.array([
            joint.joint_type in ["revolute", "continuous"]
            for joint in robot.joints
            if joint.joint_type != "fixed"
        ], dtype=bool)

        prev_error_norm = float('inf')
        stall_count = 0

        for i in range(max_iterations):
            all_transforms = compute_forward_kinematics_full(robot, q)
            current_pos = all_transforms[-1][:3, 3]
            error = target_pos - current_pos
            error_norm = np.linalg.norm(error)

            if error_norm < tolerance:
                print(f"Converged in {i} iterations (restart {restart}).")
                return q

            if abs(prev_error_norm - error_norm) < 1e-10:
                stall_count += 1
            else:
                stall_count = 0
            prev_error_norm = error_norm

            if stall_count > 30:
                print(f"Stalled at iter {i}, error={error_norm:.6f}. Trying restart.")
                break  # exit the iteration loop to trigger a restart

            J_full = compute_jacobian(robot, q)
            J_v = J_full[:3, :]

            lam = 0.01
            A = J_v @ J_v.T + (lam ** 2) * np.eye(3)
            delta_q = J_v.T @ np.linalg.solve(A, error)

            step_norm = np.linalg.norm(delta_q)
            if step_norm > 0.1:
                delta_q = delta_q * (0.1 / step_norm)

            q += delta_q
            q[revolute_mask] = np.mod(q[revolute_mask] + np.pi, 2 * np.pi) - np.pi

    print(f"IK did not converge after {max_restarts} restarts.")
    return None