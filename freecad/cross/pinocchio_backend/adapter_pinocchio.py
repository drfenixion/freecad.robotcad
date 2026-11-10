import pinocchio as pin
import numpy as np

def build_pin_model_from_robot(robot):
    """
    Build a Pinocchio model from a RoboTool Robot object.
    Reads the same chain that _build_robot_for_chain already constructed,
    so no FreeCAD objects are needed here.

    Parameters:
        robot (Robot): RoboTool robot instance (from freecad/cross/kinematics/models.py)

    Returns:
        Tuple: (pin.Model, pin.Data) Pinocchio model and data structures.
    """
    # Create a Pinocchio model
    model = pin.Model()
    parent_id = 0  # Start with the world as the parent

    for joint in robot.joints:
        # Build SE3 of the joint frame relative to its parent
        x, y, z = joint.origin_xyz
        roll, pitch, yaw = joint.origin_rpy
        R = pin.rpy.rpyToMatrix(roll, pitch, yaw)
        T = pin.SE3(R, np.array([x / 1000.0, y / 1000.0, z / 1000.0])) # mm to m

        # Select Pinocchio joint type
        if joint.joint_type in ('revolute', 'continuous'):
            #RoboTool always rotates around local Z (see _get_axis_from_placement)
            pin_joint = pin.JointModelPZ()
        elif joint.joint_type == 'prismatic':
            pin_joint = pin.JointModelPrismaticUnaligned(np.array([0, 0, 1]))
        else:
            # Fixed - skip, doesn't add DOF
            continue

        joint_id = model.addJoint(int(parent_id), pin_joint, T, joint.name)

        # Joint limits 
        if joint.limit_lower is not None and joint.limit_upper is not None:
            q_index = model.joints[joint_id].idx_q  # The index of the last added joint
            model.lowerPositionLimit[q_index] = joint.limit_lower
            model.upperPositionLimit[q_index] = joint.limit_upper

        # Add a frame for this joint's child link
        model.addFrame(
            pin.Frame(
                joint.child,
                joint_id,
                0,
                pin.SE3.Identity(),
                pin.FrameType.OP_FRAME,
            )
        )
        parent_id = int(joint_id)  # Update parent_id for the next joint

    data = model.createData()
    return model, data