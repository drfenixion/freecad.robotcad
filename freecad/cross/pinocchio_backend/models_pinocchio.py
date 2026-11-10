import pinocchio as pin

def load_model(urdf_path):
    """
    Load a Pinocchio model from a URDF file.
    Parameters:
    urdf_path (str): Path to the URDF file.
    Returns:
    pin.Model: The loaded Pinocchio model.
    """
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    return model, data

def load_model_from_xml_string(xml_string):
    """
    Load a Pinocchio model from an XML string.
    Parameters:
    xml_string (str): The XML string containing the model definition.
    Returns:
    pin.Model: The loaded Pinocchio model.
    """
    model = pin.buildModelFromXml(xml_string)
    data = model.createData()
    return model, data

def link_names(model):
    """
    Get the names of the links in a Pinocchio model.
    Parameters:
    model (pin.Model): The Pinocchio model.
    Returns:
    list: A list of link names.
    """
    return [model.names[i] for i in range(model.njoints)]

def joint_names(model):
    """Get joint names excluding universe joint at index 0."""
    return [model.names[i] for i in range(1, model.njoints)]

def joint_limits(model):
    """
    Get the joint limits of a Pinocchio model.
    Parameters:
    model (pin.Model): The Pinocchio model.
    Returns:
    tuple: A tuple containing two lists: lower limits and upper limits.
    """
    return model.lowerPositionLimit, model.upperPositionLimit

def link_names(model):
    """
    Get the names of the links in a Pinocchio model.
    Parameters:
    model (pin.Model): The Pinocchio model.
    Returns:
    list: A list of link names.
    """
    return [model.names[i] for i in range(model.njoints)]

def frame_names(model):
    """
    Get the names of the frames in a Pinocchio model.
    Parameters:
    model (pin.Model): The Pinocchio model.
    Returns:
    list: A list of frame names.
    """
    return [model.frames[i].name for i in range(model.nframes)]

def get_frame_id(model, frame_name):
    """
    Get the ID of a frame in a Pinocchio model by its name.
    Parameters:
    model (pin.Model): The Pinocchio model.
    frame_name (str): The name of the frame.
    Returns:
    int: The ID of the frame, or -1 if not found.
    """
    return model.getFrameId(frame_name)  # Returns -1 if not found

def get_joint_id(model, joint_name):
    """
    Get the ID of a joint in a Pinocchio model by its name.
    Parameters:
    model (pin.Model): The Pinocchio model.
    joint_name (str): The name of the joint.
    Returns:
    int: The ID of the joint, or -1 if not found.
    """
    return model.getJointId(joint_name)  # Returns -1 if not found

def get_joint_type(model, joint_name):
    """
    Get the type of a joint in a Pinocchio model by its name.
    Parameters:
    model (pin.Model): The Pinocchio model.
    joint_name (str): The name of the joint.
    Returns:
    str: The type of the joint, or None if not found.
    """
    joint_id = get_joint_id(model, joint_name)
    if joint_id < model.njoints:
        return model.joints[joint_id].shortname()
    return None

def pin_model (model):
    """
    Get the Pinocchio model.
    Parameters:
    model (pin.Model): The Pinocchio model.
    Returns:
    pin.Model: The Pinocchio model.
    """
    return model

def pin_data (model):
    """
    Get the Pinocchio data associated with the model.
    Parameters:
    model (pin.Model): The Pinocchio model.
    Returns:
    pin.Data: The Pinocchio data.
    """
    return model.createData()

def robot_dof(model):
    """
    Get the degrees of freedom (DoF) of a Pinocchio model.
    Parameters:
    model (pin.Model): The Pinocchio model.
    Returns:
    int: The number of degrees of freedom.
    """
    return model.nv