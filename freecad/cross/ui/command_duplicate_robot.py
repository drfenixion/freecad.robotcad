# Duplicate a Cross::Robot, its links and joints.
#
# The `Collision`, `Visual`, and `Real` parts of the new links point to the same
# objects as the original links (i.e. the linked objects are not duplicated).

import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!

# Typing hints.
from freecad.cross.robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501


class _DuplicateRobotCommand:
    """The command definition to duplicate a Robot object."""

    def GetResources(self):
        return {
            'Pixmap': 'bulb',
            'MenuText': 'Duplicate Robot',
            'Accel': 'D, R',
            'ToolTip': 'Duplicate a CROSS::Robot',
        }

    def IsActive(self):
        # Import late to avoid slowing down workbench start-up.
        from ..wb_utils import is_robot

        objs = fcgui.Selection.getSelection()
        if not objs:
            return False
        if is_robot(objs[0]):
            return True
        return False

    def Activated(self):
        # Import late to avoid slowing down workbench start-up.
        from ..freecad_utils import validate_types
        from ..freecad_utils import warn
        from .duplicate_robot_dialog import DuplicateRobotDialog

        sel = fcgui.Selection.getSelection()
        if not validate_types(sel, ['Cross::Robot']):
            warn('Please select a Cross::Robot', True)
            return

        form = DuplicateRobotDialog(sel[0].Label)

        form.exec()
        for _ in range(form.number_of_duplicates):
            duplicate(sel[0], form.base_name)


def duplicate(orig_robot: CrossRobot, base_name: str) -> CrossRobot:
    """Duplicate a Cross::Robot, its links and joints."""
    # Import late to avoid slowing down workbench start-up.
    from freecad.cross.joint_proxy import make_joint
    from freecad.cross.link_proxy import make_link
    from freecad.cross.robot_proxy import make_robot

    doc = orig_robot.Document
    robot = make_robot(f'{base_name}_000', doc)

    orig_proxy = orig_robot.Proxy

    for orig_link in orig_proxy.get_links():
        link = make_link(orig_link.Label, doc)
        robot.addObject(link)
        link.CenterOfMass = orig_link.CenterOfMass
        link.Collision = orig_link.Collision
        link.Ixx = orig_link.Ixx
        link.Ixy = orig_link.Ixy
        link.Ixz = orig_link.Ixz
        link.Iyy = orig_link.Iyy
        link.Iyz = orig_link.Iyz
        link.Izz = orig_link.Izz
        link.Label2 = orig_link.Label2
        link.Mass = orig_link.Mass
        link.MaterialCardName = orig_link.MaterialCardName
        link.MaterialCardPath = orig_link.MaterialCardPath
        link.MaterialDensity = orig_link.MaterialDensity
        link.MaterialNotCalculate = orig_link.MaterialNotCalculate
        link.MountedPlacement = orig_link.MountedPlacement
        link.Real = orig_link.Real
        link.Visual = orig_link.Visual
        link.ViewObject.DisplayMode = orig_link.ViewObject.DisplayMode
        link.ViewObject.ShowCollision = orig_link.ViewObject.ShowCollision
        link.ViewObject.ShowReal = orig_link.ViewObject.ShowReal
        link.ViewObject.ShowVisual = orig_link.ViewObject.ShowVisual

    doc.recompute()

    for orig_joint in orig_proxy.get_joints():
        joint = make_joint(orig_joint.Label, doc)
        robot.addObject(joint)
        joint.Child = orig_joint.Child
        joint.Effort = orig_joint.Effort
        joint.Label2 = orig_joint.Label2
        joint.LowerLimit = orig_joint.LowerLimit
        joint.Mimic = orig_joint.Mimic
        joint.MimickedJoint = orig_joint.MimickedJoint
        joint.Multiplier = orig_joint.Multiplier
        joint.Offset = orig_joint.Offset
        joint.Origin = orig_joint.Origin
        joint.Parent = orig_joint.Parent
        joint.Placement = orig_joint.Placement
        joint.Position = orig_joint.Position
        joint.Type = orig_joint.Type
        joint.UpperLimit = orig_joint.UpperLimit
        joint.Velocity = orig_joint.Velocity
        joint.ViewObject.AxisLength = orig_joint.ViewObject.AxisLength
        joint.ViewObject.DisplayMode = orig_joint.ViewObject.DisplayMode
        joint.ViewObject.ShowAxis = orig_joint.ViewObject.ShowAxis

    doc.recompute()

    robot.MaterialCardName = orig_robot.MaterialCardName
    robot.MaterialCardPath = orig_robot.MaterialCardPath
    robot.MaterialDensity = orig_robot.MaterialDensity
    robot.OutputPath = orig_robot.OutputPath
    robot.Placement = orig_robot.Placement
    for joint_variable_name in orig_proxy.joint_variables.values():
        setattr(robot, joint_variable_name, getattr(orig_robot, joint_variable_name))
    robot.ViewObject.DisplayMode = orig_robot.ViewObject.DisplayMode
    robot.ViewObject.JointAxisLength = orig_robot.ViewObject.JointAxisLength
    robot.ViewObject.ShowCollision = orig_robot.ViewObject.ShowCollision
    robot.ViewObject.ShowJointAxes = orig_robot.ViewObject.ShowJointAxes
    robot.ViewObject.ShowReal = orig_robot.ViewObject.ShowReal
    robot.ViewObject.ShowVisual = orig_robot.ViewObject.ShowVisual

    doc.recompute()
    return robot


fcgui.addCommand('DuplicateRobot', _DuplicateRobotCommand())
