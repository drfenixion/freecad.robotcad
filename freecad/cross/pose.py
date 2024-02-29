# Stub for a CROSS::Pose.

from __future__ import annotations

from typing import ForwardRef, List, Optional, Union

import FreeCAD as fc

# Typing hints
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
DO = fc.DocumentObject
PoseProxy = '.pose_proxy.PoseProxy'
VPPoseProxy = ForwardRef('VPPoseProxy')


class Pose(DO):
    AllowNonLeafLink: bool
    EndEffector: Union[str, list[str]]  # Both an enum and a string.
    Pose: fc.Placement
    Proxy: PoseProxy
    Robot: CrossRobot
    ViewObject: Optional[ViewProviderPose]
    _Type: str


class ViewProviderPose:
    AxisLength: float
    Object: Pose
    Proxy: VPPoseProxy
    ShowEndEffector: bool
    Visibility: bool

    def addDisplayMode(self, separator, mode: str) -> None: ...
