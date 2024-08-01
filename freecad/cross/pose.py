# Stub for a CROSS::Pose.

from __future__ import annotations

from typing import NewType, Optional, Union

import FreeCAD as fc

# Typing hints
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
DO = fc.DocumentObject
PoseProxy = NewType('PoseProxy', object)
VPPoseProxy = NewType('VPPoseProxy', object)


class Pose(DO):
    AllowNonLeafLink: bool
    EndEffector: Union[str, list[str]]  # Both an enum and a string.
    Placement: fc.Placement
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
