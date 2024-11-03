# Stub for a CROSS::Trajectory.

from __future__ import annotations

from typing import NewType, Optional

import FreeCAD as fc

# Typing hints
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
DO = fc.DocumentObject
TrajectoryProxy = NewType('TrajectoryProxy', object)
VPTrajectoryProxy = NewType('VPTrajectoryProxy', object)


class Trajectory(DO):
    PointIndex: int
    Proxy: TrajectoryProxy
    Robot: CrossRobot
    ViewObject: Optional[ViewProviderTrajectory]
    _Type: str


class ViewProviderTrajectory:
    Object: Trajectory
    Proxy: VPTrajectoryProxy
    Visibility: bool
