# Stub for a CROSS::Joint.

from __future__ import annotations

from typing import ForwardRef

import FreeCAD as fc

# Implementation note: The following import is necessary to avoid a circular
# dependency.
JointProxy = '.joint_proxy.JointProxy'
JointRef = ForwardRef('Joint')
VPJointProxy = ForwardRef('VPJointProxy')


class Joint(fc.DocumentObject):
    Child: str  # Must name a CROSS::Link by its ROS name.
    Effort: float
    LowerLimit: float
    Mimic: JointRef
    MimickedJoint: JointRef
    Multiplier: float
    Offset: float
    Origin: fc.Placement
    Parent: str  # Must name a CROSS::Link by its ROS name.
    Placement: fc.Placement
    Position: float
    Proxy: JointProxy
    Type: str
    UpperLimit: float
    Velocity: float
    _Type: str


class ViewProviderJoint:
    AxisLength: float
    Object: Joint
    Proxy: VPJointProxy
    Visibility: bool

    def addDisplayMode(self, separator, mode: str) -> None: ...
