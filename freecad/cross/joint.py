# Stub for a CROSS::Joint.

from __future__ import annotations

from typing import NewType

import FreeCAD as fc

# Implementation note: These cannot be imported because of circular
# dependency.
JointProxy = NewType('JointProxy', object)
VPJointProxy = NewType('VPJointProxy', object)


class Joint(fc.DocumentObject):
    Group: list[fc.DocumentObject]
    Child: str  # Must name a CROSS::Link by its ROS name.
    Effort: float
    LowerLimit: float
    Mimic: bool
    MimickedJoint: Joint
    Multiplier: float
    Offset: float
    Origin: fc.Placement
    Parent: str  # Must name a CROSS::Link by its ROS name.
    Placement: fc.Placement
    PlacementRelTotalCenterOfMass: fc.Placement
    Position: float
    Proxy: JointProxy
    Type: str
    UpperLimit: float
    Velocity: float
    JointSpecific: dict
    JoinRotationDirection: dict
    _Type: str


class ViewProviderJoint:
    AxisLength: float
    Object: Joint
    Proxy: VPJointProxy
    Visibility: bool

    def addDisplayMode(self, separator, mode: str) -> None: ...
