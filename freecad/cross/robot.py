# Stub for a CROSS::Robot.

from __future__ import annotations

from typing import ForwardRef, List, Union

import FreeCAD as fc

# Implementation note: The following import is necessary to avoid a circular
# dependency.
JointRef = ForwardRef('Joint')
LinkRef = ForwardRef('Link')

# Typing hints
BasicElement = Union[LinkRef, JointRef]
DO = fc.DocumentObject
DOList = List[DO]


class Robot(DO):
    Group: list[BasicElement]
    OutputPath: str
    Placement: fc.Placement
    MaterialCardName: str
    MaterialCardPath: str
    MaterialDensity: str
    RobotType: dict
    _Type: str
    Mass: float

    def addObject(self, object_to_add: fc.DocumentObject) -> None: ...
    def removeObject(self, object_to_remove: fc.DocumentObject) -> DOList: ...
