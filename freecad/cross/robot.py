# Stub for a CROSS::Robot.

from __future__ import annotations

from typing import NewType, List, Union

import FreeCAD as fc

# Typing hints
DO = fc.DocumentObject
# Implementation note: These cannot be imported because of circular
# dependency.
Joint = NewType('Joint', DO)
Link = NewType('Link', DO)
Controller = NewType('Controller', DO)
AttachedCollisionObject = NewType('AttachedCollisionObject', DO)

BasicElement = Union[AttachedCollisionObject, Joint, Link, Controller]
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
