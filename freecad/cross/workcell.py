# Stub for a CROSS::Robot.

from __future__ import annotations

from typing import NewType, Union

import FreeCAD as fc

# Implementation note: The following import is necessary to avoid a circular
# dependency.
Joint = NewType('Joint', fc.DocumentObject)
XacroObject = NewType('XacroObject', fc.DocumentObject)

JointOrXacroObject = Union[Joint, XacroObject]


class Workcell(fc.DocumentObject):
    Group: list[JointOrXacroObject]
    OutputPath: str
    RootLink: str
    _Type: str
