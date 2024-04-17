# Stub for a CROSS::PlanningScene.

from __future__ import annotations

from typing import ForwardRef

import FreeCAD as fc

# Typing hints
DO = fc.DocumentObject
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
VPPlanningSceneProxy = ForwardRef('VPPlanningSceneProxy')


class PlanningScene(DO):
    _Type: str


class ViewProviderPlanningScene:
    Object: PlanningScene
    PlaneSides: float
    Robot: CrossRobot
    Proxy: VPPlanningSceneProxy
    SubframeSize: float
    Visibility: bool

    # This is missing inFreeCAD's stubs.
    def addDisplayMode(self, separator, mode: str) -> None: ...
