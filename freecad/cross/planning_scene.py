# Stub for a CROSS::PlanningScene.

from __future__ import annotations

from typing import ForwardRef

import FreeCAD as fc

# Typing hints
DO = fc.DocumentObject
VPPlanningSceneProxy = ForwardRef('VPPlanningSceneProxy')


class PlanningScene(DO):
    _Type: str


class ViewProviderPlanningScene:
    Object: PlanningScene
    PlaneSides: float
    Proxy: VPPlanningSceneProxy
    Visibility: bool

    def addDisplayMode(self, separator, mode: str) -> None: ...
