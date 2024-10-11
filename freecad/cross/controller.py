# Stub for a CROSS::Robot.

from __future__ import annotations

from typing import List

import FreeCAD as fc

DO = fc.DocumentObject
DOList = List[DO]


class Controller(DO):
    _Type: str

    def addObject(self, object_to_add: fc.DocumentObject) -> None: ...
    def removeObject(self, object_to_remove: fc.DocumentObject) -> DOList: ...
