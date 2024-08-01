# Stub for a CROSS::XacroObject.

from __future__ import annotations

from typing import List

import FreeCAD as fc

# Typing hints
DO = fc.DocumentObject
DOList = List[DO]


class XacroObject(fc.DocumentObject):
    Group: list[fc.DocumentObject]
    InputFile: str
    MainMacro: str
    Placement: fc.Placement
    _Type: str

    def addObject(self, object_to_add: fc.DocumentObject) -> None: ...
    def removeObject(self, object_to_remove: fc.DocumentObject) -> DOList: ...
