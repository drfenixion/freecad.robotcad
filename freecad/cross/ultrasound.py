# Stub for a CROSS::Ultrasound.

from __future__ import annotations

from typing import NewType, Optional

import FreeCAD as fc

# Typing hints
DO = fc.DocumentObject
UltrasoundProxy = NewType('UltrasoundProxy', object)
UltrasoundViewProxy = NewType('UltrasoundPViewroxy', object)
Link = NewType('Link', DO)


class Ultrasound(DO):
    Proxy: UltrasoundProxy
    ViewObject: Optional[UltrasoundViewProxy]
    _Type: str


class ViewProviderUltrasound:
    Object: UltrasoundProxy
    Proxy: UltrasoundViewProxy
    Visibility: bool
    Link: Link
