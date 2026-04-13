# Stub for a CROSS::RgbCamera.

from __future__ import annotations

from typing import NewType, Optional

import FreeCAD as fc

# Typing hints
DO = fc.DocumentObject
RgbCameraProxy = NewType('RgbCameraProxy', object)
RgbCameraViewProxy = NewType('RgbCameraViewProxy', object)
Link = NewType('Link', DO)


class RgbCamera(DO):
    Proxy: RgbCameraProxy
    ViewObject: Optional[ViewProviderRgbCamera]
    _Type: str


class ViewProviderRgbCamera:
    Object: RgbCamera
    Proxy: RgbCameraViewProxy
    Visibility: bool
    Link: Link
