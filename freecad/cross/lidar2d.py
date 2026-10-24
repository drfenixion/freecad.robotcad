# Stub for a CROSS::Lidar2d.

from __future__ import annotations

from typing import NewType, Optional

import FreeCAD as fc

# Typing hints
DO = fc.DocumentObject
Lidar2dProxy = NewType('Lidar2dProxy', object)
Lidar2dViewProxy = NewType('Lidar2dPViewroxy', object)
Link = NewType('Link', DO)


class Lidar2d(DO):
    Proxy: Lidar2dProxy
    ViewObject: Optional[Lidar2dViewProxy]
    _Type: str


class ViewProviderLidar2d:
    Object: Lidar2dProxy
    Proxy: Lidar2dViewProxy
    Visibility: bool
    Link: Link
