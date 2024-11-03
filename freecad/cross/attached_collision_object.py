# Stub for a CROSS::AttachedCollisionObject.

from __future__ import annotations

from typing import NewType, Optional

import FreeCAD as fc

# Typing hints
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
DO = fc.DocumentObject
AttachedCollisionObjectProxy = NewType('AttachedCollisionObjectProxy', object)
VPAttachedCollisionObjectProxy = NewType('VPAttachedCollisionObjectProxy', object)


class AttachedCollisionObject(DO):
    Link: CrossLink
    Objects: list[DO]
    Proxy: AttachedCollisionObjectProxy
    ViewObject: Optional[ViewProviderAttachedCollisionObject]
    _Type: str


class ViewProviderAttachedCollisionObject:
    Object: AttachedCollisionObject
    Proxy: VPAttachedCollisionObjectProxy
    Visibility: bool
