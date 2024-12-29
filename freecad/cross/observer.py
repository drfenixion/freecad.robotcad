# Stub for a CROSS::Observer.

from __future__ import annotations

from typing import NewType, Optional

import FreeCAD as fc

# Typing hints
DO = fc.DocumentObject
ObserverProxy = NewType('ObserverProxy', object)
VPObserverProxy = NewType('VPObserverProxy', object)


class Observer(DO):
    Formula: int
    Proxy: ObserverProxy
    ViewObject: Optional[ViewProviderObserver]
    _Type: str


class ViewProviderObserver:
    Object: Observer
    Proxy: VPObserverProxy
    Visibility: bool
