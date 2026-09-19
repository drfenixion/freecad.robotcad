"""Qt bridge to run FreeCAD operations on the main GUI thread.

The MCP HTTP server runs in a background thread. FreeCAD's document and GUI
APIs (``fc.activeDocument()``, ``fcgui.Selection``, ...) must only be called
from the main GUI thread. This module provides ``run_on_main_thread()`` which
schedules a callable on the main thread via a QObject signal/slot (queued
connection) and blocks the calling (background) thread until the result is
available.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional

import FreeCAD as fc

try:
    from PySide import QtCore
except ImportError:
    from PySide6 import QtCore


class _MainThreadInvoker(QtCore.QObject):
    """QObject living in the main thread that executes callables."""

    _request = QtCore.Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self._request.connect(self._execute)

    def _execute(self, payload: dict) -> None:
        """Run the callable and store the result (main thread)."""
        fn = payload['fn']
        args = payload['args']
        kwargs = payload['kwargs']
        result_holder = payload['result_holder']
        try:
            result_holder['result'] = fn(*args, **kwargs)
            result_holder['error'] = None
        except Exception as exc:  # noqa: BLE001 - propagate to caller thread.
            result_holder['result'] = None
            result_holder['error'] = exc
        result_holder['done'] = True


#: The invoker must live in the main thread. It is created eagerly at import
#: time, which happens on the main thread when the workbench loads.
_invoker: Optional[_MainThreadInvoker] = None

if getattr(fc, 'GuiUp', False):
    _invoker = _MainThreadInvoker()


def _ensure_invoker() -> _MainThreadInvoker:
    """Return the singleton invoker (created on the main thread at import)."""
    global _invoker
    if _invoker is None:
        # Fallback: create it now. This is only safe if called from the main
        # thread; in practice the eager creation above covers the GUI case.
        _invoker = _MainThreadInvoker()
    return _invoker


def run_on_main_thread(
    fn: Callable[..., Any],
    *args: Any,
    timeout: float = 300.0,
    **kwargs: Any,
) -> Any:
    """Run ``fn(*args, **kwargs)`` on the main GUI thread and return its result.

    If the current thread is the main thread, ``fn`` is called directly.
    Otherwise the callable is queued to the main thread and this thread blocks
    (with a small sleep) until the result is available.

    Raises:
        RuntimeError: If the callable raised an exception (re-raised here) or
            if the timeout is exceeded.
    """
    if not getattr(fc, 'GuiUp', False):
        # No GUI: run directly.
        return fn(*args, **kwargs)

    import FreeCADGui as fcgui

    main_window = fcgui.getMainWindow()
    if main_window is None:
        # GUI not fully initialized yet.
        return fn(*args, **kwargs)
    main_thread = main_window.thread()
    current_thread = QtCore.QThread.currentThread()
    if current_thread is main_thread:
        return fn(*args, **kwargs)

    invoker = _ensure_invoker()
    result_holder: dict[str, Any] = {
        'done': False,
        'result': None,
        'error': None,
    }
    payload = {
        'fn': fn,
        'args': args,
        'kwargs': kwargs,
        'result_holder': result_holder,
    }
    # Emit from the background thread; the queued connection delivers the
    # signal to the main thread's event loop.
    invoker._request.emit(payload)

    deadline = time.monotonic() + timeout
    while not result_holder['done']:
        if time.monotonic() > deadline:
            raise RuntimeError(
                f'Timeout waiting for main-thread execution of {getattr(fn, "__name__", fn)}',
            )
        time.sleep(0.01)

    if result_holder['error'] is not None:
        raise result_holder['error']
    return result_holder['result']