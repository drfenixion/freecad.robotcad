"""Collect messages that must be reported when the workbench is activated.

During FreeCAD start-up the RobotCAD workbench is loaded, but the user has not
switched to it yet. Printing dependency/disabled-tool warnings at that time
clutters the console before the workbench is actually used. Instead, those
messages are collected here (via :func:`add_message`) and flushed to the
FreeCAD console on workbench activation (via :func:`flush`).

This module must not import FreeCAD at module level: it is used from
``freecad/cross/__init__.py`` (imported early) as well as from the GUI.
"""

from __future__ import annotations

_PENDING: list[str] = []


def add_message(text: str) -> None:
    """Queue a message to be reported on workbench activation."""
    if text:
        _PENDING.append(text)


def has_pending() -> bool:
    """Return True if there are queued messages."""
    return bool(_PENDING)


def flush(printer=None) -> list[str]:
    """Report and clear all queued messages.

    ``printer`` is a callable that prints a line; when omitted, the FreeCAD
    console is used (falling back to ``print``).

    Returns the messages that were flushed.
    """
    if printer is None:
        def printer(line: str) -> None:  # type: ignore[no-redef]
            try:
                import FreeCAD as fc
                fc.Console.PrintWarning(line + '\n')
            except Exception:
                print(line)

    messages = list(_PENDING)
    _PENDING.clear()
    for line in messages:
        printer(line)
    return messages
