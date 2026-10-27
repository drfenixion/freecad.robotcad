"""Anticipated MCP tool errors.

The MCP SDK distinguishes two kinds of tool failure:

- a *deliberate* failure (``ToolError``): the model receives the message and
  the server logs it at INFO, **without** a traceback;
- an unexpected crash (any other exception): the model only sees
  ``Error executing tool <name>`` and the server logs a full traceback at
  ERROR, which is what pollutes the FreeCAD console.

The workbench raises ``RuntimeError`` for expected, user-correctable problems
(wrong object type, bad reference, missing joint, ...). Converting those to
``ToolError`` keeps the console clean and hands the actual reason to the agent
so it can correct the call.

This module must stay dependency-free (no FreeCAD, no workbench imports) so it
can be imported from both ``main_thread`` and ``tools_registry``.
"""

from __future__ import annotations

from typing import Any, Optional

#: Cached SDK ``ToolError`` class (``None`` until it has been resolved).
_tool_error_class: Optional[type[Exception]] = None


def tool_error_class() -> type[Exception]:
    """Return the SDK ``ToolError`` class, or ``RuntimeError`` as a fallback.

    The ``mcp`` package is installed lazily (on the first server start), so the
    import is done here rather than at module import time. If the package is
    missing or its layout differs, ``RuntimeError`` is used: the behaviour is
    then the previous one, never a hard failure. A successful resolution is
    cached; a failure is retried on the next call (the package may be
    installed in the meantime).
    """
    global _tool_error_class
    if _tool_error_class is not None:
        return _tool_error_class
    try:
        from mcp.server.mcpserver.exceptions import ToolError
    except Exception:  # noqa: BLE001 - mcp not installed / different layout.
        return RuntimeError
    _tool_error_class = ToolError
    return ToolError


def make_tool_error(message: str) -> Exception:
    """Build an anticipated tool error carrying ``message``."""
    return tool_error_class()(message)


def is_tool_error(exc: BaseException) -> bool:
    """Return True if ``exc`` is already an anticipated tool error."""
    return isinstance(exc, tool_error_class())


def enrich_error(exc: BaseException, messages: list[str]) -> Exception:
    """Return ``exc`` enriched with the recorded workbench messages.

    The exception class is preserved for anticipated tool errors, so a
    ``ToolError`` stays a ``ToolError`` (INFO log, message to the model)
    instead of degrading into a crash.
    """
    text = f'{exc}\n' + '\n'.join(messages)
    if is_tool_error(exc):
        return make_tool_error(text)
    return RuntimeError(text)


def as_anticipated_error(fn: Any) -> Any:
    """Wrap a tool so expected failures reach the agent as ``ToolError``.

    ``RuntimeError``/``ValueError`` are the workbench's way of reporting a
    problem the caller can fix (wrong object type, invalid reference, ...).
    They are converted to the SDK's ``ToolError``; anything else is left
    untouched and still surfaces as a genuine crash.

    ``functools.wraps`` keeps ``__wrapped__``, so ``inspect.signature`` (used by
    the SDK to build the tool schema) still sees the original signature.
    """
    import functools

    @functools.wraps(fn)
    def _wrapper(*args: Any, **kwargs: Any) -> Any:
        try:
            return fn(*args, **kwargs)
        except Exception as exc:  # noqa: BLE001 - re-raised below.
            if is_tool_error(exc):
                raise
            if isinstance(exc, (RuntimeError, ValueError)):
                raise make_tool_error(str(exc)) from exc
            raise

    return _wrapper
