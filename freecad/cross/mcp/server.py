"""MCP server for the RobotCAD workbench.

Builds an ``MCPServer`` (mcp SDK 2.x, formerly FastMCP) with all tools from
``tools_registry`` and runs it over the Streamable HTTP transport in a
background thread inside the FreeCAD process. External agents (VS Code, Claude
Desktop, ...) connect to ``http://127.0.0.1:<port>/mcp``.
"""

from __future__ import annotations

import logging
import threading
from typing import Any, Optional

import FreeCAD as fc

from ..wb_utils import get_workbench_param, set_workbench_param
from .config import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    ENV_MCP_URL,
    MCP_ENDPOINT,
    PREF_MCP_SERVER_HOST,
    PREF_MCP_SERVER_PORT,
)

__all__ = [
    'DEFAULT_HOST',
    'DEFAULT_PORT',
    'ENV_MCP_URL',
    'MCP_ENDPOINT',
    'PREF_MCP_SERVER_HOST',
    'PREF_MCP_SERVER_PORT',
    'McpServer',
    'get_server',
    'get_server_host',
    'get_server_port',
    'get_server_url',
    'set_server_port',
]


def get_server_host() -> str:
    """Return the configured MCP server host."""
    return get_workbench_param(PREF_MCP_SERVER_HOST, DEFAULT_HOST)


def get_server_port() -> int:
    """Return the configured MCP server port."""
    return int(get_workbench_param(PREF_MCP_SERVER_PORT, DEFAULT_PORT))


def set_server_port(port: int) -> None:
    """Persist the MCP server port."""
    # Store as a string: set_param() calls .strip() on non-bool values,
    # and get_server_port() converts back with int().
    set_workbench_param(PREF_MCP_SERVER_PORT, str(int(port)))


def get_server_url() -> str:
    """Return the full MCP endpoint URL."""
    return f'http://{get_server_host()}:{get_server_port()}{MCP_ENDPOINT}'


class _FreeCADConsoleHandler(logging.Handler):
    """Logging handler that writes into the FreeCAD console.

    ``Console.PrintLog`` prints in the default (black) color, while
    ``Console.PrintError`` prints in red. uvicorn/mcp log records at INFO level
    must not look like errors, so they are routed here instead of stderr.
    """

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
            if record.levelno >= logging.ERROR:
                fc.Console.PrintError(msg + '\n')
            elif record.levelno >= logging.WARNING:
                fc.Console.PrintWarning(msg + '\n')
            else:
                # INFO and DEBUG: plain (black) console text.
                fc.Console.PrintLog(msg + '\n')
        except Exception:  # noqa: BLE001
            self.handleError(record)


def _configure_logging() -> None:
    """Route MCP/uvicorn logging into the FreeCAD console.

    By default the ``mcp`` and ``uvicorn`` loggers write to stderr, which the
    FreeCAD console renders in red. Attach a handler that prints INFO records
    in the default (black) color and keep the stderr handler away.
    """
    handler = _FreeCADConsoleHandler()
    handler.setFormatter(logging.Formatter('[%(name)s] %(levelname)s: %(message)s'))
    for logger_name in ('mcp', 'uvicorn', 'uvicorn.access', 'robotcad-mcp'):
        logger = logging.getLogger(logger_name)
        logger.handlers = [handler]
        logger.propagate = False
        if logger.level in (logging.NOTSET, 0):
            logger.setLevel(logging.INFO)


def _build_mcp_server():
    """Build and return a configured MCPServer instance with all tools."""
    from mcp.server import MCPServer

    from .tools_registry import get_tools

    mcp = MCPServer(
        'robotcad-mcp',
        instructions=(
            'Tools to control the RobotCAD (FreeCAD OVERCROSS) workbench: '
            'create robots, links, joints, collisions, set placements, '
            'rotate objects, set materials, calculate mass/inertia, '
            'select objects and capture 3D view snapshots.'
        ),
    )
    for name, fn in get_tools():
        mcp.tool(name=name)(fn)
    return mcp


def _ensure_mcp_packages() -> None:
    """Make sure the ``mcp`` package is installed, installing it on demand.

    The check was moved out of the workbench ``__init__`` so that the FreeCAD
    start-up does not pay the ``pip install`` cost: the package is installed
    lazily, the first time the MCP tools are used (server start). ``uvicorn``
    comes as a dependency of the official ``mcp`` SDK. Reuses the shared
    ``check_install_package`` helper (same code the workbench uses for its
    other pip dependencies).

    After a fresh install, pure-Python dependencies that FreeCAD/conda already
    imported (e.g. ``typing_extensions``) are reloaded from the
    AdditionalPythonPackages directory: otherwise the stale ``sys.modules``
    entry (old version without ``sentinel``) shadows the freshly installed
    copy and ``import mcp`` fails.
    """
    from ..packages import check_install_package, reload_pure_python_module

    check_install_package('mcp')
    # mcp 2.x requires typing_extensions >= 4.13 (provides `sentinel`).
    reload_pure_python_module('typing_extensions')


class McpServer:
    """Manages the lifecycle of the in-process HTTP MCP server."""

    def __init__(self) -> None:
        self._mcp: Any = None
        self._uvicorn_server: Any = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> str:
        """Start the HTTP MCP server in a background thread.

        Returns the server URL.
        """
        with self._lock:
            if self.is_running:
                return get_server_url()
            # Lazy install: the `mcp` package is installed on the first use
            # of the MCP tools (this call), not at workbench start.
            _ensure_mcp_packages()
            try:
                from mcp.server import MCPServer  # noqa: F401
                import uvicorn
            except ImportError as exc:
                # Diagnostics: show where the conflicting modules actually
                # resolve from, to make the root cause visible.
                import sys as _sys
                import typing_extensions as _te
                _diag = (
                    f'Import error: {exc}\n'
                    f'typing_extensions file: {getattr(_te, "__file__", "?")}\n'
                    f'typing_extensions has sentinel: {hasattr(_te, "sentinel")}\n'
                    f'packages path in sys.path: '
                    f'{[p for p in _sys.path if "AdditionalPythonPackages" in p]}\n'
                )
                raise RuntimeError(
                    'The "mcp" Python package could not be imported after '
                    'installation. Run: pip install mcp\n\n' + _diag,
                ) from exc

            self._mcp = _build_mcp_server()
            app = self._mcp.streamable_http_app()
            host = get_server_host()
            port = get_server_port()
            # INFO records of the MCP server must be printed in the default
            # (black) console color, not red: route the mcp/uvicorn loggers
            # into the FreeCAD console instead of stderr.
            _configure_logging()
            config = uvicorn.Config(
                app,
                host=host,
                port=port,
                log_level='info',
                # Do not let uvicorn install its own stderr handlers.
                log_config=None,
                access_log=False,
            )
            # uvicorn.Server.run() would install signal handlers, which is not
            # allowed from a non-main thread.
            config.install_signal_handlers = False
            self._uvicorn_server = uvicorn.Server(config)

            def _run() -> None:
                self._uvicorn_server.run()

            self._thread = threading.Thread(target=_run, name='robotcad-mcp-http', daemon=True)
            self._thread.start()
            return get_server_url()

    def stop(self) -> None:
        """Stop the HTTP MCP server."""
        with self._lock:
            if self._uvicorn_server is not None:
                self._uvicorn_server.should_exit = True
            if self._thread is not None:
                self._thread.join(timeout=5.0)
            self._thread = None
            self._uvicorn_server = None
            self._mcp = None

    def status(self) -> dict[str, Any]:
        """Return a status dictionary for the UI."""
        return {
            'running': self.is_running,
            'url': get_server_url(),
            'host': get_server_host(),
            'port': get_server_port(),
        }


#: Module-level singleton.
_server: Optional[McpServer] = None
_server_lock = threading.Lock()


def get_server() -> McpServer:
    """Return the module-level McpServer singleton."""
    global _server
    with _server_lock:
        if _server is None:
            _server = McpServer()
        return _server