"""MCP (Model Context Protocol) support for the RobotCAD workbench.

Provides an MCP server with tools that control the RobotCAD document:
creating robots, links, joints, collisions, positioning, rotation, materials,
mass/inertia calculation, object selection and 3D view snapshots.

Two transports are supported:

- **Streamable HTTP**: in-process server running in a background thread
  (``freecad.cross.mcp.server``), URL ``http://127.0.0.1:8006/mcp``.
- **stdio**: standalone bridge script ``freecad/cross/mcp/stdio_server.py``
  that connects to the HTTP server and re-exports the same tools over stdio
  (it must not import ``freecad.cross``, so it loads the pure-Python
  ``config`` and ``tool_schemas`` modules directly).

Shared defaults (host, port, endpoint path, environment variable names) live
in ``config.py`` — the single source of truth for both transports.
"""

from __future__ import annotations

from .config import (  # noqa: F401
    DEFAULT_HOST,
    DEFAULT_PORT,
    ENV_MCP_URL,
    MCP_ENDPOINT,
    PREF_MCP_SERVER_HOST,
    PREF_MCP_SERVER_PORT,
    default_server_url,
    get_mcp_url,
)
from .server import (  # noqa: F401
    McpServer,
    get_server,
    get_server_host,
    get_server_port,
    get_server_url,
    set_server_port,
)
from .tools_registry import get_tools  # noqa: F401

__all__ = [
    'DEFAULT_HOST',
    'DEFAULT_PORT',
    'ENV_MCP_URL',
    'MCP_ENDPOINT',
    'PREF_MCP_SERVER_HOST',
    'PREF_MCP_SERVER_PORT',
    'McpServer',
    'default_server_url',
    'get_mcp_url',
    'get_server',
    'get_server_host',
    'get_server_port',
    'get_server_url',
    'get_tools',
    'set_server_port',
]
