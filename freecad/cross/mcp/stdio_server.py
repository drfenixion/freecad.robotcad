#!/usr/bin/env python3
"""stdio MCP server bridge for the RobotCAD workbench.

Standalone entry point for external agents: exposes the same tools as the
in-process HTTP MCP server (running inside FreeCAD) over the stdio transport.
It connects to the FreeCAD HTTP MCP server as a client and re-exports the
tools, so external agents can use:

    "mcpServers": {
        "robotcad": {
            "type": "stdio",
            "command": "<python-exe>",
            "args": ["<path-to>/freecad/cross/mcp/stdio_server.py"],
            "env": {"MCP_FREECAD_URL": "http://127.0.0.1:8006/mcp"}
        }
    }

The FreeCAD application must be running with the HTTP MCP server started
(use the "MCP Agent" panel tool).

IMPORTANT: This script must NOT import ``freecad.cross`` (the package
``__init__``) because that triggers the heavy workbench initialization
(FreeCAD, ROS, debugger attach). It loads the pure-Python modules
``config`` and ``tool_schemas`` from this directory, and the shared
``packages`` helper — without importing the ``freecad.cross`` package.

Defaults (host, port, endpoint, env variable name) are shared with the
in-process server via ``config.py`` — the single source of truth.
"""

from __future__ import annotations

import asyncio
import importlib.util
import sys
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# Import the pure-Python sibling modules (config, tool_schemas, packages)
# without importing the `freecad.cross` package. Works both as a standalone
# script (plain `import config`) and as a package module (relative imports).
# ---------------------------------------------------------------------------

_HERE = Path(__file__).resolve().parent

try:  # Package mode (e.g. `python -m freecad.cross.mcp.stdio_server`).
    from ..packages import check_install_package, reload_pure_python_module
    from .config import get_mcp_url
    from .tool_schemas import get_tool_schemas
except ImportError:  # Standalone script mode.
    if str(_HERE) not in sys.path:
        sys.path.insert(0, str(_HERE))
    from config import get_mcp_url
    from tool_schemas import get_tool_schemas

    # Load `packages.py` (one directory up) via importlib under a unique
    # module name. We must NOT put `freecad/cross/` on sys.path: our own
    # `mcp/` package would shadow the pip-installed `mcp` SDK.
    _packages_spec = importlib.util.spec_from_file_location(
        '_robotcad_packages', _HERE.parent / 'packages.py',
    )
    _packages = importlib.util.module_from_spec(_packages_spec)
    _packages_spec.loader.exec_module(_packages)
    check_install_package = _packages.check_install_package
    reload_pure_python_module = _packages.reload_pure_python_module


def _ensure_mcp_packages() -> None:
    """Make the ``mcp`` package importable, installing it on demand.

    Reuses ``check_install_package`` from ``freecad.cross.packages`` — the
    same shared helper the workbench uses. It adds FreeCAD's
    AdditionalPythonPackages directory to ``sys.path`` (packages installed
    by the workbench live there) and pip-installs ``mcp`` into it if the
    package is missing.

    After a fresh install, pure-Python dependencies that the base environment
    also provides (e.g. ``typing_extensions``) are pre-loaded from the
    AdditionalPythonPackages directory: otherwise the conda copy (older,
    without ``sentinel``) would win on the subsequent ``import mcp``.
    """
    check_install_package('mcp')
    # mcp 2.x requires typing_extensions >= 4.13 (provides `sentinel`).
    reload_pure_python_module('typing_extensions')


# Ensure the `mcp` package is importable before the bridge connects: the HTTP
# server must be running in FreeCAD anyway, but the SDK must exist in this
# process too.
_ensure_mcp_packages()


class _RemoteClient:
    """Lazily-created MCP client session to the FreeCAD HTTP server."""

    def __init__(self, url: str) -> None:
        self.url = url
        self._session: Any = None
        self._ctx: Any = None

    async def _ensure(self) -> Any:
        if self._session is None:
            from mcp import ClientSession
            from mcp.client.streamable_http import streamable_http_client

            self._ctx = streamable_http_client(self.url)
            read, write = await self._ctx.__aenter__()
            self._session = await ClientSession(read, write).__aenter__()
            # mcp 2.x requires an explicit initialize() handshake.
            await self._session.initialize()
        return self._session

    async def call_tool(self, name: str, arguments: dict[str, Any]) -> str:
        session = await self._ensure()
        result = await session.call_tool(name, arguments)
        if result.is_error:
            texts = [c.text for c in result.content if hasattr(c, 'text')]
            raise RuntimeError('Remote error: ' + '\n'.join(texts))
        texts = [c.text for c in result.content if hasattr(c, 'text')]
        return '\n'.join(texts)

    async def close(self) -> None:
        if self._session is not None:
            await self._session.__aexit__(None, None, None)
            self._session = None
        if self._ctx is not None:
            await self._ctx.__aexit__(None, None, None)
            self._ctx = None


def _build_stdio_server():
    """Build a low-level MCP Server that proxies to the FreeCAD HTTP server."""
    from mcp.server import Server
    from mcp.types import TextContent, Tool

    url = get_mcp_url()
    client = _RemoteClient(url)
    schemas = get_tool_schemas()

    server = Server('robotcad-stdio-bridge')

    @server.list_tools()
    async def on_list_tools() -> list[Tool]:
        tools = []
        for name, schema in schemas.items():
            tools.append(
                Tool(
                    name=name,
                    description=f'RobotCAD tool: {name} (proxied to {url})',
                    inputSchema={
                        'type': 'object',
                        'properties': schema.get('properties', {}),
                        'required': schema.get('required', []),
                    },
                ),
            )
        return tools

    @server.call_tool()
    async def on_call_tool(name: str, arguments: dict) -> list[TextContent]:
        if name not in schemas:
            raise RuntimeError(f'Unknown tool: {name}')
        text = await client.call_tool(name, arguments)
        return [TextContent(type='text', text=text)]

    return server, client


async def _run() -> None:
    from mcp.server.stdio import stdio_server

    server, client = _build_stdio_server()
    try:
        async with stdio_server() as (read_stream, write_stream):
            await server.run(
                read_stream,
                write_stream,
                server.create_initialization_options(),
            )
    finally:
        await client.close()


def main() -> None:
    """Run the stdio bridge server."""
    asyncio.run(_run())


if __name__ == '__main__':
    main()
