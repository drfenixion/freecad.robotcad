"""Centralized configuration for the RobotCAD MCP server.

Single source of truth for the default host/port, endpoint path, preference
keys and the stdio-bridge environment variable. Pure Python: this module must
NOT import FreeCAD, so that the standalone stdio bridge (``stdio_server.py``)
can load it without initializing the workbench.
"""

from __future__ import annotations

import os

# Preference keys (stored in the FreeCAD workbench parameters).
PREF_MCP_SERVER_PORT = 'mcp_server_port'
PREF_MCP_SERVER_HOST = 'mcp_server_host'

#: Environment variable with the FreeCAD HTTP MCP endpoint URL, read by the
#: stdio bridge (falls back to ``default_server_url()``).
ENV_MCP_URL = 'MCP_FREECAD_URL'

#: Default HTTP server settings.
DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 8006

#: Path suffix of the streamable-http MCP endpoint.
MCP_ENDPOINT = '/mcp'


def default_server_url() -> str:
    """Return the default (unconfigured) MCP endpoint URL."""
    return f'http://{DEFAULT_HOST}:{DEFAULT_PORT}{MCP_ENDPOINT}'


def get_mcp_url() -> str:
    """Return the FreeCAD MCP endpoint URL: env override or the default."""
    return os.environ.get(ENV_MCP_URL, default_server_url())


