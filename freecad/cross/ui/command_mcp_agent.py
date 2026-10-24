"""Panel command to open the "MCP Agent" dialog.

The dialog shows the status of the RobotCAD MCP server and provides
ready-to-copy configuration for connecting an external LLM agent (VS Code,
Claude Desktop, ...) to the MCP server that exposes RobotCAD tools.
"""

from __future__ import annotations

import FreeCADGui as fcgui

from ..gui_utils import tr
from .mcp_agent_dialog import McpAgentDialog


class _McpAgentCommand:
    """The command definition to open the MCP agent dialog."""

    def GetResources(self):
        return {
            'Pixmap': 'mcp_agent.svg',
            'MenuText': tr('MCP Agent'),
            'ToolTip': tr(
                'Open the MCP agent dialog: manage the MCP server and copy '
                'configuration for connecting an external LLM agent.',
            ),
        }

    def IsActive(self):
        return True

    def Activated(self):
        dialog = McpAgentDialog(fcgui.getMainWindow())
        dialog.show()


fcgui.addCommand('MCPAgent', _McpAgentCommand())