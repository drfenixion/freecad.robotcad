"""Dialog for the "MCP Agent" panel tool.

Shows the status of the in-process HTTP MCP server, allows starting/stopping
it, and provides ready-to-copy configuration snippets for connecting an
external agent (VS Code, Claude Desktop, ...) to the RobotCAD MCP server.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Optional

import FreeCAD as fc
import FreeCADGui as fcgui

try:
    from PySide import QtCore, QtGui, QtWidgets
except ImportError:
    from PySide6 import QtCore, QtGui, QtWidgets

from ..mcp import get_server, get_server_url, set_server_port
from ..mcp.tools_registry import get_tools
from ..wb_utils import MOD_PATH


def _python_exe() -> str:
    """Return the Python executable to use in the stdio config."""
    try:
        from addonmanager_utilities import get_python_exe
        return get_python_exe()
    except (ModuleNotFoundError, ImportError, AttributeError):
        try:
            from freecad.utils import get_python_exe
            return get_python_exe()
        except (ModuleNotFoundError, ImportError, AttributeError):
            return sys.executable


def _stdio_script() -> str:
    """Return the path to the standalone stdio bridge script."""
    return str(MOD_PATH / 'freecad' / 'cross' / 'mcp' / 'stdio_server.py')


class McpAgentDialog(QtWidgets.QDialog):
    """Dialog to manage the MCP server and show connection instructions."""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle('MCP Agent')
        self.setModal(False)
        self.resize(640, 560)
        self._server = get_server()
        self._setup_ui()
        self._refresh_status()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(10)
        layout.setContentsMargins(12, 12, 12, 12)

        # --- Status group -------------------------------------------------
        status_group = QtWidgets.QGroupBox('MCP server status')
        status_layout = QtWidgets.QVBoxLayout()

        self.status_label = QtWidgets.QLabel()
        status_layout.addWidget(self.status_label)

        port_row = QtWidgets.QHBoxLayout()
        port_row.addWidget(QtWidgets.QLabel('Port:'))
        self.port_spin = QtWidgets.QSpinBox()
        self.port_spin.setRange(1024, 65535)
        self.port_spin.setValue(self._server.status()['port'])
        port_row.addWidget(self.port_spin)
        port_row.addStretch(1)
        status_layout.addLayout(port_row)

        btn_row = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton('Start server')
        self.stop_btn = QtWidgets.QPushButton('Stop server')
        self.refresh_btn = QtWidgets.QPushButton('Refresh')
        btn_row.addWidget(self.start_btn)
        btn_row.addWidget(self.stop_btn)
        btn_row.addWidget(self.refresh_btn)
        btn_row.addStretch(1)
        status_layout.addLayout(btn_row)

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        # --- Instructions -------------------------------------------------
        instr_group = QtWidgets.QGroupBox('How to connect an external agent')
        instr_layout = QtWidgets.QVBoxLayout()

        self.instructions = QtWidgets.QTextEdit()
        self.instructions.setReadOnly(True)
        self.instructions.setMinimumHeight(220)
        instr_layout.addWidget(self.instructions)

        copy_row = QtWidgets.QHBoxLayout()
        self.copy_http_btn = QtWidgets.QPushButton('Copy HTTP config')
        self.copy_stdio_btn = QtWidgets.QPushButton('Copy stdio config')
        copy_row.addWidget(self.copy_http_btn)
        copy_row.addWidget(self.copy_stdio_btn)
        copy_row.addStretch(1)
        instr_layout.addLayout(copy_row)

        instr_group.setLayout(instr_layout)
        layout.addWidget(instr_group)

        # --- Tools list ---------------------------------------------------
        tools_group = QtWidgets.QGroupBox('Available MCP tools')
        tools_layout = QtWidgets.QVBoxLayout()
        self.tools_list = QtWidgets.QListWidget()
        for name, _ in get_tools():
            self.tools_list.addItem(name)
        tools_layout.addWidget(self.tools_list)
        tools_group.setLayout(tools_layout)
        layout.addWidget(tools_group)

        # --- Close --------------------------------------------------------
        close_row = QtWidgets.QHBoxLayout()
        close_row.addStretch(1)
        close_btn = QtWidgets.QPushButton('Close')
        close_btn.clicked.connect(self.close)
        close_row.addWidget(close_btn)
        layout.addLayout(close_row)

        self.setLayout(layout)

        # Connections.
        self.start_btn.clicked.connect(self._on_start)
        self.stop_btn.clicked.connect(self._on_stop)
        self.refresh_btn.clicked.connect(self._refresh_status)
        self.port_spin.valueChanged.connect(self._on_port_changed)
        self.copy_http_btn.clicked.connect(self._copy_http_config)
        self.copy_stdio_btn.clicked.connect(self._copy_stdio_config)

    # ------------------------------------------------------------------
    # Status / actions
    # ------------------------------------------------------------------

    def _refresh_status(self) -> None:
        status = self._server.status()
        if status['running']:
            self.status_label.setText(
                f'<b style="color:green">Running</b> — {status["url"]}',
            )
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
        else:
            self.status_label.setText(
                '<b style="color:red">Stopped</b> — start the server to allow '
                'external agents to connect.',
            )
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
        self._update_instructions()

    def _on_start(self) -> None:
        try:
            self._server.start()
        except Exception as exc:  # noqa: BLE001
            QtWidgets.QMessageBox.critical(self, 'MCP server', str(exc))
        self._refresh_status()

    def _on_stop(self) -> None:
        self._server.stop()
        self._refresh_status()

    def _on_port_changed(self, value: int) -> None:
        set_server_port(value)
        self._refresh_status()

    # ------------------------------------------------------------------
    # Config snippets
    # ------------------------------------------------------------------

    def _http_config(self) -> str:
        # Roo / Cline expect mcpServers to be an OBJECT keyed by server name,
        # and the HTTP transport type must be "streamable-http" (not "http").
        return json.dumps(
            {
                'mcpServers': {
                    'robotcad': {
                        'type': 'streamable-http',
                        'url': get_server_url(),
                        'disabled': False,
                    },
                },
            },
            indent=2,
        )

    def _stdio_config(self) -> str:
        # Roo / Cline expect mcpServers to be an OBJECT keyed by server name.
        return json.dumps(
            {
                'mcpServers': {
                    'robotcad': {
                        'type': 'stdio',
                        'command': _python_exe(),
                        'args': [_stdio_script()],
                        'env': {
                            'MCP_FREECAD_URL': get_server_url(),
                        },
                    },
                },
            },
            indent=2,
        )

    def _update_instructions(self) -> None:
        url = get_server_url()
        text = (
            '<h3>Connect from VS Code (Roo / Cline / Continue)</h3>'
            '<p>Add the following to <code>.mcp.json</code> in your workspace '
            '(or use the "Copy" buttons below):</p>'
            '<pre>' + self._http_config() + '</pre>'
            '<p>Or use the stdio transport (works even if the HTTP server is '
            'not reachable from the agent host):</p>'
            '<pre>' + self._stdio_config() + '</pre>'
            '<h3>Connect from Claude Desktop</h3>'
            '<p>Add the stdio entry above to '
            '<code>claude_desktop_config.json</code>.</p>'
            '<h3>Notes</h3>'
            '<ul>'
            f'<li>The HTTP server listens on <code>{url}</code>.</li>'
            '<li>Keep FreeCAD running while the agent uses the tools.</li>'
            '<li>Tools operate on the active document by object name '
            '(Name or Label).</li>'
            '</ul>'
        )
        self.instructions.setHtml(text)

    def _copy_http_config(self) -> None:
        QtWidgets.QApplication.clipboard().setText(self._http_config())
        self._flash('HTTP config copied to clipboard.')

    def _copy_stdio_config(self) -> None:
        QtWidgets.QApplication.clipboard().setText(self._stdio_config())
        self._flash('stdio config copied to clipboard.')

    def _flash(self, message: str) -> None:
        self.status_label.setText(message)
        QtCore.QTimer.singleShot(2500, self._refresh_status)