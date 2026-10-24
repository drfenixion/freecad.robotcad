"""Command to show the RobotCAD version (read from package.xml)."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import FreeCAD as fc
import FreeCADGui as fcgui

try:
    from PySide import QtCore, QtWidgets
except ImportError:
    from PySide6 import QtCore, QtWidgets

from ..gui_utils import tr
from ..wb_constants import MOD_PATH
from .dependencies_dialog import open_dependencies_dialog


def _get_package_xml_value(tag: str) -> str:
    """Read a top-level element value from the addon's package.xml.

    Returns the element text, or an empty string if it cannot be determined.
    """
    package_xml = MOD_PATH / 'package.xml'
    if not package_xml.exists():
        return ''
    try:
        tree = ET.parse(str(package_xml))
        root = tree.getroot()
        # package.xml may declare a default namespace, so search all elements
        # regardless of namespace.
        for elem in root.iter():
            if elem.tag.rsplit('}', 1)[-1] == tag:
                if elem.text:
                    return elem.text.strip()
    except ET.ParseError:
        pass
    return ''


def get_robotcad_version() -> str:
    """Read the RobotCAD version from the addon's package.xml."""
    return _get_package_xml_value('version')


def get_robotcad_release_date() -> str:
    """Read the RobotCAD release date from the addon's package.xml."""
    return _get_package_xml_value('date')


class AboutDialog(QtWidgets.QDialog):
    """About dialog with version information and a dependencies button."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle(tr('About RobotCAD'))
        self.setModal(False)
        self.resize(420, 200)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(15, 15, 15, 15)

        version = get_robotcad_version() or tr('unknown')
        release_date = get_robotcad_release_date() or tr('unknown')

        title = QtWidgets.QLabel(f'<h2>RobotCAD</h2>')
        layout.addWidget(title)

        info = QtWidgets.QLabel(
            tr('RobotCAD version: {}').format(version) + '<br>' +
            tr('Release date: {}').format(release_date),
        )
        info.setTextFormat(QtCore.Qt.RichText)
        info.setWordWrap(True)
        layout.addWidget(info)

        layout.addStretch(1)

        button_row = QtWidgets.QHBoxLayout()

        self.dependencies_button = QtWidgets.QPushButton(
            tr('Check and install dependencies'),
        )
        self.dependencies_button.clicked.connect(self._on_dependencies_clicked)
        button_row.addWidget(self.dependencies_button)

        button_row.addStretch(1)

        self.close_button = QtWidgets.QPushButton(tr('Close'))
        self.close_button.clicked.connect(self.close)
        button_row.addWidget(self.close_button)

        layout.addLayout(button_row)

    def _on_dependencies_clicked(self) -> None:
        open_dependencies_dialog(parent=self, auto_install=False)


class _AboutCommand:
    """The command definition to show the RobotCAD version."""

    def GetResources(self):
        return {
            'Pixmap': 'about_robotcad.svg',
            'MenuText': tr('About RobotCAD'),
            'Accel': 'W, A',
            'ToolTip': tr('Show RobotCAD version information'),
        }

    def IsActive(self):
        return True

    def Activated(self):
        dialog = AboutDialog(fcgui.getMainWindow())
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose, False)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()


fcgui.addCommand('AboutRobotCAD', _AboutCommand())
