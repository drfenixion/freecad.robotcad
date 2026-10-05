from __future__ import annotations

import FreeCADGui as fcgui
from PySide import QtWidgets  # FreeCAD's PySide

from ..gui_utils import tr
from .inertia_calculator import LOG_LEVELS, LOG_LEVEL_INFO, LOG_LEVEL_WARN

# Frames in which the inertial properties of the object can be expressed, in
# the order shown in the dialog (the first one is the default).
#   * FRAME_GLOBAL: global frame (document root).
#   * FRAME_LOCAL: local frame of the object (its Placement is ignored).
FRAME_GLOBAL = 'global'
FRAME_LOCAL = 'local'
TARGET_FRAMES = [FRAME_GLOBAL, FRAME_LOCAL]

# Text shown for the values of the "Target frame" enumeration.
_TARGET_FRAME_TEXTS = {
    FRAME_GLOBAL: 'global (document root)',
    FRAME_LOCAL: 'local (selected object frame)',
}

_LOG_LEVEL_TEXTS = {
    LOG_LEVEL_WARN: 'warn (only warnings)',
    LOG_LEVEL_INFO: 'info (calculation trace and warnings)',
}


class CalculateObjectMassAndInertiaDialog:
    """A dialog to get the parameters of the object mass and inertia calculation.

    The last accepted values are kept for the next dialog of the session.

    """

    # Values of the last accepted dialog (initial values of the next one).
    _last_target_frame: str = FRAME_GLOBAL
    _last_log_level: str = LOG_LEVEL_WARN

    def __init__(self, object_label: str = ''):
        self.target_frame: str = type(self)._last_target_frame
        self.log_level: str = type(self)._last_log_level

        self.form = QtWidgets.QDialog(fcgui.getMainWindow())
        self.form.setWindowTitle(tr('Calculate object mass and inertia'))
        layout = QtWidgets.QFormLayout(self.form)

        if object_label:
            layout.addRow(tr('Object'), QtWidgets.QLabel(object_label))

        self._combo_target_frame = QtWidgets.QComboBox()
        for value in TARGET_FRAMES:
            self._combo_target_frame.addItem(tr(_TARGET_FRAME_TEXTS[value]), value)
        self._combo_target_frame.setCurrentIndex(TARGET_FRAMES.index(self.target_frame))
        self._combo_target_frame.setToolTip(tr(
            'Frame in which the center of mass and the inertia matrix are expressed:\n'
            '- global: global frame (document root);\n'
            '- local: local frame of the selected object (its Placement is ignored).',
        ))
        layout.addRow(tr('Target frame'), self._combo_target_frame)

        self._combo_log_level = QtWidgets.QComboBox()
        for value in LOG_LEVELS:
            self._combo_log_level.addItem(tr(_LOG_LEVEL_TEXTS[value]), value)
        self._combo_log_level.setCurrentIndex(LOG_LEVELS.index(self.log_level))
        self._combo_log_level.setToolTip(tr('Console verbosity of the calculation.'))
        layout.addRow(tr('Log level'), self._combo_log_level)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok
            | QtWidgets.QDialogButtonBox.StandardButton.Cancel,
        )
        buttons.accepted.connect(self.form.accept)
        buttons.rejected.connect(self.form.reject)
        layout.addRow(buttons)

    def exec(self) -> bool:
        """Show the dialog, return True if accepted (OK)."""
        if not self.form.exec():
            return False
        self.target_frame = self._combo_target_frame.currentData()
        self.log_level = self._combo_log_level.currentData()
        type(self)._last_target_frame = self.target_frame
        type(self)._last_log_level = self.log_level
        return True

    def close(self) -> None:
        self.form.close()
