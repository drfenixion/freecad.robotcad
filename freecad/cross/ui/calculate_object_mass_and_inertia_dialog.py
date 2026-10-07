from __future__ import annotations

from typing import Callable

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


def _enum_to_int(value) -> int:
    """Return the integer value of a Qt enum or flag.

    PySide6 enums are Python enums that cannot always be converted with
    `int()`, they have a `value` attribute. PySide2 enums are int-like.

    """
    return int(getattr(value, 'value', value))


class CalculateObjectMassAndInertiaDialog:
    """A task panel to calculate the mass and inertia of the selected object.

    The panel is shown in the "Tasks" tab of the combo view (FreeCAD task
    dialog). It is not modal and stays open until "Close" is pressed: the user
    selects an object and presses "Calculate", as many times as needed. The
    `on_calculate` callback is called with the panel at each click on
    "Calculate"; it must check the selection itself.

    FreeCAD does not store the values of task panels. The last selected values
    are kept in class attributes and used as initial values of the next panel,
    until FreeCAD is closed.

    """

    # Last selected values (initial values of the next panel).
    _last_target_frame: str = FRAME_GLOBAL
    _last_log_level: str = LOG_LEVEL_WARN

    def __init__(
            self,
            on_calculate: Callable[[CalculateObjectMassAndInertiaDialog], None],
    ):
        """Constructor.

        Parameters
        ----------
        - on_calculate: called when the user presses "Calculate". The selected
                        values are in the attributes `target_frame` and
                        `log_level`.

        """
        self._on_calculate = on_calculate

        # The widget shown in the task panel.
        self.form = QtWidgets.QWidget()
        self.form.setWindowTitle(tr('Calculate object mass and inertia'))
        layout = QtWidgets.QFormLayout(self.form)

        hint = QtWidgets.QLabel(tr('Select an object and press "Calculate".'))
        hint.setWordWrap(True)
        layout.addRow(hint)

        self._combo_target_frame = QtWidgets.QComboBox()
        for value in TARGET_FRAMES:
            self._combo_target_frame.addItem(tr(_TARGET_FRAME_TEXTS[value]), value)
        self._combo_target_frame.setCurrentIndex(
            TARGET_FRAMES.index(type(self)._last_target_frame),
        )
        self._combo_target_frame.setToolTip(tr(
            'Frame in which the center of mass and the inertia matrix are expressed:\n'
            '- global: global frame (document root);\n'
            '- local: local frame of the selected object (its Placement is ignored).',
        ))
        layout.addRow(tr('Target frame'), self._combo_target_frame)

        self._combo_log_level = QtWidgets.QComboBox()
        for value in LOG_LEVELS:
            self._combo_log_level.addItem(tr(_LOG_LEVEL_TEXTS[value]), value)
        self._combo_log_level.setCurrentIndex(
            LOG_LEVELS.index(type(self)._last_log_level),
        )
        self._combo_log_level.setToolTip(tr('Console verbosity of the calculation.'))
        layout.addRow(tr('Log level'), self._combo_log_level)

        self._button_calculate = QtWidgets.QPushButton(tr('Calculate'))
        self._button_calculate.setToolTip(
            tr('Calculate inertia properties of selected object'),
        )
        self._button_calculate.clicked.connect(self._on_calculate_clicked)
        layout.addRow(self._button_calculate)

    @property
    def target_frame(self) -> str:
        return self._combo_target_frame.currentData()

    @property
    def log_level(self) -> str:
        return self._combo_log_level.currentData()

    @staticmethod
    def is_task_dialog_active() -> bool:
        """Return True if a task dialog (of any command) is already shown."""
        return bool(fcgui.Control.activeDialog())

    def show(self) -> None:
        """Show the panel in the Tasks tab of the combo view."""
        fcgui.Control.showDialog(self)

    def _on_calculate_clicked(self, checked: bool = False) -> None:
        # Remember the values for the next panel.
        type(self)._last_target_frame = self.target_frame
        type(self)._last_log_level = self.log_level
        self._on_calculate(self)

    # Task dialog interface called by FreeCAD.

    def getStandardButtons(self) -> int:
        # Only "Close", "Calculate" is in the form. "Close" calls `reject()`.
        return _enum_to_int(QtWidgets.QDialogButtonBox.StandardButton.Close)

    def reject(self) -> bool:
        fcgui.Control.closeDialog()
        return True
