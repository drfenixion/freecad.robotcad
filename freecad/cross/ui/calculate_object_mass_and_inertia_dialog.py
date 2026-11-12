from __future__ import annotations

from typing import Callable, Optional

import FreeCADGui as fcgui
from PySide import QtCore, QtGui, QtWidgets  # FreeCAD's PySide

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

_OBJECT_NAME = 'RobotCadObjectMassAndInertiaCalculator'


class CalculateObjectMassAndInertiaDialog:
    """A non-modal dockable window to calculate the mass and inertia of the selected object.

    Window contains the calculation parameters (target frame and log level),
    the "Calculate" and "Close" buttons and a zone
    where the result of the last calculation is displayed.

    "Calculate" press activates `on_calculate` callback. It checks selection,
    inwokes calulation method and displays its result with `show_result()` or `show_error()`.
    The result zone is only updated on "Calculate".

    "Close" hides window, but the selected parameters and
    the last result are still there when it is shown again, until FreeCAD is
    closed. Use `open()` to show (or create) it.

    """

    _instance: Optional[CalculateObjectMassAndInertiaDialog] = None

    @classmethod
    def open(
            cls,
            on_calculate: Callable[[CalculateObjectMassAndInertiaDialog], None],
    ) -> CalculateObjectMassAndInertiaDialog:
        """Show the window, create it if needed."""
        instance = cls._instance
        if instance is None or not instance._is_alive():
            instance = cls(on_calculate)
            cls._instance = instance
        instance._on_calculate = on_calculate
        instance.show()
        return instance

    def __init__(
            self,
            on_calculate: Callable[[CalculateObjectMassAndInertiaDialog], None],
    ):
        """Constructor, use `open()` to get the single window.

        Parameters
        ----------
        - on_calculate: called when the user presses "Calculate". The selected
                        values are in the attributes `target_frame`,
                        `principal_axes` and `log_level`.

        """
        self._on_calculate = on_calculate

        main_window = fcgui.getMainWindow()
        self.dock = QtWidgets.QDockWidget(tr('Object mass and inertia'), main_window)
        # The name is used by FreeCAD to save the layout of the windows.
        self.dock.setObjectName(_OBJECT_NAME)

        self.form = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout(self.form)

        # Parameters. The fields keep their size hint instead of being
        # stretched to the width of the window.
        parameters = QtWidgets.QFormLayout()
        parameters.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.FieldsStayAtSizeHint)
        self._combo_target_frame = QtWidgets.QComboBox()
        for value in TARGET_FRAMES:
            self._combo_target_frame.addItem(tr(_TARGET_FRAME_TEXTS[value]), value)
        self._combo_target_frame.setToolTip(tr(
            'Frame in which the center of mass and the inertia matrix are expressed:\n'
            '- global: global frame (document root);\n'
            '- local: local frame of the selected object (its Placement is ignored).',
        ))

        self._check_principal_axes = QtWidgets.QCheckBox(tr('Principal axes'))
        self._check_principal_axes.setChecked(False)
        self._check_principal_axes.setToolTip(tr(
            'Express the inertia matrix in the principal axes of inertia\n'
            '(diagonal matrix of the principal moments). The rotation from the\n'
            'target frame axes to the principal axes is shown before the matrix.\n'
            'Unchecked: the matrix is expressed in the axes of the target frame.',
        ))
        # "Principal axes" on the same row as the target frame.
        frame_row = QtWidgets.QHBoxLayout()
        frame_row.addWidget(self._combo_target_frame)
        frame_row.addSpacing(12)
        frame_row.addWidget(self._check_principal_axes)
        frame_row.addStretch(1)
        parameters.addRow(tr('Target frame'), frame_row)

        self._combo_log_level = QtWidgets.QComboBox()
        for value in LOG_LEVELS:
            self._combo_log_level.addItem(tr(_LOG_LEVEL_TEXTS[value]), value)
        self._combo_log_level.setToolTip(tr('Console verbosity of the calculation.'))
        parameters.addRow(tr('Log level'), self._combo_log_level)
        main_layout.addLayout(parameters)

        # Buttons.
        buttons = QtWidgets.QHBoxLayout()
        self._button_calculate = QtWidgets.QPushButton(tr('Calculate'))
        self._button_calculate.setToolTip(
            tr('Calculate inertia properties of selected object'),
        )
        self._button_calculate.clicked.connect(self._on_calculate_clicked)
        self._button_close = QtWidgets.QPushButton(tr('Close'))
        self._button_close.setToolTip(tr('Close this window'))
        self._button_close.clicked.connect(self.close)
        buttons.addWidget(QtWidgets.QLabel(tr('Result'))) # add result header to reduce vertical size
        buttons.addStretch(1)
        buttons.addWidget(self._button_calculate)
        buttons.addWidget(self._button_close)
        main_layout.addLayout(buttons)

        # Result of the last calculation. Fixed pitch font and no line
        # wrapping to keep the columns of the inertia matrix aligned.
        self._result = QtWidgets.QPlainTextEdit()
        self._result.setReadOnly(True)
        self._result.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        self._result.setFont(
            QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont),
        )
        self._result.setPlaceholderText(
            tr('Select an object and press "Calculate".'),
        )
        main_layout.addWidget(self._result, 1)

        self.dock.setWidget(self.form)
        main_window.addDockWidget(QtCore.Qt.DockWidgetArea.RightDockWidgetArea, self.dock)

    @property
    def target_frame(self) -> str:
        return self._combo_target_frame.currentData()

    @property
    def principal_axes(self) -> bool:
        return self._check_principal_axes.isChecked()

    @property
    def log_level(self) -> str:
        return self._combo_log_level.currentData()

    def show(self) -> None:
        """Show the window and bring it to the front."""
        self.dock.show()
        self.dock.raise_()

    def close(self, checked: bool = False) -> None:
        """Hide the window, the parameters and the result are kept."""
        self.dock.close()

    def show_result(self, text: str) -> None:
        """Display the result of a calculation."""
        self._result.setPlainText(text)

    def show_error(self, text: str) -> None:
        """Display why the calculation could not be done."""
        self._result.setPlainText(f'{tr("Error")}: {text}')

    def _is_alive(self) -> bool:
        """Return False if the Qt window has been deleted (e.g. workbench reload)."""
        try:
            self.dock.objectName()
        except RuntimeError:
            return False
        return True

    def _on_calculate_clicked(self, checked: bool = False) -> None:
        self._on_calculate(self)
